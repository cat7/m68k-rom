/*
 * Windowed calibration governor.
 *
 * Mac ROMs calibrate the TimeDBRA family of low-memory globals
 * (SetUpTimeK) by arming VIA Timer 2 as a ~1 ms one-shot (780 ticks)
 * and spinning `dbra d0,self` with d0 = 0xFFFF, expecting the T2
 * interrupt to break the loop part-way through. The spin is a pure
 * register loop, so under TCG it runs at host speed and completes all
 * 65536 iterations before the timer expires. It then falls through and
 * the ROM stores not.w(0xFFFF) == 0 into TimeDBRA (0x0D00); the next
 * division by it is a divide-by-zero Sad Mac or system error.
 *
 * Upstream QEMU works around this in mac_via.c by recognising the
 * calibration's VIA writes and storing fixed values afterwards. That
 * depends on seeing exactly the expected sequence (it missed the LC 475's
 * Restart) and gives every machine the same numbers.
 *
 * This instead paces the CPU to a period-plausible rate while, and only
 * while, a calibration window is open, so the ROM measures sensible
 * values itself. Ported from the G3 tree, where it replaced a
 * whole-session MIPS cap. Three parts, all load-bearing:
 *
 * 1. PACING. A fine settle quantum (MAX_QUANTUM_INSNS) so the vCPU
 *    cannot burst through the whole ~1 ms window at host speed; a cap on
 *    banked credit (MAX_CREDIT_NS); the host MONOTONIC clock; and debts
 *    below a millisecond settled by busy-waiting, since a sleep of a few
 *    microseconds overshoots by an order of magnitude and the paced rate
 *    must be exact for the result to be host independent.
 *
 * 2. FORCED TRANSLATION-BLOCK EXITS. `dbra d0,self` chains to itself via
 *    goto_tb and would never return to the execution loop, so while a
 *    window is open curr_cflags() adds CF_NO_GOTO_TB | CF_NO_GOTO_PTR
 *    (NO_GOTO_TB alone still chains through the jump cache) and opening
 *    the window kicks every vCPU. Blocks are keyed on cflags, so the
 *    non-chaining copies are translated once and reused.
 *
 * 3. A SPIN DETECTOR. "T2 armed as a one-shot of 100 us to 2 ms" also
 *    matches ordinary Time Manager delays. A calibration window executes
 *    a handful of tiny blocks and nothing else, so over each 256-block
 *    epoch the distinct block addresses are counted; more than
 *    CALIB_GOV_SPIN_DISTINCT and the window stops pacing (it stays open,
 *    so a re-arm looks again). An epoch spoilt by the window's own setup
 *    code costs a re-sync, not the window.
 *
 * 4. DELAY LOOPS. Mac OS also times `dbra dN,self` loops of TimeDBRA
 *    iterations against a Time Manager task (System 7.5's speed check
 *    runs four and divides by the microseconds they took). At host speed
 *    they take no measurable time and the check divides by zero. Such a
 *    loop is one instruction branching to itself, so it can only be a
 *    delay: the translator calls calib_governor_delay_insn() on each pass
 *    and, while a VIA T2 one-shot is pending, the loop runs at the paced
 *    rate. Other code, and delay loops with no timer running, are not
 *    touched.
 *
 * Under icount the guest's clock follows its instruction count, so the
 * ROM already measures a deterministic value and pacing against the
 * host clock would only slow it down: the governor stands aside.
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 */

#include "qemu/osdep.h"
#include "qemu/atomic.h"
#include "qemu/main-loop.h"
#include "qemu/error-report.h"
#include "qemu/timer.h"
#include "qemu/cutils.h"
#include "qapi/error.h"
#include "qom/object.h"
#include "hw/core/cpu.h"
#include "exec/icount.h"
#include "system/calib-governor.h"

bool calib_gov_window_open;

#define NSEC_IN_ONE_SEC (1000LL * 1000 * 1000)

/* Rate-check granularity, and the hard bound on a single burst. */
#define NUM_TIME_UPDATE_PER_SEC 8000
#define MAX_QUANTUM_INSNS       2000

/* Debts below this are settled by busy-waiting (see throttle_sleep()). */
#define SLEEP_GRANULARITY_NS (1000 * 1000)
#define MAX_CREDIT_NS        (100 * 1000)

/*
 * Keep pacing this long past the timer's expiry, so the interrupt and
 * the few instructions the ROM runs after the loop breaks are covered.
 */
#define CALIB_GOV_GUARD_NS (300 * 1000)

/*
 * One-shots that can be a calibration: the ROM's own arm is ~996 us.
 * Shorter ones are handshake timeouts, longer ones Time Manager delays.
 */
#define CALIB_GOV_MAX_ARM_NS (2 * 1000 * 1000)
#define CALIB_GOV_MIN_ARM_NS (100 * 1000)

/* Hard cap on one window, so back-to-back re-arms can't pace forever. */
#define CALIB_GOV_MAX_WINDOW_NS (10 * 1000 * 1000)

/*
 * Spin detector (thresholds measured on the G3, where the ROM's spins
 * run 4 to 8 distinct blocks per epoch and ordinary code ~200; on a 68K
 * the dbra spin is a single block).
 */
#define CALIB_GOV_EPOCH_BLOCKS  256
#define CALIB_GOV_PROBE_NS      (50 * 1000)
#define CALIB_GOV_DET_SLOTS     32      /* > SPIN_DISTINCT: overflow rejects */
#define CALIB_GOV_SPIN_DISTINCT 12
#define CALIB_GOV_RESYNC_TRIES  8

/* Warn when the share of wall time spent pacing stays high. */
#define CALIB_GOV_WARN_BUCKET_NS (15LL * 1000 * 1000 * 1000)
#define CALIB_GOV_WARN_SPAN_NS   (2 * CALIB_GOV_WARN_BUCKET_NS)
#define CALIB_GOV_WARN_PCT       5

#define CALIB_GOV_MAX_CPUS 16

typedef struct CalibGovVCPU {
    uint64_t quantum_insn;
    uint64_t window_insn;
    int64_t window_start_ns;
    /* delay-loop pacing */
    uint64_t delay_insn;
    int64_t delay_start_ns;
    int64_t delay_last_ns;
    /* polling-loop pacing (coprocessor busy) */
    uint64_t poll_insn;
    int64_t poll_start_ns;
    int64_t poll_last_ns;
} CalibGovVCPU;

static CalibGovVCPU gov_vcpu[CALIB_GOV_MAX_CPUS];

/* Off until a board asks; an explicit property setting always wins. */
static bool gov_enabled;
static bool gov_explicit;
static bool gov_explicit_mips;
static uint64_t gov_insn_per_second = 32 * 1000 * 1000;
static uint64_t gov_insn_per_quantum = MAX_QUANTUM_INSNS;

/* On-time interrupt delivery for the governed one-shot. */
static void (*gov_irq_cb)(void *opaque);
static void *gov_irq_opaque;
static int64_t gov_irq_due_ns;          /* host monotonic, one-shot expiry */
static int64_t gov_oneshot_due_ns;      /* host monotonic, any T2 one-shot */
static bool gov_coproc_busy;            /* a coprocessor works for the CPU */
static bool gov_irq_delivered;

/* Window state, written with the BQL held. */
static QEMUTimer *gov_close_timer;
static QEMUTimer *gov_probe_timer;
static bool gov_window_active;          /* a window is open (maybe unpaced) */
static int64_t gov_window_open_ns;      /* host monotonic, window start */
static int64_t gov_window_deadline_ns;  /* host monotonic, window end */
static int64_t gov_paced_since_ns;      /* host monotonic, pacing armed at */
static bool gov_warned;

static int64_t gov_warn_bucket_index = -1;
static uint64_t gov_warn_bucket_ns[2];

/* Spin detector state, written by the paced vCPU. */
typedef struct CalibGovDetSlot {
    uint64_t pc;
    uint32_t count;
} CalibGovDetSlot;

static CalibGovDetSlot gov_det[CALIB_GOV_DET_SLOTS];
static uint32_t gov_det_used;
static uint32_t gov_det_blocks;
static uint32_t gov_det_tries;          /* re-syncs left before giving up */
static bool gov_det_overflow;

static CalibGovStats gov_stats;

static int64_t now_ns(void)
{
    return g_get_monotonic_time() * 1000;
}

static void calib_governor_kick_all(void)
{
    CPUState *cpu;

    CPU_FOREACH(cpu) {
        cpu_exit(cpu);
    }
}

static void calib_governor_note_paced(int64_t now, int64_t paced_ns)
{
    int64_t bucket = now / CALIB_GOV_WARN_BUCKET_NS;
    uint64_t sum;

    if (bucket != gov_warn_bucket_index) {
        if (bucket == gov_warn_bucket_index + 1) {
            gov_warn_bucket_ns[1] = gov_warn_bucket_ns[0];
        } else {
            gov_warn_bucket_ns[1] = 0;
        }
        gov_warn_bucket_ns[0] = 0;
        gov_warn_bucket_index = bucket;
    }
    gov_warn_bucket_ns[0] += (uint64_t)paced_ns;

    if (gov_warned) {
        return;
    }
    sum = gov_warn_bucket_ns[0] + gov_warn_bucket_ns[1];
    if (sum * 100 > (uint64_t)CALIB_GOV_WARN_SPAN_NS * CALIB_GOV_WARN_PCT) {
        gov_warned = true;
        warn_report("calibration-governor: paced %" PRIu64 " ms of the last "
                    "%" PRId64 " s across %" PRIu64 " windows -- the guest "
                    "keeps arming the VIA one-shot timer and this has become "
                    "a general throttle", sum / 1000000,
                    CALIB_GOV_WARN_SPAN_NS / 1000000000,
                    qatomic_read(&gov_stats.windows));
    }
}

/*
 * Stop pacing. Clearing the global is enough: blocks are keyed on
 * cflags, so the next lookup finds the ordinary chained copy.
 */
static void calib_governor_unpace(int64_t now)
{
    int64_t paced;

    if (!qatomic_xchg(&calib_gov_window_open, false)) {
        return;
    }
    paced = now - gov_paced_since_ns;
    if (paced < 0) {
        paced = 0;
    }
    qatomic_add(&gov_stats.governed_ns, (uint64_t)paced);
    calib_governor_note_paced(now, paced);
}

/* Arm (or re-arm) pacing for the open window, starting a fresh epoch. */
static void calib_governor_pace(int64_t now)
{
    int i;

    for (i = 0; i < CALIB_GOV_MAX_CPUS; i++) {
        gov_vcpu[i].quantum_insn = 0;
        gov_vcpu[i].window_insn = 0;
        gov_vcpu[i].window_start_ns = now;
    }
    memset(gov_det, 0, sizeof(gov_det));
    gov_det_used = 0;
    gov_det_blocks = 0;
    gov_det_tries = CALIB_GOV_RESYNC_TRIES;
    gov_det_overflow = false;
    gov_paced_since_ns = now;

    qatomic_set(&calib_gov_window_open, true);

    /* Break any running block chain so the new cflags take effect. */
    calib_governor_kick_all();
}

/*
 * Deliver the governed one-shot's interrupt if its expiry has passed on
 * the host clock and it has not been delivered yet -- never early. May
 * be called from a vCPU thread (no BQL) or the main loop (BQL held).
 */
static void calib_governor_irq_catch_up(int64_t now)
{
    void (*cb)(void *opaque);
    void *opaque;
    bool locked;

    if (!qatomic_read(&gov_irq_cb) || gov_irq_delivered ||
        now < gov_irq_due_ns) {
        return;
    }

    /* Whatever retires the callback holds the BQL: recheck under it. */
    locked = bql_locked();
    if (!locked) {
        bql_lock();
    }
    cb = qatomic_read(&gov_irq_cb);
    opaque = gov_irq_opaque;
    if (cb && !gov_irq_delivered && now >= gov_irq_due_ns) {
        gov_irq_delivered = true;
        cb(opaque);
        qatomic_inc(&gov_stats.irq_catch_ups);
    }
    if (!locked) {
        bql_unlock();
    }
}

static void calib_governor_close(void)
{
    int64_t now = now_ns();

    /* Last chance before pacing ends, or the spin runs out at host speed. */
    calib_governor_irq_catch_up(now);

    calib_governor_unpace(now);
    gov_window_active = false;
    qatomic_set(&gov_irq_cb, NULL);
    if (gov_probe_timer) {
        timer_del(gov_probe_timer);
    }
}

static void calib_governor_close_timer_cb(void *opaque)
{
    calib_governor_close();
}

/*
 * The window is still open but the last epoch did not look like a spin:
 * pace one more epoch, so a spin starting part-way through is caught.
 */
static void calib_governor_probe_timer_cb(void *opaque)
{
    int64_t now = now_ns();

    if (!gov_window_active || qatomic_read(&calib_gov_window_open)) {
        return;
    }
    if (now >= gov_window_deadline_ns) {
        calib_governor_close();
        return;
    }
    qatomic_inc(&gov_stats.probes);
    calib_governor_pace(now);
}

bool calib_governor_enabled(void)
{
    return gov_enabled && !icount_enabled();
}

bool calib_governor_arm(int64_t countdown_ns,
                        void (*irq_catch_up)(void *opaque), void *opaque)
{
    int64_t now, deadline;

    if (!calib_governor_enabled()) {
        return false;
    }
    /* any one-shot up to a second paces delay loops while it runs */
    if (countdown_ns > 0 && countdown_ns <= NSEC_IN_ONE_SEC) {
        qatomic_set(&gov_oneshot_due_ns, now_ns() + countdown_ns);
    }
    /* Also disposes of the INT64_MAX a stopped timer produces. */
    if (countdown_ns > CALIB_GOV_MAX_ARM_NS ||
        countdown_ns < CALIB_GOV_MIN_ARM_NS) {
        qatomic_inc(&gov_stats.ignored);
        return false;
    }

    now = now_ns();
    deadline = now + countdown_ns + CALIB_GOV_GUARD_NS;

    /* The catch-up target is always the latest arm's expiry. */
    gov_irq_opaque = opaque;
    gov_irq_due_ns = now + countdown_ns;
    gov_irq_delivered = false;
    qatomic_set(&gov_irq_cb, irq_catch_up);

    if (gov_window_active) {
        /* Extend, but never past the cap measured from the first open. */
        int64_t hard = gov_window_open_ns + CALIB_GOV_MAX_WINDOW_NS;

        qatomic_inc(&gov_stats.rearms);
        if (deadline > hard) {
            deadline = hard;
            qatomic_inc(&gov_stats.capped);
        }
        if (deadline > gov_window_deadline_ns) {
            gov_window_deadline_ns = deadline;
            timer_mod_ns(gov_close_timer,
                         qemu_clock_get_ns(QEMU_CLOCK_REALTIME) +
                         (deadline - now));
        }
        /* A fresh arm is fresh evidence: look again straight away. */
        if (!qatomic_read(&calib_gov_window_open)) {
            qatomic_inc(&gov_stats.probes);
            calib_governor_pace(now);
        }
        return true;
    }

    gov_window_active = true;
    gov_window_open_ns = now;
    gov_window_deadline_ns = deadline;
    qatomic_inc(&gov_stats.windows);

    if (!gov_close_timer) {
        gov_close_timer = timer_new_ns(QEMU_CLOCK_REALTIME,
                                       calib_governor_close_timer_cb, NULL);
        gov_probe_timer = timer_new_ns(QEMU_CLOCK_REALTIME,
                                       calib_governor_probe_timer_cb, NULL);
    }
    timer_mod_ns(gov_close_timer,
                 qemu_clock_get_ns(QEMU_CLOCK_REALTIME) + (deadline - now));

    /* Open optimistically: the spin starts on the window's first block. */
    calib_governor_pace(now);
    return true;
}

/*
 * Fold one executed block into the current epoch. Returns false only
 * once the re-sync budget for this arm is spent.
 */
static bool calib_governor_looks_like_spin(uint64_t pc)
{
    uint32_t h = (uint32_t)((pc >> 1) * 2654435761u) % CALIB_GOV_DET_SLOTS;
    uint32_t i;

    gov_det_blocks++;

    if (!gov_det_overflow) {
        for (i = 0; i < CALIB_GOV_DET_SLOTS; i++) {
            CalibGovDetSlot *s = &gov_det[(h + i) % CALIB_GOV_DET_SLOTS];

            if (s->count == 0) {
                s->pc = pc;
                s->count = 1;
                gov_det_used++;
                break;
            }
            if (s->pc == pc) {
                s->count++;
                break;
            }
        }
        if (i == CALIB_GOV_DET_SLOTS) {
            gov_det_overflow = true;
        }
    }

    if (gov_det_overflow || gov_det_used > CALIB_GOV_SPIN_DISTINCT) {
        if (gov_det_tries == 0) {
            return false;
        }
        gov_det_tries--;
        goto resync;
    }
    if (gov_det_blocks < CALIB_GOV_EPOCH_BLOCKS) {
        return true;
    }
    /* A clean full epoch: this is a spin, restore the re-sync budget. */
    gov_det_tries = CALIB_GOV_RESYNC_TRIES;

resync:
    memset(gov_det, 0, sizeof(gov_det));
    gov_det_used = 0;
    gov_det_blocks = 0;
    gov_det_overflow = false;
    return true;
}

static void throttle_sleep(int64_t debt_ns)
{
    if (debt_ns < SLEEP_GRANULARITY_NS) {
        int64_t until = now_ns() + debt_ns;

        while (now_ns() < until) {
            /* busy-wait: precise where a short sleep is not */
        }
        return;
    }
#ifdef _WIN32
    g_usleep((debt_ns / SLEEP_GRANULARITY_NS) * 1000);
#else
    g_usleep(debt_ns / 1000);
#endif
}

void calib_governor_account(CPUState *cpu, uint64_t pc, unsigned int n_insns)
{
    CalibGovVCPU *v;
    int64_t now, due_ns, elapsed_ns;
    unsigned index = cpu->cpu_index;

    if (unlikely(index >= CALIB_GOV_MAX_CPUS)) {
        return;
    }
    v = &gov_vcpu[index];

    if (unlikely(!calib_governor_looks_like_spin(pc))) {
        /* Ordinary code inside the window: stop pacing it. */
        qatomic_inc(&gov_stats.ungoverned);
        calib_governor_unpace(now_ns());
        if (gov_probe_timer) {
            timer_mod_ns(gov_probe_timer,
                         qemu_clock_get_ns(QEMU_CLOCK_REALTIME) +
                         CALIB_GOV_PROBE_NS);
        }
        return;
    }

    v->quantum_insn += n_insns;
    if (v->quantum_insn < gov_insn_per_quantum) {
        return;
    }

    now = now_ns();
    v->window_insn += v->quantum_insn;
    qatomic_add(&gov_stats.insns, v->quantum_insn);
    v->quantum_insn = 0;

    /* The one-shot may be due while its main-loop timer has not run. */
    calib_governor_irq_catch_up(now);

    /* How long should this window's instructions have taken? */
    due_ns = (int64_t)((double)v->window_insn /
                       (double)gov_insn_per_second *
                       (double)NSEC_IN_ONE_SEC);
    elapsed_ns = now - v->window_start_ns;

    if (due_ns > elapsed_ns) {
        int64_t after;

        throttle_sleep(due_ns - elapsed_ns);
        after = now_ns();
        qatomic_add(&gov_stats.slept_ns, (uint64_t)(after - now));
        now = after;
    } else if (elapsed_ns - due_ns > MAX_CREDIT_NS) {
        /* Running below target: don't bank the excess as burst budget. */
        v->window_start_ns = now - due_ns - MAX_CREDIT_NS;
    }

    if (now - v->window_start_ns > NSEC_IN_ONE_SEC / 4) {
        v->window_start_ns = now;
        v->window_insn = 0;
    }

    /* The vCPU closes the window itself; the close timer is the backstop. */
    if (now >= gov_window_deadline_ns) {
        calib_governor_close();
    }
}

/* checked every so many passes, so the clock is read rarely */
#define CALIB_GOV_DELAY_CHECK   64
/* a pause longer than this starts a new loop */
#define CALIB_GOV_DELAY_GAP_NS  (1000 * 1000)

void calib_governor_set_coproc_busy(bool busy)
{
    qatomic_set(&gov_coproc_busy, busy);
}

void calib_governor_poll_loop(CPUState *cpu, unsigned insns)
{
    CalibGovVCPU *v;
    int64_t now, due_ns, elapsed_ns;
    unsigned index = cpu->cpu_index;

    if (!gov_enabled || !qatomic_read(&gov_coproc_busy) ||
        unlikely(index >= CALIB_GOV_MAX_CPUS) || icount_enabled()) {
        return;
    }
    v = &gov_vcpu[index];
    v->poll_insn += insns;
    if (v->poll_insn % (CALIB_GOV_DELAY_CHECK * 4) >= insns) {
        return;
    }
    now = now_ns();
    if (now - v->poll_last_ns > CALIB_GOV_DELAY_GAP_NS) {
        v->poll_start_ns = now;
        v->poll_insn = 0;
    }
    due_ns = (int64_t)((double)v->poll_insn / (double)gov_insn_per_second *
                       (double)NSEC_IN_ONE_SEC);
    elapsed_ns = now - v->poll_start_ns;
    if (due_ns > elapsed_ns) {
        throttle_sleep(due_ns - elapsed_ns);
        now = now_ns();
    }
    v->poll_last_ns = now;
}

void calib_governor_delay_insn(CPUState *cpu)
{
    CalibGovVCPU *v;
    int64_t now, due_ns, elapsed_ns;
    unsigned index = cpu->cpu_index;

    if (!gov_enabled || unlikely(index >= CALIB_GOV_MAX_CPUS)) {
        return;
    }
    v = &gov_vcpu[index];
    if (++v->delay_insn % CALIB_GOV_DELAY_CHECK) {
        return;
    }
    if (icount_enabled()) {
        return;
    }
    now = now_ns();
    if (now >= qatomic_read(&gov_oneshot_due_ns)) {
        v->delay_last_ns = 0;
        return;
    }
    if (now - v->delay_last_ns > CALIB_GOV_DELAY_GAP_NS) {
        v->delay_start_ns = now;
        v->delay_insn = 0;
    }
    qatomic_add(&gov_stats.delay_insns, CALIB_GOV_DELAY_CHECK);

    due_ns = (int64_t)((double)v->delay_insn / (double)gov_insn_per_second *
                       (double)NSEC_IN_ONE_SEC);
    elapsed_ns = now - v->delay_start_ns;
    if (due_ns > elapsed_ns) {
        int64_t after;

        throttle_sleep(due_ns - elapsed_ns);
        after = now_ns();
        qatomic_add(&gov_stats.delay_slept_ns, (uint64_t)(after - now));
        now = after;
    }
    v->delay_last_ns = now;
}

void calib_governor_get_stats(CalibGovStats *out)
{
    out->windows = qatomic_read(&gov_stats.windows);
    out->rearms = qatomic_read(&gov_stats.rearms);
    out->capped = qatomic_read(&gov_stats.capped);
    out->ignored = qatomic_read(&gov_stats.ignored);
    out->ungoverned = qatomic_read(&gov_stats.ungoverned);
    out->probes = qatomic_read(&gov_stats.probes);
    out->governed_ns = qatomic_read(&gov_stats.governed_ns);
    out->insns = qatomic_read(&gov_stats.insns);
    out->slept_ns = qatomic_read(&gov_stats.slept_ns);
    out->irq_catch_ups = qatomic_read(&gov_stats.irq_catch_ups);
    out->delay_insns = qatomic_read(&gov_stats.delay_insns);
    out->delay_slept_ns = qatomic_read(&gov_stats.delay_slept_ns);
}

static void calib_governor_set_mips(uint64_t mips)
{
    gov_insn_per_second = mips * 1000 * 1000;
    gov_insn_per_quantum = gov_insn_per_second / NUM_TIME_UPDATE_PER_SEC;
    if (gov_insn_per_quantum > MAX_QUANTUM_INSNS) {
        gov_insn_per_quantum = MAX_QUANTUM_INSNS;
    }
    if (gov_insn_per_quantum < 1) {
        gov_insn_per_quantum = 1;
    }
}

void calib_governor_default_on(uint64_t mips)
{
    if (!gov_explicit_mips) {
        calib_governor_set_mips(mips);
    }
    if (!gov_explicit) {
        gov_enabled = true;
    }
}

static bool calib_governor_configure_one(const char *value, Error **errp)
{
    if (!g_strcmp0(value, "on")) {
        gov_explicit = true;
        gov_enabled = true;
    } else if (!g_strcmp0(value, "off")) {
        gov_explicit = true;
        gov_enabled = false;
        calib_governor_close();
    } else if (g_str_has_prefix(value, "mips=")) {
        const char *arg = value + strlen("mips=");
        uint64_t mips;

        if (qemu_strtou64(arg, NULL, 10, &mips) < 0 || mips == 0 ||
            mips > 100000) {
            error_setg(errp, "calibration-governor: bad mips value '%s'", arg);
            return false;
        }
        gov_explicit = true;
        gov_explicit_mips = true;
        gov_enabled = true;
        calib_governor_set_mips(mips);
    } else {
        error_setg(errp, "calibration-governor: expected 'on', 'off' or "
                         "'mips=<n>', got '%s'", value);
        return false;
    }
    return true;
}

/* A comma-separated list, so what the property reads back can be set. */
bool calib_governor_configure(const char *value, Error **errp)
{
    g_auto(GStrv) parts = g_strsplit(value, ",", -1);
    int i;

    for (i = 0; parts[i]; i++) {
        if (!calib_governor_configure_one(parts[i], errp)) {
            return false;
        }
    }
    return true;
}

char *calib_governor_get_config(void)
{
    if (!gov_enabled) {
        return g_strdup("off");
    }
    return g_strdup_printf("mips=%" PRIu64, gov_insn_per_second / 1000000);
}

static char *calib_governor_prop_get(Object *obj, Error **errp)
{
    return calib_governor_get_config();
}

static void calib_governor_prop_set(Object *obj, const char *value,
                                    Error **errp)
{
    calib_governor_configure(value, errp);
}

void calib_governor_add_machine_property(ObjectClass *oc)
{
    object_class_property_add_str(oc, "calibration-governor",
                                  calib_governor_prop_get,
                                  calib_governor_prop_set);
    object_class_property_set_description(oc, "calibration-governor",
        "Pace the CPU while the ROM times its delay loops: "
        "on, off (fixed values instead) or mips=<n>");
}

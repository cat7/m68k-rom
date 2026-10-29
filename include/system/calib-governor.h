/*
 * Windowed calibration governor -- pace guest execution only while a
 * one-shot VIA timer calibration window is open.
 *
 * See system/calib-governor.c for the rationale.
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 */
#ifndef SYSTEM_CALIB_GOVERNOR_H
#define SYSTEM_CALIB_GOVERNOR_H

#include "qemu/atomic.h"

typedef struct CPUState CPUState;

/*
 * True while the vCPU is being paced: a calibration window is open and
 * the spin detector has not ruled it out. Read once per TCG execution
 * loop iteration, so it is a plain global. Never write it directly.
 */
extern bool calib_gov_window_open;

static inline bool calib_governor_active(void)
{
    return unlikely(qatomic_read(&calib_gov_window_open));
}

/*
 * Device side. Called (BQL held, from an MMIO write) when a one-shot
 * timer has just been armed to expire in @countdown_ns. Returns true if
 * a window was opened or extended.
 *
 * @irq_catch_up, if non-NULL, is invoked (BQL held) from the paced vCPU
 * once the one-shot's expiry has passed on the host clock: the spin is a
 * pure register loop, so nothing else would deliver the interrupt before
 * the main loop gets round to it. The callback must recheck its own
 * state, as it can run late or after a re-arm.
 */
bool calib_governor_arm(int64_t countdown_ns,
                        void (*irq_catch_up)(void *opaque), void *opaque);

/*
 * Accelerator side. Called from the TCG execution loop after each
 * translation block, only while calib_governor_active(). @pc is the
 * block that just ran, for the spin detector.
 */
void calib_governor_account(CPUState *cpu, uint64_t pc, unsigned int n_insns);

/*
 * True when the governor, rather than a fixed-value workaround, is in
 * charge of the calibration. Under icount the guest's clock follows its
 * instruction count, so the ROM measures a deterministic value itself
 * and the governor stays out of the way.
 */
bool calib_governor_enabled(void);

/*
 * Opt a board in, from its machine init. @mips is the board's pacing
 * rate. An explicit "calibration-governor=" always wins.
 */
void calib_governor_default_on(uint64_t mips);

/* Add the "calibration-governor" property to a machine class. */
void calib_governor_add_machine_property(ObjectClass *oc);

/* Machine-property plumbing: "on" | "off" | "mips=<n>". */
bool calib_governor_configure(const char *value, Error **errp);
char *calib_governor_get_config(void);

/* Statistics, for "info via". */
typedef struct CalibGovStats {
    uint64_t windows;       /* windows opened */
    uint64_t rearms;        /* re-arms that extended an open window */
    uint64_t capped;        /* windows truncated by CALIB_GOV_MAX_WINDOW_NS */
    uint64_t ignored;       /* one-shots too short or long to be a calibration */
    uint64_t ungoverned;    /* epochs the spin detector refused to pace */
    uint64_t probes;        /* pacing re-armed to look again mid-window */
    uint64_t governed_ns;   /* total real time actually spent pacing */
    uint64_t insns;         /* guest instructions executed inside windows */
    uint64_t slept_ns;      /* real time spent waiting to enforce the rate */
    uint64_t irq_catch_ups; /* overdue one-shot IRQs delivered inline */
} CalibGovStats;

void calib_governor_get_stats(CalibGovStats *out);

#endif /* SYSTEM_CALIB_GOVERNOR_H */

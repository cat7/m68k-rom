/*
 * dsp3210_emu.c — reference emulator core for the AT&T DSP3210
 *
 * See dsp3210_emu.h for provenance and scope.  Section references below
 * ([IM §x.y], instruction page names) are to the AT&T DSP3210 Information
 * Manual; [DOC §x] cites the hardware-verified notes the core was
 * developed against.
 *
 * The interpreter is deliberately the simplest possible shape: one big
 * switch on the 6-bit top-level opcode, mirroring the reference
 * disassembler's decode exactly.
 *
 * This file is the shared core: decode, the CAU (integer/branch/move)
 * instructions, the exception model, the memory map and the host
 * interface.  The DAU (floating-point) implementation is textually
 * included below — exactly one of:
 *
 *   dsp3210_dau_double.inc   approximate, host doubles      (default)
 *   dsp3210_dau_exact.inc    exact 40-bit integer model
 *                                            (-DDSP3210_DAU_EXACT)
 *
 * Both provide the same functions (the DSP32 codec, the DA execution
 * entry points, the accumulator accessors); the build compiles this
 * file twice to produce the two cores in one translation unit each,
 * so every internal helper stays static.
 */

#include "dsp3210_emu.h"

#include <math.h>
#include <string.h>
/* ------------------------------------------------------------------ */
/* field helpers                                                       */

static uint32_t bits(uint32_t w, int hi, int lo)
{
    return (w >> lo) & ((hi - lo == 31) ? 0xFFFFFFFFu
                                        : ((1u << (hi - lo + 1)) - 1u));
}

static int32_t sext16(uint32_t v) { return (int32_t)(int16_t)(v & 0xFFFFu); }

/* ------------------------------------------------------------------ */
/* CAU register-code mapping (IM Table 10-2; deliberately discontinuous:
 * pc = 15, r15-r19 = 17-21, -n/+n = 22/23, r20-r22 = 24-26, pcsh = 30) */

enum { RK_ZERO, RK_REG, RK_PC, RK_PCSH, RK_PLUS, RK_MINUS, RK_RES };

typedef struct { uint8_t kind; uint8_t idx; } regcode;

static regcode rc_decode(unsigned code)
{
    regcode rc = { RK_RES, 0 };
    code &= 31;
    if (code == 0)                       { rc.kind = RK_ZERO; }
    else if (code <= 14)                 { rc.kind = RK_REG; rc.idx = (uint8_t)code; }
    else if (code == 15)                 { rc.kind = RK_PC; }
    else if (code >= 17 && code <= 21)   { rc.kind = RK_REG; rc.idx = (uint8_t)(code - 2); }
    else if (code == 22)                 { rc.kind = RK_MINUS; }
    else if (code == 23)                 { rc.kind = RK_PLUS; }
    else if (code >= 24 && code <= 26)   { rc.kind = RK_REG; rc.idx = (uint8_t)(code - 4); }
    else if (code == 30)                 { rc.kind = RK_PCSH; }
    return rc;
}

/*
 * Operand read.  pc reads the address of the instruction after the
 * latent instruction, i.e. insn_addr + 8 [IM CALL page; DOC §1.5.4].
 * The -n/+n pseudo-operands read as -1/+1 in ALU source positions
 * (INCR/DECR pages); the sp-bump ±4 special case is handled by the ALU.
 * Reserved codes read as 0 (hardware behaviour undocumented).
 */
static uint32_t opval(dsp3210_emu *s, unsigned code)
{
    regcode rc = rc_decode(code);
    switch (rc.kind) {
    case RK_REG:   return s->r[rc.idx];
    case RK_PC:    return s->cur_insn + 8;
    case RK_PCSH:  return s->pcsh;
    case RK_PLUS:  return 1;
    case RK_MINUS: return 0xFFFFFFFFu;
    default:       return 0;   /* r0 / reserved */
    }
}

/*
 * Register write.  r0, the pseudo-operands and reserved codes discard.
 * pcsh is writable (the boot ROM's SAR path depends on it).  A write to
 * pc is not in any replacement table; we implement it as an immediate
 * branch (documented deviation for an undefined encoding).
 */
static void regw(dsp3210_emu *s, unsigned code, uint32_t v)
{
    regcode rc = rc_decode(code);
    switch (rc.kind) {
    case RK_REG:  s->r[rc.idx] = v; break;
    case RK_PC:   s->npc = v; break;
    case RK_PCSH: s->pcsh = v; break;
    default:      break;
    }
}

/* ------------------------------------------------------------------ */
/* exceptions [IM §7.5]                                                */

static int raw_access(dsp3210_emu *s, uint32_t addr, int size,
                      uint32_t *inout, int write);
static void da_shadow_overlay(dsp3210_emu *s, uint32_t addr, int size,
                              uint32_t *val);

/* Raise an exception.  Returns 1 if the exception was actually taken
 * (the current instruction must abort), 0 if it was masked or ignored
 * (the instruction proceeds — see the address-error note in mem_read). */
static int take_error(dsp3210_emu *s, int vn)
{
    s->last_vector = vn;
    if (vn >= 4 && !(s->emr & (1u << vn)))
        return 0;                            /* masked → no exception */
    if (s->level == DSP3210_LVL_DERROR)
        return 1;
    if (s->level == DSP3210_LVL_ERROR) {
        if (vn >= 4)
            return 0;                        /* maskable at error level:
                                                ignored [IM §7.5.2] */
        if (s->error_nonmaskable) {          /* non-maskable during
                                                non-maskable → double */
            s->level = DSP3210_LVL_DERROR;
            return 1;
        }
    }
    /* abort model: kill do loops, disable interrupts, unlock pcw
     * [IM §7.5.2, Table 7-3] */
    s->do_active = 0;
    s->waiting = 0;
    s->error_nonmaskable = (vn < 4);
    s->level = DSP3210_LVL_ERROR;
    s->pcw &= (uint16_t)~(1u << 13);
    s->pcw_locked = 0;
    /* dispatch = call evtp + vn*8 (r20); r20 gets the pc, which is the
     * address of the prefetched instruction (= aborted insn + 8, the
     * same value pc reads as an operand) */
    s->r[20] = s->cur_insn + 8;
    s->pc  = s->r[22] + (uint32_t)vn * 8;
    s->npc = s->pc + 4;
    return 1;
}

static int pending_vector(dsp3210_emu *s)
{
    unsigned p = s->pending & s->emr & 0xFF00u;
    int vn;
    if (!p)
        return 0;
    for (vn = 8; vn <= 15; vn++)             /* EXT0 (8) highest priority */
        if (p & (1u << vn))
            return vn;
    return 0;
}

static void take_interrupt(dsp3210_emu *s, int vn)
{
    s->last_vector = vn;
    s->pending &= (uint16_t)~(1u << vn);
    /* hardware context save [IM §7.5.1]: a0-a3 + dauc (+ ps/ctr per
     * Figure 4-1) + invisible do-loop state; resume address in pcsh */
    s->sh_ps = s->ps;
    s->sh_dauc = s->dauc;
    s->sh_ctr = s->ctr;
    memcpy(s->sh_a, s->a, sizeof s->a);
    memcpy(s->sh_a_pipe, s->a_pipe, sizeof s->a_pipe);
    memcpy(s->sh_dau_flag_pipe, s->dau_flag_pipe, sizeof s->dau_flag_pipe);
    s->sh_do_active = s->do_active; s->sh_do_lock = s->do_lock;
    s->sh_do_start = s->do_start;   s->sh_do_end = s->do_end;
    s->sh_do_count = s->do_count;
    s->do_active = 0;
    /* the not-yet-executed instruction at the resume address goes into
     * the instruction shadow register — as prefetched, i.e. under the
     * memory map in force while the PREVIOUS instruction executed (the
     * boot ROM's SAR flow prefetches `*r0 = r0` in computer mode and
     * replays it in processor mode) — and pcsh points at the location
     * to be fetched when exiting [IM §7.5.1, IRETURN page] */
    if (s->prefetch_addr == s->pc) {
        s->irsh = s->prefetch;
    } else {
        s->irsh = 0x80000000u;               /* nop */
        raw_access(s, s->pc, 4, &s->irsh, 0);
    }
    s->irsh_addr = s->pc;
    s->pcsh = s->pc + 4;
    s->level = DSP3210_LVL_INTERRUPT;
    s->pc  = s->r[22] + (uint32_t)vn * 8;    /* goto evtp + vn*8 ; nop */
    s->npc = s->pc + 4;
}

static void do_ireturn(dsp3210_emu *s)
{
    if (s->level == DSP3210_LVL_INTERRUPT) {
        s->ps = s->sh_ps;
        s->dauc = s->sh_dauc;
        s->ctr = s->sh_ctr;
        memcpy(s->a, s->sh_a, sizeof s->a);
        memcpy(s->a_pipe, s->sh_a_pipe, sizeof s->a_pipe);
        memcpy(s->dau_flag_pipe, s->sh_dau_flag_pipe,
               sizeof s->dau_flag_pipe);
        s->do_active = s->sh_do_active; s->do_lock = s->sh_do_lock;
        s->do_start = s->sh_do_start;   s->do_end = s->sh_do_end;
        s->do_count = s->sh_do_count;
        s->level = DSP3210_LVL_BASE;
    }
    /* No latent instruction: the word after ireturn is never executed.
     * Instead the instruction prefetched before the interrupt executes
     * from the instruction shadow register, and fetching resumes at
     * pcsh [IM IRETURN page, Figure 7-6].  The boot ROM's SAR path
     * depends on the replay: its EXT1 handler overwrites pcsh with the
     * host vector and lets the replayed `*r0 = r0` post the boot-done
     * signal. */
    s->pc  = s->pcsh;
    s->npc = s->pcsh + 4;
    s->irsh_pending = 1;
}

/* ------------------------------------------------------------------ */
/* memory [DOC §1.1]                                                   */

static uint8_t *mem_ptr(dsp3210_emu *s, uint32_t addr)
{
    /* The on-chip 64 KB window (boot ROM, MMIO, RAM1, RAM0) is the same
     * physical array decoded at $0000xxxx in computer mode (pcw[10]=1)
     * and at $5003xxxx in processor mode [IM Figure 3-4] — the boot
     * ROM's SAR path runs across the switch.  External memory A backs
     * everything else (it starts at $10000 in computer mode). */
    uint32_t base = (s->pcw & (1u << 10)) ? 0u : 0x50030000u;
    if (addr - base < 0x10000u)
        return &s->chip[addr - base];
    if (addr < s->mem_size)
        return &s->mem[addr];
    return NULL;
}

/* raw access, no exceptions — for loaders and hosts */
static int raw_access(dsp3210_emu *s, uint32_t addr, int size,
                      uint32_t *inout, int write)
{
    int big = (s->pcw >> 8) & 1, i;
    uint32_t v = write ? *inout : 0;
    if (addr & (uint32_t)(size - 1))
        return -1;
    for (i = 0; i < size; i++) {
        int shift = 8 * (big ? size - 1 - i : i);
        uint8_t *p = mem_ptr(s, addr + (uint32_t)i);
        if (!p)
            return -1;
        if (write)
            *p = (uint8_t)(v >> shift);
        else
            v |= (uint32_t)*p << shift;
    }
    if (!write)
        *inout = v;
    return 0;
}

/* access from executing code — raises address/bus errors.
 *
 * A misaligned access raises the address error (vector 4).  When that
 * error is MASKED, the access still completes with the low address bits
 * ignored (aligned down): shipped DSP kernel code relies on this — it
 * programs the byte-wide tcon MMIO register at $50030413 with a long
 * store, which only works if the masked misaligned store lands on the
 * $0410 word slot with tcon in its low byte lane. */
/* QEMU: whether an address goes to the I/O hooks */
static int is_io(dsp3210_emu *s, uint32_t addr)
{
    uint32_t base = (s->pcw & (1u << 10)) ? 0u : 0x50030000u;
    if (!s->io_read_fn)
        return 0;
    if (addr - base < 0x10000u)
        return addr - base >= 0x400u && addr - base < 0x800u;
    return addr >= s->mem_size;
}

static int mem_read(dsp3210_emu *s, uint32_t addr, int size, uint32_t *out)
{
    int fault = 0;
    if (addr & (uint32_t)(size - 1)) {       /* [IM §7.5, address error] */
        if (take_error(s, DSP3210_VEC_AERR))
            return -1;
        addr &= ~(uint32_t)(size - 1);
    }
    if (is_io(s, addr)) {
        *out = s->io_read_fn(s->hook_ctx, addr, size, &fault);
        if (fault) { take_error(s, DSP3210_VEC_BUSERR); return -1; }
        return 0;
    }
    if (s->read_fn) {
        *out = s->read_fn(s->hook_ctx, addr, size, &fault);
        if (fault) { take_error(s, DSP3210_VEC_BUSERR); return -1; }
        if (!s->in_fetch)
            da_shadow_overlay(s, addr, size, out);
        return 0;
    }
    if (raw_access(s, addr, size, out, 0)) {
        take_error(s, DSP3210_VEC_BUSERR);
        return -1;
    }
    if (!s->in_fetch)
        da_shadow_overlay(s, addr, size, out);
    return 0;
}

static int mem_write(dsp3210_emu *s, uint32_t addr, int size, uint32_t val)
{
    int fault = 0;
    if (addr & (uint32_t)(size - 1)) {
        if (take_error(s, DSP3210_VEC_AERR))
            return -1;
        addr &= ~(uint32_t)(size - 1);       /* see mem_read */
    }
    if (is_io(s, addr)) {
        s->io_write_fn(s->hook_ctx, addr, val, size, &fault);
        if (fault) { take_error(s, DSP3210_VEC_BUSERR); return -1; }
        return 0;
    }
    if (s->write_fn) {
        s->write_fn(s->hook_ctx, addr, val, size, &fault);
        if (fault) { take_error(s, DSP3210_VEC_BUSERR); return -1; }
        return 0;
    }
    if (raw_access(s, addr, size, &val, 1)) {
        take_error(s, DSP3210_VEC_BUSERR);
        return -1;
    }
    return 0;
}


/* ------------------------------------------------------------------ */
/* DA store shadow (Latency 1) [IM §4.4.2.1]                           */

/* A DA memory write lands at the pipeline's fourth stage, so the value
 * written is not readable *by the DSP itself* until four instructions
 * later.  The write is committed to memory immediately (hosts, DMA and
 * faults behave as before); a 4-entry shadow of the pre-write bytes
 * overlays the DSP's own data reads while the window lasts.
 * Instruction fetch is exempt, and so is hook-backed memory (see
 * below) — an embedder that replaces the map models the latency, if it
 * wants it, on its own side. */
static int da_shadowed_write(dsp3210_emu *s, uint32_t addr, int size,
                             uint32_t val)
{
    uint8_t old[4] = { 0, 0, 0, 0 };
    uint32_t maddr = addr & ~(uint32_t)(size - 1);
    int have_old, i, slot = 0;

    /*
     * Snapshot the pre-write bytes from the *aligned* address, which is
     * where mem_write will land them, but hand mem_write the address as
     * given: it owns the alignment rules, and pre-masking here would
     * suppress the address error (vector 4) a misaligned DA store must
     * raise.
     *
     * The snapshot reads the built-in map directly, so it cannot see
     * hook-backed memory — and reading through the hook is not an
     * option, since an MMIO read may have side effects.  When hooks are
     * installed, skip the shadow entirely rather than overlay the
     * DSP's reads with bytes from an array the embedder isn't using.
     */
    have_old = (s->read_fn == NULL && s->write_fn == NULL);
    for (i = 0; have_old && i < size; i++) {
        uint32_t b = 0;
        if (raw_access(s, maddr + (uint32_t)i, 1, &b, 0))
            have_old = 0;
        old[i] = (uint8_t)b;
    }
    if (mem_write(s, addr, size, val))
        return -1;
    if (!have_old)
        return 0;                      /* hook-backed or unmapped */
    for (i = 0; i < 4; i++) {          /* free slot, else the oldest */
        if (!s->da_wr[i].live) { slot = i; break; }
        if (s->da_wr[i].rem < s->da_wr[slot].rem)
            slot = i;
    }
    s->da_wr[slot].addr = maddr;
    s->da_wr[slot].size = (uint8_t)size;
    s->da_wr[slot].rem = 3;
    s->da_wr[slot].live = 1;
    memcpy(s->da_wr[slot].old, old, sizeof old);
    return 0;
}

/* Overlay any byte still inside a store's three-instruction window with
 * its pre-write value; when windows overlap, the OLDEST wins (the
 * result is the memory content from before the whole window). */
static void da_shadow_overlay(dsp3210_emu *s, uint32_t addr, int size,
                              uint32_t *val)
{
    int big = (s->pcw >> 8) & 1;
    int rem, e, i;
    for (rem = 3; rem >= 0; rem--)             /* newest first, so the */
        for (e = 0; e < 4; e++) {              /* oldest lands last    */
            const struct dsp3210_da_wr *w = &s->da_wr[e];
            if (!w->live || w->rem != rem)
                continue;
            for (i = 0; i < size; i++) {
                uint32_t a = addr + (uint32_t)i;
                if (a - w->addr < w->size) {
                    int shift = 8 * (big ? size - 1 - i : i);
                    *val = (*val & ~(0xFFu << shift))
                         | ((uint32_t)w->old[a - w->addr] << shift);
                }
            }
        }
}

/* ------------------------------------------------------------------ */
/* pipeline/pin time base                                              */

/* one unit of core time: each executed instruction (both the normal
 * fetch path and the irsh replay) advances the DAU latency pipes and
 * ages the DA store shadow */
static void latency_tick(dsp3210_emu *s)
{
    int i;
    memmove(s->a_pipe[1], s->a_pipe[0], 2 * sizeof s->a_pipe[0]);
    memcpy(s->a_pipe[0], s->a, sizeof s->a);
    memmove(&s->dau_flag_pipe[1], &s->dau_flag_pipe[0], 3);
    s->dau_flag_pipe[0] = (uint8_t)((s->ps >> 4) & 0xFu);
    for (i = 0; i < 4; i++)
        if (s->da_wr[i].live) {
            if (s->da_wr[i].rem == 0)
                s->da_wr[i].live = 0;
            else
                s->da_wr[i].rem--;
        }
}

/* derive the live ps.IR pin bits from the pulse counters (active-low
 * pins: 1 = negated/idle) and burn one slot of pulse time */
static void pin_tick(dsp3210_emu *s)
{
    int i;
    s->ps = (uint16_t)((s->ps & ~(DSP3210_PS_IR0 | DSP3210_PS_IR1))
          | (s->ext_pulse[0] ? 0 : DSP3210_PS_IR0)
          | (s->ext_pulse[1] ? 0 : DSP3210_PS_IR1));
    for (i = 0; i < 2; i++)
        if (s->ext_pulse[i])
            s->ext_pulse[i]--;
}

/* ------------------------------------------------------------------ */
/* The DAU — one of two implementations (see the file header)          */

#ifdef DSP3210_DAU_EXACT
#include "dsp3210_dau_exact.inc"
#else
#include "dsp3210_dau_double.inc"
#endif

/* ------------------------------------------------------------------ */
/* CAU flags (Table 4-1)                                               */

static void set_flags(dsp3210_emu *s, unsigned n, unsigned z, unsigned v,
                      unsigned c)
{
    s->ps = (uint16_t)((s->ps & ~0xFu)
          | (n ? DSP3210_PS_n : 0) | (z ? DSP3210_PS_z : 0)
          | (v ? DSP3210_PS_v : 0) | (c ? DSP3210_PS_c : 0));
}

/* add with carry-in; flags per 16/32-bit rules; result sign-extended
 * when short [ADD page] */
static uint32_t alu_add(dsp3210_emu *s, uint32_t a, uint32_t b, unsigned cin,
                        int w16, int setf)
{
    if (w16) {
        uint32_t ua = a & 0xFFFFu, ub = b & 0xFFFFu;
        uint32_t sum = ua + ub + cin;
        uint32_t res = (uint32_t)sext16(sum);
        if (setf)
            set_flags(s, (res >> 31) & 1, (sum & 0xFFFFu) == 0,
                      ((~(ua ^ ub) & (ua ^ sum)) >> 15) & 1,
                      (sum >> 16) & 1);
        return res;
    } else {
        uint64_t sum = (uint64_t)a + b + cin;
        uint32_t res = (uint32_t)sum;
        if (setf)
            set_flags(s, res >> 31, res == 0,
                      (uint32_t)((~(a ^ b) & (a ^ res)) >> 31) & 1,
                      (unsigned)(sum >> 32) & 1);
        return res;
    }
}

/* a - b; c is the borrow (c=1 ⇒ borrow occurred) — matches the negate
 * special case documented on the SUBTRACT page */
static uint32_t alu_sub(dsp3210_emu *s, uint32_t a, uint32_t b,
                        int w16, int setf)
{
    if (w16) {
        uint32_t ua = a & 0xFFFFu, ub = b & 0xFFFFu;
        uint32_t diff = ua - ub;
        uint32_t res = (uint32_t)sext16(diff);
        if (setf)
            set_flags(s, (res >> 31) & 1, (diff & 0xFFFFu) == 0,
                      (((ua ^ ub) & (ua ^ diff)) >> 15) & 1,
                      ua < ub);
        return res;
    } else {
        uint32_t res = a - b;
        if (setf)
            set_flags(s, res >> 31, res == 0,
                      (((a ^ b) & (a ^ res)) >> 31) & 1,
                      a < b);
        return res;
    }
}

/* logic-class flags: nz, v=c=0 */
static void flags_nz(dsp3210_emu *s, uint32_t res, int w16)
{
    if (w16)
        set_flags(s, (res >> 15) & 1, (res & 0xFFFFu) == 0, 0, 0);
    else
        set_flags(s, res >> 31, res == 0, 0, 0);
}

static uint32_t bitrev(uint32_t x, int nbits)
{
    uint32_t r = 0;
    int i;
    for (i = 0; i < nbits; i++)
        r |= ((x >> i) & 1u) << (nbits - 1 - i);
    return r;
}

/* condition evaluation (Table 4-7 / DOC §1.5.2) */
static int cond_eval(dsp3210_emu *s, unsigned c)
{
    unsigned ps = s->ps;
    unsigned n = !!(ps & DSP3210_PS_n), z = !!(ps & DSP3210_PS_z);
    unsigned v = !!(ps & DSP3210_PS_v), cf = !!(ps & DSP3210_PS_c);
    /* DAU conditions test the flags established four instructions back
     * (Latency 4) [IM §4.4.2.4]; ifalt/ifaeq/ifagt read the live flags
     * and do not come through here */
    unsigned dnzuv = s->dau_flag_pipe[3];
    unsigned AN = (dnzuv >> 0) & 1, AZ = (dnzuv >> 1) & 1;
    unsigned AU = (dnzuv >> 2) & 1, AV = (dnzuv >> 3) & 1;

    switch (c & 63) {
    case 0:  return 0;                 /* false */
    case 1:  return 1;                 /* true */
    case 2:  return !n;                /* pl */
    case 3:  return n;                 /* mi */
    case 4:  return !z;                /* ne */
    case 5:  return z;                 /* eq */
    case 6:  return !v;                /* vc */
    case 7:  return v;                 /* vs */
    case 8:  return !cf;               /* cc */
    case 9:  return cf;                /* cs */
    case 10: return !(n ^ v);          /* ge */
    case 11: return n ^ v;             /* lt */
    case 12: return !(z | (n ^ v));    /* gt */
    case 13: return z | (n ^ v);       /* le */
    case 14: return !(cf | z);         /* hi (unsigned >) */
    case 15: return cf | z;            /* ls (unsigned <=) */
    case 16: return !AU;               /* auc */
    case 17: return AU;                /* aus */
    case 18: return !AN;               /* age */
    case 19: return AN;                /* alt */
    case 20: return !AZ;               /* ane */
    case 21: return AZ;                /* aeq */
    case 22: return !AV;               /* avc */
    case 23: return AV;                /* avs */
    case 24: return !(AN | AZ);        /* agt */
    case 25: return AN | AZ;           /* ale */
    case 32: return !(ps & DSP3210_PS_IBF);  /* ibe */
    case 33: return !!(ps & DSP3210_PS_IBF); /* ibf */
    case 34: return !(ps & DSP3210_PS_OBE);  /* obf */
    case 35: return !!(ps & DSP3210_PS_OBE); /* obe */
    case 40: return !(ps & DSP3210_PS_SY);   /* syc */
    case 41: return !!(ps & DSP3210_PS_SY);  /* sys */
    case 42: return !(ps & DSP3210_PS_FB);   /* fbc */
    case 43: return !!(ps & DSP3210_PS_FB);  /* fbs */
    case 44: return !(ps & DSP3210_PS_IR0);  /* ir0c */
    case 45: return !!(ps & DSP3210_PS_IR0); /* ir0s */
    case 46: return !(ps & DSP3210_PS_IR1);  /* ir1c */
    case 47: return !!(ps & DSP3210_PS_IR1); /* ir1s */
    default: return 0;                 /* reserved: undefined, take false */
    }
}

/* ------------------------------------------------------------------ */
/* IO registers (format 7b/7d) [DOC §1.4]                              */

/* spc write side effects, distinguished only by the W size */
enum { SPC_NONE = 0, SPC_SFTRST, SPC_BKPT, SPC_WAITI };

static uint32_t ior_read(dsp3210_emu *s, unsigned n)
{
    switch (n & 31) {
    case 0:  return s->ps & 0x3FFFu;   /* bits 15-14 read 0 */
    case 8:  return s->emr;
    case 10: return 0;                 /* spc is write-only */
    case 12: return s->pcw;
    case 14: return s->dauc;
    case 15: return s->ctr & 0x3Fu;
    default: return 0;                 /* reserved */
    }
}

static int ior_write(dsp3210_emu *s, unsigned n, uint32_t v, int size)
{
    switch (n & 31) {
    case 0:                            /* only CAU flags are writable */
        s->ps = (uint16_t)((s->ps & ~0xFu) | (v & 0xFu));
        break;
    case 8:
        s->emr = (uint16_t)v;
        if (v & 1u)
            /* hardware-verified: an emr write with bit 0 set drops a
             * latched-but-untaken EXT1 request without taking it (the
             * kernel's r=emr; emr=r|1; emr=r pulse); the pin level is
             * unaffected */
            s->pending &= (uint16_t)~(1u << DSP3210_VEC_EXT1);
        break;
    case 10:                           /* spc pseudo-register */
        return size == 1 ? SPC_SFTRST : size == 2 ? SPC_BKPT : SPC_WAITI;
    case 12:                           /* pcw; bit 13 locks it */
        if (!s->pcw_locked) {
            s->pcw = (uint16_t)v;
            if (v & (1u << 13))
                s->pcw_locked = 1;
        }
        break;
    case 14:
        s->dauc = (uint8_t)(v & 0x7Fu);/* bit 7 must be 0 */
        break;
    case 15:                           /* ctr is read-only in practice */
    default:
        break;
    }
    return SPC_NONE;
}

/* ------------------------------------------------------------------ */
/* CA moves: sizes and extensions (LOAD/STORE pages)                   */

static int w_size(unsigned wsz)
{
    switch (wsz & 7) {
    case 0: case 1: case 4: return 1;  /* byte / char / hbyte */
    case 2: case 3:         return 2;  /* ushort / short */
    default:                return 4;  /* long (and reserved 101/110) */
    }
}

static uint32_t w_extend(unsigned wsz, uint32_t raw)
{
    switch (wsz & 7) {
    case 0: return raw & 0xFFu;                        /* (byte) */
    case 1: return (uint32_t)(int32_t)(int8_t)raw;     /* (char) */
    case 2: return raw & 0xFFFFu;                      /* (ushort) */
    case 3: return (uint32_t)sext16(raw);              /* (short) */
    case 4: return (raw & 0xFFu) << 8;                 /* (hbyte) */
    default: return raw;                               /* (long) */
    }
}

static uint32_t w_select(unsigned wsz, uint32_t reg)
{
    switch (wsz & 7) {
    case 0: case 1: return reg & 0xFFu;        /* (byte)/(char): bits 7-0 */
    case 2: case 3: return reg & 0xFFFFu;      /* 16-bit: bits 15-0 */
    case 4: return (reg >> 8) & 0xFFu;         /* (hbyte): bits 15-8 */
    default: return reg;
    }
}

/* post-modification of rP by the rI code: r0 = +0, +n/-n = ±size,
 * anything else adds the register value (unscaled) */
static void ca_postmod(dsp3210_emu *s, unsigned rp, unsigned ri, int size)
{
    uint32_t delta;
    switch (ri & 31) {
    case 0:  return;
    case 23: delta = (uint32_t)size; break;
    case 22: delta = (uint32_t)-size; break;
    default: delta = opval(s, ri); break;
    }
    regw(s, rp, opval(s, rp) + delta);
}

/* ------------------------------------------------------------------ */
/* CA ALU (formats 6a-6d) [IM Table 10-2 "CA - F Field"]               */

enum {
    F_ADD = 0, F_SHL = 1, F_RSUB = 2, F_CRADD = 3, F_SUB = 4,
    F_RES5 = 5, F_ANDC = 6, F_CMP = 7, F_XOR = 8, F_ROR = 9,
    F_OR = 10, F_ROL = 11, F_SHR = 12, F_ASR = 13, F_AND = 14,
    F_BTST = 15
};

/* shared op body: computes result + flags; *store = 0 for cmp/btst */
static uint32_t alu_op(dsp3210_emu *s, unsigned f, uint32_t a, uint32_t b,
                       int w16, int *store)
{
    uint32_t res = 0;
    unsigned msb = w16 ? 15 : 31;
    unsigned oldc = !!(s->ps & DSP3210_PS_c);
    *store = 1;

    switch (f) {
    case F_ADD:
        res = alu_add(s, a, b, 0, w16, 1);
        break;
    case F_SUB:
        res = alu_sub(s, a, b, w16, 1);
        break;
    case F_RSUB:                       /* N - rD / rS2 - rS1 */
        res = alu_sub(s, b, a, w16, 1);
        break;
    case F_CRADD: {                    /* carry propagates MSB → LSB */
        int nb = w16 ? 16 : 32;
        uint64_t sum = (uint64_t)bitrev(a & (w16 ? 0xFFFFu : ~0u), nb)
                     + bitrev(b & (w16 ? 0xFFFFu : ~0u), nb);
        res = bitrev((uint32_t)sum & (w16 ? 0xFFFFu : ~0u), nb);
        if (w16)
            res = (uint32_t)sext16(res);
        set_flags(s, (res >> 31) & 1, (w16 ? (res & 0xFFFFu) : res) == 0,
                  0, (unsigned)(sum >> nb) & 1);
        break;
    }
    case F_ANDC: res = a & ~b; goto logic;
    case F_XOR:  res = a ^ b;  goto logic;
    case F_OR:   res = a | b;  goto logic;
    case F_AND:  res = a & b;  goto logic;
    case F_BTST:
        flags_nz(s, a & b, w16);
        *store = 0;
        break;
    case F_CMP:
        alu_sub(s, a, b, w16, 1);
        *store = 0;
        break;
    case F_SHL:                        /* logical; short zero-extends */
        res = (w16 ? (a << (b & 31)) & 0xFFFFu : a << (b & 31));
        flags_nz(s, res, w16);
        break;
    case F_SHR:
        res = (w16 ? (a & 0xFFFFu) : a) >> (b & 31);
        flags_nz(s, res, w16);
        break;
    case F_ASR: {
        int32_t sa = w16 ? (int32_t)sext16(a) : (int32_t)a;
        res = (uint32_t)(sa >> (b & 31));
        if (w16)
            res = (uint32_t)sext16(res);
        flags_nz(s, res, w16);
        break;
    }
    case F_ROL:
        /* rotate left through carry ≡ a+a+carry-in (the ADD page calls
         * rS*2 "a left shift by 1 with carry"); flags via add rules */
        res = alu_add(s, a, a, oldc, w16, 1);
        break;
    case F_ROR: {                      /* nzc, v = 0 */
        unsigned newc = a & 1u;
        uint32_t ua = w16 ? (a & 0xFFFFu) : a;
        res = (ua >> 1) | (oldc << msb);
        if (w16)
            res = (uint32_t)sext16(res);
        set_flags(s, (res >> 31) & 1, (w16 ? (res & 0xFFFFu) : res) == 0,
                  0, newc);
        break;
    }
    default:                           /* F_RES5: reserved function */
        *store = 0;
        break;
    }
    return res;

logic:
    if (w16)
        res = (uint32_t)sext16(res);   /* AND-COMPLEMENT page: short
                                          results sign-extend */
    flags_nz(s, res, w16);
    return res;
}

static void exec_alu_reg(dsp3210_emu *s, uint32_t w)
{
    int  w16 = !bits(w, 31, 31);
    unsigned f   = bits(w, 24, 21);
    unsigned rd  = bits(w, 20, 16);
    unsigned rs1 = bits(w, 15, 11);
    unsigned c   = bits(w, 10, 5);
    unsigned rs2 = bits(w, 4, 0);
    uint32_t a, b, res;
    int store;

    if (!cond_eval(s, c))
        return;                        /* COND false: no store, no flags */

    a = opval(s, rs1);
    /* sp = sp++ / sp = sp-- move by 4, not 1 (INCR/DECR page) */
    if (f == F_ADD && rd == 25 && rs1 == 25 && (rs2 == 22 || rs2 == 23))
        b = rs2 == 23 ? 4u : (uint32_t)-4;
    else
        b = opval(s, rs2);             /* +n/-n read as ±1 */

    res = alu_op(s, f, a, b, w16, &store);
    if (store)
        regw(s, rd, res);
}

static void exec_alu_imm(dsp3210_emu *s, uint32_t w)
{
    int  w16 = !bits(w, 31, 31);
    unsigned f  = bits(w, 24, 21);
    unsigned rd = bits(w, 20, 16);
    uint32_t n  = (uint32_t)sext16(w);
    uint32_t a = opval(s, rd), res;
    int store;

    /* rotates have no immediate operand — they rotate rD by 1 */
    if (f == F_ROL || f == F_ROR)
        n = a;
    res = alu_op(s, f, a, n, w16, &store);
    if (store)
        regw(s, rd, res);
}

/* ------------------------------------------------------------------ */
/* CA moves (formats 7a-7d)                                            */

static int exec_move(dsp3210_emu *s, uint32_t w)
{
    int direct = !bits(w, 31, 31);     /* 7a */
    unsigned io  = bits(w, 25, 25);    /* 7d */
    unsigned t   = bits(w, 24, 24);    /* 0 = load register, 1 = store */
    unsigned wsz = bits(w, 23, 21);
    unsigned rh  = bits(w, 20, 16);
    int size = w_size(wsz);
    uint32_t raw;

    /* NOTE: CA register loads SET the CAU flags from the loaded value.
     * The LOAD page's header claims "CAU FLAGS AFFECTED: None", but its
     * own restriction reads "the CAU register loaded and THE FLAGS SET
     * AS A RESULT OF THE LOAD cannot be referenced in the following
     * instruction" [IM §4.4.1.3 Restriction 3] — and Apple's shipped DSP
     * code branches on `rD = *rD ; nop ; if (gt)`, which only works
     * if loads set flags.  n/z from the extended value; v = c = 0
     * (exact v/c behaviour undocumented). */

    if (direct) {
        /* 7a: rH = (w) *L / *L = (w) rH.  L is a 16-bit on-chip window
         * address: upper half $0000 (computer) or $5003 (processor)
         * [IM §3.5.6]. */
        uint32_t addr = (s->pcw & (1u << 10)) ? (w & 0xFFFFu)
                                              : 0x50030000u | (w & 0xFFFFu);
        if (bits(w, 25, 25))
            return DSP3210_STEP_OK;    /* reserved encoding */
        if (t == 0) {
            if (mem_read(s, addr, size, &raw))
                return DSP3210_STEP_OK;
            regw(s, rh, w_extend(wsz, raw));
            flags_nz(s, w_extend(wsz, raw), 0);
        } else {
            mem_write(s, addr, size, w_select(wsz, opval(s, rh)));
        }
        return DSP3210_STEP_OK;
    }

    if (!io && bits(w, 10, 10)) {
        /* 7b: rH = (w) iorN / iorN = (w) rH — and the three spc
         * pseudo-instructions (waiti/bkpt/sftrst), which differ only in
         * the W field [DOC §1.5.4] */
        unsigned ior = w & 31;
        if (t == 0) {
            regw(s, rh, w_extend(wsz, ior_read(s, ior)));
            flags_nz(s, w_extend(wsz, ior_read(s, ior)), 0);
        } else {
            int spc = ior_write(s, ior, w_select(wsz, opval(s, rh)), size);
            if (spc == SPC_SFTRST) {   /* drop to base level [SFTRST] */
                s->level = DSP3210_LVL_BASE;
                s->error_nonmaskable = 0;
            } else if (spc == SPC_BKPT) {
                return DSP3210_STEP_BKPT;
            } else if (spc == SPC_WAITI) {
                /* ignored if an interrupt is already pending [WAITI] */
                if (!pending_vector(s))
                    s->waiting = 1;
            }
        }
        return DSP3210_STEP_OK;
    }

    /* 7c: rH <-> MEM  |  7d: iorH <-> MEM */
    {
        unsigned rp = bits(w, 15, 11);
        unsigned ri = w & 31;
        uint32_t addr = opval(s, rp);

        if (t == 0) {
            if (mem_read(s, addr, size, &raw))
                return DSP3210_STEP_OK;
            ca_postmod(s, rp, ri, size);
            if (io) {
                int spc = ior_write(s, rh, raw, size);   /* iorD = MEM:
                        (byte)/(short) affect only the low bits (LOAD-IOR),
                        approximated here as a plain sized write */
                (void)spc;             /* spc as 7d destination: undefined;
                                          side effect not honoured */
            } else {
                regw(s, rh, w_extend(wsz, raw));
                flags_nz(s, w_extend(wsz, raw), 0);
            }
        } else {
            uint32_t v = io ? w_select(wsz, ior_read(s, rh))
                            : w_select(wsz, opval(s, rh));
            if (mem_write(s, addr, size, v))
                return DSP3210_STEP_OK;
            ca_postmod(s, rp, ri, size);
        }
    }
    return DSP3210_STEP_OK;
}

/* ------------------------------------------------------------------ */
/* main execute                                                        */

static int opcode_is_illegal(unsigned op6)
{
    switch (op6) {                     /* IM §7.5.3.2, the seven patterns */
    case 0x00: case 0x01: case 0x02:
    case 0x0F:
    case 0x16: case 0x17:
    case 0x22:
        return 1;
    default:
        return 0;
    }
}

static int exec_insn(dsp3210_emu *s, uint32_t w)
{
    unsigned op6 = w >> 26;
    uint32_t addr = s->cur_insn;

    if (opcode_is_illegal(op6)) {
        take_error(s, DSP3210_VEC_ILLOP);
        return DSP3210_STEP_OK;
    }
    if ((w >> 29) >= 1 && (w >> 29) <= 3)
        return exec_da(s, w);

    switch (op6) {
    case 0x03: {                       /* 3a: if (rM-- >= 0) goto — test
                                          the old value, always decrement
                                          [GOTO-LOOP] */
        unsigned rm = bits(w, 25, 21), rb = bits(w, 20, 16);
        uint32_t val = opval(s, rm);
        if ((int32_t)val >= 0)
            s->npc = opval(s, rb) + (uint32_t)sext16(w);
        regw(s, rm, val - 1);
        break;
    }
    case 0x04: {                       /* 4a: call {rB,rB+N} (rM); rM gets
                                          the address after the latent
                                          instruction = insn + 8 [CALL] */
        unsigned rm = bits(w, 25, 21), rb = bits(w, 20, 16);
        uint32_t target = opval(s, rb) + (uint32_t)sext16(w);
        regw(s, rm, addr + 8);
        s->npc = target;
        break;
    }
    case 0x05: case 0x25: {            /* 5a/5b: rD = (size) rS3 + N,
                                          flags nzvc [ADD] */
        unsigned rd = bits(w, 25, 21), rs3 = bits(w, 20, 16);
        int w16 = !(op6 & 0x20);
        regw(s, rd, alu_add(s, opval(s, rs3), (uint32_t)sext16(w), 0,
                            w16, 1));
        break;
    }
    case 0x06: case 0x26:              /* 6a-6d: ALU */
        if (bits(w, 25, 25))
            exec_alu_imm(s, w);
        else
            exec_alu_reg(s, w);
        break;
    case 0x07: case 0x27:              /* 7a-7d: moves */
        return exec_move(s, w);
    case 0x20: case 0x21: {            /* 0b/1b: if (COND) goto / nop /
                                          ireturn */
        unsigned c = bits(w, 26, 21), rb = bits(w, 20, 16);
        if (c == 1 && rb == 30 && (w & 0xFFFFu) == 0) {
            do_ireturn(s);
            break;
        }
        if (cond_eval(s, c))
            s->npc = opval(s, rb) + (uint32_t)sext16(w);
        break;
    }
    case 0x23: {                       /* 3b/3c: do / dolock / doblock */
        unsigned isreg = bits(w, 25, 25);
        unsigned b = bits(w, 24, 24), m = bits(w, 23, 23);
        unsigned k = bits(w, 17, 11);
        uint32_t count = isreg ? (opval(s, w & 31) & 0x7FFu) + 1
                               : bits(w, 10, 0) + 1;
        if (b && m)
            break;                     /* reserved combination */
        if (m)
            k = 0;                     /* doblock: single instruction */
        s->do_active = 1;
        s->do_lock = (b != 0);         /* dolock masks interrupts */
        s->do_start = s->pc;           /* next instruction */
        s->do_end = s->pc + 4u * k;    /* K+1 instructions */
        s->do_count = count;           /* L+1 / (rM&0x7FF)+1 iterations */
        break;
    }
    case 0x24: {                       /* 4b: rD = rS <<| N — N<<16 | rS,
                                          always long, flags nz [SHIFT-OR] */
        unsigned rd = bits(w, 25, 21), rs = bits(w, 20, 16);
        uint32_t res = opval(s, rs) | ((w & 0xFFFFu) << 16);
        flags_nz(s, res, 0);
        regw(s, rd, res);
        break;
    }
    case 0x28: case 0x29: case 0x2A: case 0x2B:
    case 0x2C: case 0x2D: case 0x2E: case 0x2F: {  /* 8a: goto {M, rB+M} */
        unsigned rb = bits(w, 20, 16);
        uint32_t mm = (bits(w, 28, 21) << 16) | (w & 0xFFFFu);
        s->npc = opval(s, rb) + mm;
        break;
    }
    case 0x30: case 0x31: case 0x32: case 0x33:
    case 0x34: case 0x35: case 0x36: case 0x37: {  /* 8b: rD=(ushort24)M,
                                          no flags [SET24] */
        unsigned rd = bits(w, 20, 16);
        regw(s, rd, (bits(w, 28, 21) << 16) | (w & 0xFFFFu));
        break;
    }
    case 0x38: case 0x39: case 0x3A: case 0x3B:
    case 0x3C: case 0x3D: case 0x3E: case 0x3F: {  /* 8c: call M (rM) */
        unsigned rm = bits(w, 20, 16);
        regw(s, rm, addr + 8);
        s->npc = (bits(w, 28, 21) << 16) | (w & 0xFFFFu);
        break;
    }
    default:                           /* unreachable */
        break;
    }
    return DSP3210_STEP_OK;
}

/* ------------------------------------------------------------------ */
/* step                                                                */

int dsp3210_step(dsp3210_emu *s)
{
    uint32_t w, addr;
    int st;

    if (s->level == DSP3210_LVL_DERROR)
        return DSP3210_STEP_DERROR;

    /* one slot of core time passes per step, executed or asleep */
    pin_tick(s);

    if (s->waiting) {
        if (pending_vector(s)) {
            /* the instruction after waiti executes when the interrupt
             * is recognised, before the interrupt is taken [WAITI] */
            s->waiting = 0;
            s->int_defer = 1;
        } else {
            /* time passes while asleep: the DAU pipeline keeps
             * clocking, so stale accumulator/flag values settle and
             * the DA store shadow drains during waiti */
            latency_tick(s);
            return DSP3210_STEP_WAITI;
        }
    }

    /* interrupt recognition: base level only, never in the shadow of a
     * delayed branch (branches are not interruptible [IM §7.5.1]), never
     * inside a dolock loop, and not before a pending irsh replay */
    if (!s->int_defer && !s->irsh_pending && s->level == DSP3210_LVL_BASE
        && s->npc == s->pc + 4
        && !(s->do_active && s->do_lock)) {
        int vn = pending_vector(s);
        if (vn)
            take_interrupt(s, vn);
    }
    s->int_defer = 0;

    if (s->irsh_pending) {
        /* replay the shadowed instruction; pc/npc already aim at pcsh */
        s->irsh_pending = 0;
        addr = s->irsh_addr;
        w = s->irsh;
        s->cur_insn = addr;
        s->prefetch_addr = 1;          /* odd: never matches a pc */
        s->icount++;
        latency_tick(s);
        st = exec_insn(s, w);
    } else {
        addr = s->pc;
        s->cur_insn = addr;
        s->in_fetch = 1;               /* fetch bypasses the DA store
                                          shadow [IM §8.2.6] */
        if (mem_read(s, addr, 4, &w)) {
            s->in_fetch = 0;
            return DSP3210_STEP_OK;    /* fetch fault → exception taken */
        }
        s->in_fetch = 0;

        s->pc = s->npc;
        s->npc = s->pc + 4;
        s->icount++;
        latency_tick(s);

        /* model the prefetch of the next instruction: it happens under
         * the memory map in force NOW, before this instruction's side
         * effects (a pcw write) can change it — this is what lands in
         * irsh if an interrupt hits at the next boundary */
        s->prefetch_addr = s->pc;
        if (raw_access(s, s->pc, 4, &s->prefetch, 0))
            s->prefetch_addr = 1;

        st = exec_insn(s, w);
    }

    /* do-loop back-edge [DO page]: after the last instruction of the
     * body, loop while iterations remain.  (An error exception inside
     * the body clears do_active, so this is skipped on abort.) */
    if (s->do_active && addr == s->do_end) {
        if (--s->do_count == 0) {
            s->do_active = 0;
        } else {
            s->pc = s->do_start;
            s->npc = s->do_start + 4;
        }
    }
    return st;
}

/* ------------------------------------------------------------------ */
/* host interface                                                      */

void dsp3210_reset(dsp3210_emu *s, unsigned straps)
{
    /* Reset state per [IM Table 7-4]; r1-r19, r21, a0-a3, ps, ctr are
     * architecturally undefined — cleared here for determinism. */
    uint32_t prev_pc = s->pc;
    memset(s->r, 0, sizeof s->r);
    memset(s->a, 0, sizeof s->a);   /* e = 0 => all four read as zero */
    s->r[20] = prev_pc;                /* r20 = previous pc */
    s->pc = 0;                         /* reset vector: call evtp+0 (r20),
                                          evtp cleared to 0 */
    s->npc = 4;
    s->pcsh = 0;
    s->ps = 0;
    s->emr = 0;
    s->pcw = (uint16_t)(0x38F | ((straps & 0xFu) << 10));
    s->pcw_locked = 0;
    s->dauc = 0;
    s->ctr = 0;
    s->do_active = 0;
    s->do_lock = 0;
    s->level = DSP3210_LVL_BASE;
    s->error_nonmaskable = 0;
    s->pending = 0;
    s->waiting = 0;
    s->int_defer = 0;
    s->irsh_pending = 0;
    s->irsh = 0;
    s->irsh_addr = 0;
    s->prefetch_addr = 1;
    s->last_vector = -1;
    s->cur_insn = 0;
    memset(s->a_pipe, 0, sizeof s->a_pipe);
    memset(s->dau_flag_pipe, 0, sizeof s->dau_flag_pipe);
    memset(s->da_wr, 0, sizeof s->da_wr);
    s->in_fetch = 0;
    s->ext_pulse[0] = s->ext_pulse[1] = 0;
    /* the EXT pins are active-low: idle pins read negated = 1 */
    s->ps |= DSP3210_PS_IR0 | DSP3210_PS_IR1;
    /* with no SIO modelled, the serial OUTPUT buffer is (vacuously)
     * empty — obe tests "obe=1: output buffer empty" [IM cond table],
     * so a never-written buffer must read empty, or a guest waiting
     * `if (obe)` hangs.  IBF stays 0 (input buffer not full). */
    s->ps |= DSP3210_PS_OBE;
}

void dsp3210_init(dsp3210_emu *s, uint8_t *mem, uint32_t mem_size)
{
    memset(s, 0, sizeof *s);
    s->mem = mem;
    s->mem_size = mem_size;
    dsp3210_reset(s, 0);
}

void dsp3210_request_interrupt(dsp3210_emu *s, int vector)
{
    if (vector >= 8 && vector <= 15)
        s->pending |= (uint16_t)(1u << vector);
}

void dsp3210_ext_pulse(dsp3210_emu *s, int vector, unsigned slots)
{
    int idx;
    if (vector == DSP3210_VEC_EXT0)      idx = 0;
    else if (vector == DSP3210_VEC_EXT1) idx = 1;
    else                                 return;
    s->pending |= (uint16_t)(1u << vector);
    if (slots) {
        s->ext_pulse[idx] = slots;
        s->ps &= (uint16_t)~(idx ? DSP3210_PS_IR1 : DSP3210_PS_IR0);
    }
}

int dsp3210_load(dsp3210_emu *s, uint32_t addr, const void *buf, size_t len)
{
    const uint8_t *b = (const uint8_t *)buf;
    size_t i;
    for (i = 0; i < len; i++) {
        uint8_t *p = mem_ptr(s, addr + (uint32_t)i);
        if (!p)
            return -1;
        *p = b[i];
    }
    return 0;
}

int dsp3210_peek(dsp3210_emu *s, uint32_t addr, int size, uint32_t *out)
{
    return raw_access(s, addr, size, out, 0);
}

int dsp3210_poke(dsp3210_emu *s, uint32_t addr, int size, uint32_t val)
{
    return raw_access(s, addr, size, &val, 1);
}

const char *dsp3210_step_name(int status)
{
    switch (status) {
    case DSP3210_STEP_OK:     return "ok";
    case DSP3210_STEP_WAITI:  return "waiti";
    case DSP3210_STEP_BKPT:   return "bkpt";
    case DSP3210_STEP_DERROR: return "double-error";
    default:                  return "?";
    }
}

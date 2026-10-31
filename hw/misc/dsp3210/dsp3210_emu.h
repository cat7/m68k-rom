/*
 * dsp3210_emu.h — reference emulator core for the AT&T DSP3210
 *
 * Sources of truth:
 *   - AT&T "DSP3210 Information Manual" (Sept 1991): chapter 4 instruction
 *     pages for per-instruction semantics and flag rules (Table 4-1),
 *     chapter 7 for the exception model, chapter 8 for the DAU and the
 *     DSP32 floating-point format, chapter 10 for encodings.
 *   - Hardware-verified encoding and behaviour notes recovered from real
 *     DSP3210 systems (Macintosh Quadra 840AV / Centris 660AV).
 *   - Encodings cross-checked against the companion reference disassembler
 *     (dsp3210dis), which was validated against Apple's shipped DSP code.
 *
 * Scope (version 1):
 *   - Complete instruction-set decode: every one of the 64 top-level
 *     opcodes is either executed or raises the documented illegal-opcode
 *     error (the seven patterns of IM §7.5.3.2).
 *   - CA (integer/branch/move) instructions are implemented to the letter
 *     of the manual, including delayed branches, do-loops, the hard-wired
 *     r0, pc = insn+8, and the 16-bit flag rules.
 *   - DA (floating-point) instructions: the decode and every addressing
 *     side effect (pointer post-modification, memory reads/writes,
 *     Z-field stores) are performed exactly.  The arithmetic itself
 *     comes in TWO drop-in implementations, chosen when the core is
 *     compiled (see dsp3210_emu.c):
 *       . the approximate DAU (default, libdsp3210emu.a) stands in host
 *         doubles for the 40-bit accumulators — at least as precise as
 *         hardware, but guard-bit rounding is not bit-exact;
 *       . the exact DAU (-DDSP3210_DAU_EXACT, libdsp3210emu-exact.a)
 *         models the datapath in integers: 40-bit accumulators, exact
 *         multiplier, truncating adder, documented rounding/saturation.
 *     Both cores expose this same header and the same ABI; portable
 *     code reads accumulators through the accessors below.
 *   - Exception model: the 16-entry vector table at evtp, error vs
 *     interrupt dispatch, emr masking, processing levels, ireturn with
 *     shadow restore, waiti, sftrst.  bkpt halts the emulator.
 *   - Peripherals (SIO, DMA, timer, BIO) are NOT modelled; their MMIO
 *     words behave as plain on-chip memory.
 *
 * Portable C99.  No global state; everything lives in dsp3210_emu.
 */

#ifndef DSP3210_EMU_H
#define DSP3210_EMU_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/* step() return status                                                */

enum {
    DSP3210_STEP_OK = 0,      /* executed one instruction */
    DSP3210_STEP_WAITI,       /* in wait-for-interrupt, nothing pending */
    DSP3210_STEP_BKPT,        /* executed bkpt (spc = (short) r0) */
    DSP3210_STEP_DERROR       /* double error — only RESTN recovers */
};

/* processing levels [IM §7.5] */
enum {
    DSP3210_LVL_BASE = 0,
    DSP3210_LVL_INTERRUPT,
    DSP3210_LVL_ERROR,
    DSP3210_LVL_DERROR
};

/* exception vector numbers [IM Figure 7-5] */
enum {
    DSP3210_VEC_RESET  = 0,
    DSP3210_VEC_BUSERR = 1,   /* non-maskable */
    DSP3210_VEC_ILLOP  = 2,   /* non-maskable */
    DSP3210_VEC_AERR   = 4,   /* maskable, emr[4] */
    DSP3210_VEC_UV     = 5,   /* DAU overflow/underflow, emr[5] */
    DSP3210_VEC_NAN    = 6,   /* IEEE NaN on dsp(), emr[6] */
    DSP3210_VEC_EXT0   = 8,
    DSP3210_VEC_TIMER  = 9,
    DSP3210_VEC_IBF    = 11,
    DSP3210_VEC_OBE    = 12,
    DSP3210_VEC_IFRM   = 13,
    DSP3210_VEC_OFRM   = 14,
    DSP3210_VEC_EXT1   = 15
};

/* ps flag bits [IM Table 10-4] */
#define DSP3210_PS_n    (1u << 0)   /* CAU negative */
#define DSP3210_PS_z    (1u << 1)   /* CAU zero */
#define DSP3210_PS_v    (1u << 2)   /* CAU overflow */
#define DSP3210_PS_c    (1u << 3)   /* CAU carry/borrow */
#define DSP3210_PS_N    (1u << 4)   /* DAU negative */
#define DSP3210_PS_Z    (1u << 5)   /* DAU zero */
#define DSP3210_PS_U    (1u << 6)   /* DAU underflow */
#define DSP3210_PS_V    (1u << 7)   /* DAU overflow */
#define DSP3210_PS_IBF  (1u << 8)
#define DSP3210_PS_OBE  (1u << 9)
#define DSP3210_PS_SY   (1u << 10)
#define DSP3210_PS_FB   (1u << 11)
#define DSP3210_PS_IR0  (1u << 12)
#define DSP3210_PS_IR1  (1u << 13)

/*
 * Optional memory hooks.  When set they replace the built-in flat
 * memory + on-chip window.  size is 1, 2 or 4; addresses are already
 * alignment-checked.  Set *fault nonzero to signal a bus error.
 */
typedef uint32_t (*dsp3210_read_fn)(void *ctx, uint32_t addr, int size,
                                    int *fault);
typedef void     (*dsp3210_write_fn)(void *ctx, uint32_t addr, uint32_t val,
                                     int size, int *fault);

/*
 * A 40-bit DAU accumulator, as the exact DAU models it
 * [IM Figure 8-8B, §8.2.3]:
 *
 *   40-bit word = mantissa (bits 39-16) | guard (15-8) | exponent (7-0)
 *
 * The mantissa+guard field (bits 39-8) is a single 2's complement
 * quantity `s (!s) . f31…f0` — the implicit leading bit is !s, so the
 * represented mantissa M lies in [1,2) or [-2,-1).  `m` holds that
 * value *including* the implicit bit, as a fixed-point integer with 31
 * fractional bits:
 *
 *   value = m * 2^(e - 128) / 2^31,   |m| in [2^31, 2^32)  when normal
 *   value = 0                                              when e == 0
 */
typedef struct {
    int64_t m;      /* mantissa+guard incl. implicit bit, 31 frac bits */
    int16_t e;      /* biased exponent; 0 means the value is zero */
} dsp3210_acc;

/*
 * One accumulator slot in the machine state.  Which member is live
 * depends on the DAU compiled into the core you linked: the approximate
 * DAU keeps a host double in .d, the exact DAU a dsp3210_acc in .x.
 * The union keeps the two cores ABI-identical — portable code never
 * touches the members, only the dsp3210_acc_* accessors below.
 */
typedef union {
    double      d;
    dsp3210_acc x;
} dsp3210_accslot;

struct dsp3210_emu;

typedef struct dsp3210_emu {
    /* --- CAU registers --- */
    uint32_t r[23];        /* r0 (hardwired 0) .. r22; r20 = error trace,
                              r21 = sp, r22 = evtp */
    uint32_t pc;           /* address of the next instruction to execute */
    uint32_t npc;          /* address of the one after (delayed-branch
                              machinery: a taken branch sets npc) */
    uint32_t pcsh;         /* program counter shadow register.  Saved as
                              resume address + 4: the not-yet-executed
                              instruction at the resume address is held
                              in irsh and replayed by ireturn, and pcsh
                              is "the location to be fetched when
                              exiting" [IM §7.5.1, Figure 7-6] */

    /* instruction shadow register: the word prefetched before an
     * interrupt (under the pre-interrupt memory map — the boot ROM's
     * SAR path depends on that), replayed by ireturn */
    uint32_t irsh, irsh_addr;
    int      irsh_pending;
    uint32_t prefetch, prefetch_addr;   /* last word prefetched at npc */

    /* --- IO registers --- */
    uint16_t ps;           /* processor status (flags) */
    uint16_t emr;          /* exception mask register */
    uint16_t pcw;          /* processor control word */
    uint8_t  dauc;         /* DAU control */
    uint8_t  ctr;          /* clip-test register (DAU N history) */
    int      pcw_locked;   /* pcw[13] written 1 → locked until reset/error */

    /* --- DAU accumulators (see dsp3210_accslot above) --- */
    dsp3210_accslot a[4];

    /* --- do-loop state [IM DO page] --- */
    int      do_active;
    int      do_lock;      /* dolock: interrupts disabled for the loop */
    uint32_t do_start, do_end, do_count;

    /* --- exception state [IM §7.5] --- */
    int      level;              /* DSP3210_LVL_* */
    int      error_nonmaskable;  /* type of error being processed */
    uint16_t pending;            /* pending interrupts, bit n = vector n */
    int      waiting;            /* in waiti */
    int      int_defer;          /* run one insn before taking interrupt
                                    (the waiti latent instruction) */
    int      last_vector;        /* last vector raised (incl. masked) */

    /*
     * --- DAU pipeline latency model [IM §4.4.2] ---
     * a_pipe[k]        = the accumulator file as it stood after
     *                    instruction n-1-k; the multiplier reads its
     *                    accumulator inputs from a_pipe[2] (Latency 2)
     * dau_flag_pipe[k] = the ps N/Z/U/V nibble after instruction n-1-k;
     *                    conditional branches/ALU on DAU conditions test
     *                    dau_flag_pipe[3] (Latency 4; ifalt/ifaeq/ifagt
     *                    stay zero-latency per the manual)
     * da_wr            = DA memory-write shadow: a DA store is not
     *                    readable by the DSP itself for the next three
     *                    instructions (Latency 1); reads overlay the
     *                    pre-write bytes while an entry lives.  Memory
     *                    itself is committed immediately, so hosts,
     *                    DMA and faults are unaffected; instruction
     *                    fetch is exempt.  So is hook-backed memory:
     *                    the shadow can only snapshot the built-in map,
     *                    and probing a hook for the pre-write bytes
     *                    could trigger MMIO read side effects, so
     *                    setting read_fn/write_fn disables the shadow
     *                    (model the latency host-side if you need it).
     */
    dsp3210_accslot a_pipe[3][4];
    uint8_t  dau_flag_pipe[4];
    struct dsp3210_da_wr {
        uint32_t addr;
        uint8_t  size, rem, live, old[4];
    } da_wr[4];
    int      in_fetch;           /* fetch in progress: bypass da_wr */

    /*
     * --- external interrupt pins (EXT0/EXT1) ---
     * ps.IR0/IR1 mirror the LIVE pin level (1 = negated: the pins are
     * active-low, so idle pins read set, including right after reset).
     * ext_pulse[] counts how many more core time slots (executed
     * instructions or waiti sleep slots) each pin stays asserted after
     * dsp3210_ext_pulse().  The latched interrupt *request* (`pending`)
     * is separate: taking the interrupt clears the request, never the
     * pin.
     */
    uint32_t ext_pulse[2];       /* [0] = EXT0/vector 8, [1] = EXT1/15 */

    /* interrupt shadow registers [IM §7.5.1; ps/ctr per Figure 4-1] */
    uint16_t sh_ps;
    uint8_t  sh_dauc, sh_ctr;
    dsp3210_accslot sh_a[4];
    dsp3210_accslot sh_a_pipe[3][4];
    uint8_t  sh_dau_flag_pipe[4];
    int      sh_do_active, sh_do_lock;
    uint32_t sh_do_start, sh_do_end, sh_do_count;

    /* --- memory --- */
    uint8_t  *mem;          /* flat external memory at address 0 */
    uint32_t  mem_size;
    uint8_t   chip[0x10000];/* on-chip 64 KB window: at $50030000-$5003FFFF
                               in processor mode (pcw[10]=0).  In computer
                               mode the window is $0000xxxx and simply
                               aliases flat memory.  Boot ROM and MMIO
                               inside the window are plain RAM (stub). */
    dsp3210_read_fn  read_fn;   /* optional overrides */
    dsp3210_write_fn write_fn;
    void            *hook_ctx;
    /* QEMU: I/O hooks, used only for the on-chip MMIO words
     * ($0400-$07FF of the window) and for addresses outside both the
     * window and the flat memory; plain memory keeps the built-in map
     * (and with it the DA store shadow) */
    dsp3210_read_fn  io_read_fn;
    dsp3210_write_fn io_write_fn;

    /* --- bookkeeping --- */
    uint64_t icount;
    uint32_t cur_insn;      /* address of the instruction being executed */
} dsp3210_emu;

/*
 * Initialise: attach a flat memory buffer (may be NULL/0 if hooks are
 * used) and perform a hardware reset.  straps = the BIO7..BIO4 reset
 * straps as the value latched into pcw[10..13]: bit 0 → pcw[10] (C/PN),
 * bit 1 → pcw[11], bit 2 → pcw[12], bit 3 → pcw[13] (BRC).
 * The AV Macs strap 0 on bit 0 (processor mode); pass 0 for that.
 */
void dsp3210_init(dsp3210_emu *s, uint8_t *mem, uint32_t mem_size);
void dsp3210_reset(dsp3210_emu *s, unsigned straps);

/* Execute one instruction (or report the blocked state). */
int dsp3210_step(dsp3210_emu *s);

/* Assert an interrupt request (vector 8..15).  It is taken when the
 * corresponding emr bit is set and the processor is at base level.
 * This latches the REQUEST only; it does not touch the ps IR0/IR1 pin
 * bits — use dsp3210_ext_pulse for pin-accurate EXT0/EXT1 modelling. */
void dsp3210_request_interrupt(dsp3210_emu *s, int vector);

/*
 * Pulse an external interrupt pin (vector 8 = EXT0, 15 = EXT1): latch
 * the interrupt request and assert the live pin — ps.IR0/IR1 reads 0
 * (asserted) — for `slots` units of core time, where one slot elapses
 * per executed instruction and per waiti sleep step.  Guest code that
 * polls the pin level (`if (ir1s) goto …`) sees a pulse of that width.
 * Note: a write to emr with bit 0 set drops a latched-but-untaken EXT1
 * request without taking it (hardware-verified kernel behaviour); the
 * pin level is unaffected.
 */
void dsp3210_ext_pulse(dsp3210_emu *s, int vector, unsigned slots);

/* Copy bytes into emulated memory through the address map (for loading
 * program images).  Returns 0, or -1 if part of the range is unmapped. */
int dsp3210_load(dsp3210_emu *s, uint32_t addr, const void *buf, size_t len);

/* Raw memory peek/poke through the address map (endianness per pcw[8]).
 * Return 0 on success, -1 on unmapped/misaligned.  No exceptions raised. */
int dsp3210_peek(dsp3210_emu *s, uint32_t addr, int size, uint32_t *out);
int dsp3210_poke(dsp3210_emu *s, uint32_t addr, int size, uint32_t val);

/* Uniform accumulator accessors, provided by both DAU implementations
 * so shared sources compile and link against either core. */
double dsp3210_acc_get(const struct dsp3210_emu *s, int n);
void   dsp3210_acc_set(struct dsp3210_emu *s, int n, double v);
/* Raw 40-bit accumulator fields; the approximate DAU synthesises them
 * from its double, the exact DAU returns the real register. */
void   dsp3210_acc_raw(const struct dsp3210_emu *s, int n,
                       int64_t *mant_guard, int *exp);

/* DSP32 32-bit floating-point format [IM §3.4.2] <-> host double.
 * Bit-identical in both cores (ties resolve toward +infinity, the
 * direction the DSP's own `round` instruction uses). */
double   dsp3210_dsp32_to_double(uint32_t w);
uint32_t dsp3210_double_to_dsp32(double v);

/*
 * Exact-DAU extras: build and inspect accumulators directly.
 *
 * These four are declared unconditionally (so you need not define
 * DSP3210_DAU_EXACT in your own sources — it is a build flag for the
 * core, not for its callers) but are DEFINED ONLY in
 * libdsp3210emu-exact.a.  Calling them in a program linked against the
 * default libdsp3210emu.a compiles cleanly and fails at link time with
 * an undefined reference; there is no approximate-DAU equivalent
 * because that core has no 40-bit accumulator to expose.  Code that
 * must work against either core uses dsp3210_acc_get/_set/_raw above.
 */
double      dsp3210_acc_double(dsp3210_acc a);
dsp3210_acc dsp3210_acc_from_double(double v);
/* accumulator <-> packed 32-bit DSP32 word (guard bits truncated) */
uint32_t    dsp3210_acc_pack(dsp3210_acc a);
dsp3210_acc dsp3210_acc_unpack(uint32_t w);

const char *dsp3210_step_name(int status);

#ifdef __cplusplus
}
#endif

#endif /* DSP3210_EMU_H */

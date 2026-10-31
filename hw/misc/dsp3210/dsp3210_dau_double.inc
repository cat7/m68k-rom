/*
 * dsp3210_dau_double.inc — the approximate DAU: host doubles.
 *
 * Not compiled on its own: dsp3210_emu.c #includes exactly one of
 * dsp3210_dau_double.inc / dsp3210_dau_exact.inc (this one unless
 * DSP3210_DAU_EXACT is defined), so the DAU implementation composes
 * with the shared core in a single translation unit and everything
 * stays static.
 *
 * This DAU approximates the 40-bit accumulators with host doubles
 * (accumulator slot member `.d`).  A double's 53-bit mantissa strictly
 * contains the hardware's 32-bit mantissa + guard bits, so results are
 * at least as precise as hardware, but guard-bit rounding is not
 * bit-exact.  See README "Which DAU?".
 */

/* ------------------------------------------------------------------ */
/* DSP32 32-bit floating point [IM §3.4.2, §8.2.2; DOC §1.5.6]
 *
 * word = mantissa[31:8] . exponent[7:0]
 * mantissa is a single 2's-complement quantity s(!s).fff…f (implicit
 * second bit = !s), value = m * 2^(e-128), e unsigned, e = 0 is zero.  */

double dsp3210_dsp32_to_double(uint32_t w)
{
    unsigned e = w & 0xFFu;
    int32_t mant;
    double m;
    if (e == 0)
        return 0.0;                          /* incl. "dirty zeros" */
    mant = (int32_t)(w & 0xFFFFFF00u) >> 8;  /* signed fraction bits */
    m = (double)mant * 0x1p-23 + (mant >= 0 ? 1.0 : -1.0);
    return ldexp(m, (int)e - 128);
}

/* The DAU judges overflow against the 40-bit range, not the 32-bit one
 * [Table 4-1, DOC §1.5.6]: the accumulator carries 8 guard bits, so the
 * largest magnitude is (2 - 2^-31) * 2^127.  (The saturation constants
 * this core clamps to are still the 32-bit ones — see its README.) */
#define DSP32_MAX ((2.0 - 0x1p-31) * 0x1p127)   /* 40-bit range */
#define DSP32_SAT ((2.0 - 0x1p-23) * 0x1p127)   /* 0x7FFFFFFF */
#define DSP32_MIN_POS 0x1p-127                  /* e = 1, f = 0 */
#define DSP32_MIN_NEG ((1.0 + 0x1p-31) * 0x1p-127) /* largest (closest to
                                                      zero) negative:
                                                      M = -(1+2^-31), e=1 */

uint32_t dsp3210_double_to_dsp32(double v)
{
    int exp;
    double m;
    long f;
    unsigned e;
    uint32_t mant24;

    if (v == 0.0 || !isfinite(v))
        return v > 0 ? 0x7FFFFFFFu : (isnan(v) || v == 0.0) ? 0
                                                            : 0x800000FFu;
    m = frexp(v, &exp);                      /* v = m * 2^exp, |m|∈[.5,1) */
    m *= 2.0;
    exp -= 1;                                /* |m| ∈ [1,2) */
    if (m >= 0) {
        f = (long)floor((m - 1.0) * 0x1p23 + 0.5);
        if (f == 1L << 23) { f = 0; exp++; }
        mant24 = (uint32_t)f;                /* 0.f, sign bit clear */
    } else {
        if (m == -1.0) { m = -2.0; exp--; }  /* renormalise: -1 = -2*2^-1 */
        f = (long)floor((m + 2.0) * 0x1p23 + 0.5);
        if (f == 1L << 23) { f = 0; exp--; } /* rounded to -1 → -2*2^-1 */
        mant24 = 0x800000u | (uint32_t)f;    /* 1(0).f */
    }
    if (exp + 128 > 255)                     /* saturate [IM §8.2.4.1] */
        return v > 0 ? 0x7FFFFFFFu : 0x800000FFu;
    if (exp + 128 < 1)
        return 0;                            /* flush to zero */
    e = (unsigned)(exp + 128);
    return (mant24 << 8) | e;
}

/* ------------------------------------------------------------------ */
/* DAU flags + clip-test history                                       */

static double da_flags(dsp3210_emu *s, double res, int affect_vu)
{
    unsigned N, Z, V = 0, U = 0;
    if (affect_vu) {
        if (fabs(res) > DSP32_MAX) {         /* 40-bit range ≈ ±2·2^127 */
            V = 1;
            res = res < 0 ? -DSP32_SAT : DSP32_SAT;
        } else if (res != 0.0 && (res > 0.0 ? res < DSP32_MIN_POS
                                             : res > -DSP32_MIN_NEG)) {
            /* the ranges are asymmetric [IM §7.5.3.4]: negative
             * magnitudes start at (1+2^-31)*2^-127, so -(1*2^-127) is in
             * the underflow gap */
            U = 1;
            res = 0.0;                       /* flush to zero */
        }
    }
    N = res < 0.0;
    Z = res == 0.0;
    s->ps = (uint16_t)((s->ps & ~0xF0u)
          | (N ? DSP3210_PS_N : 0) | (Z ? DSP3210_PS_Z : 0)
          | (U ? DSP3210_PS_U : 0) | (V ? DSP3210_PS_V : 0));
    s->ctr = (uint8_t)(((s->ctr << 1) | N) & 0x3F);  /* [IM Table 8-4] */
    if (V || U)
        take_error(s, DSP3210_VEC_UV);       /* masked unless emr[5] */
    return res;
}

/* ------------------------------------------------------------------ */
/* DA operands (7-bit X/Y/Z fields) [IM Table 10-3; DOC §1.5.3]        */

typedef struct {
    int      from_acc;   /* 1: accumulator (or reserved field) */
    unsigned acc;        /* accumulator number when from_acc */
    uint32_t raw;        /* raw bits read when memory */
    double   val;        /* numeric value (interpretation per caller) */
    int      is_mem;     /* 1: memory operand; ea/preg below valid */
    uint32_t ea;         /* effective address (before post-modify) */
    unsigned preg;       /* pointer register number (1-14) */
} da_opnd;

static void da_postmod(dsp3210_emu *s, unsigned p, unsigned i, int size)
{
    switch (i & 7) {
    case 0: break;
    case 6: s->r[p] -= (uint32_t)size; break;
    case 7: s->r[p] += (uint32_t)size; break;
    default: s->r[p] += s->r[14 + (i & 7)]; break;   /* r15-r19, unscaled */
    }
}

/* Read an X/Y operand of the given size.  Returns -1 on a memory fault
 * (exception already raised).  Memory words are NOT interpreted here —
 * the caller converts .raw as float/int/byte as the instruction needs;
 * .val is filled with the accumulator value for register-direct fields. */
static int da_read(dsp3210_emu *s, unsigned f, int size, da_opnd *o)
{
    unsigned p = (f >> 3) & 15, i = f & 7;
    memset(o, 0, sizeof *o);
    if (p == 0 || p == 15) {           /* register direct (or reserved) */
        o->from_acc = 1;
        o->acc = i & 3;
        o->val = s->a[i & 3].d;
        return 0;
    }
    o->is_mem = 1;
    o->ea = s->r[p];
    o->preg = p;
    if (mem_read(s, s->r[p], size, &o->raw))
        return -1;
    da_postmod(s, p, i, size);
    return 0;
}

/*
 * The Z field's store behaviour (hardware-verified against shipped
 * in-place converts and buffer scales; the manual's Table 10-3 calls
 * p=1111 "not allowed"):
 *   p = 0001-1110  store through the Z field's own pointer register;
 *   p = 1111       store through the *Y operand's* own effective
 *                  address, the Z I bits post-modifying Y's POINTER
 *                  register — with an accumulator Y there is nothing
 *                  to store through and no write happens (the source
 *                  of the old "0x7F = no write" misreading);
 *   p = 0000       never stores.
 */
static int da_z_writes(unsigned z, const da_opnd *y)
{
    unsigned p = (z >> 3) & 15;
    if (p == 0)
        return 0;
    if (p != 15)
        return 1;
    return y->is_mem;
}

static int da_store_z(dsp3210_emu *s, unsigned z, const da_opnd *y,
                      int size, uint32_t val)
{
    unsigned p = (z >> 3) & 15, i = z & 7;
    if (p == 0)
        return 0;
    if (p == 15) {                     /* through Y's effective address */
        if (!y->is_mem)
            return 0;
        if (da_shadowed_write(s, y->ea, size, val))
            return -1;
        da_postmod(s, y->preg, i, size);
        return 0;
    }
    if (da_shadowed_write(s, s->r[p], size, val))
        return -1;
    da_postmod(s, p, i, size);
    return 0;
}

/* Perform a Z field's address post-modification WITHOUT storing.  The
 * ifalt/ifaeq/ifagt pages are explicit: with dauc[6] = 1 the memory
 * write is conditional "but the address postmodifications are always
 * performed". */
static void da_postmod_only(dsp3210_emu *s, unsigned z, const da_opnd *y,
                            int size)
{
    unsigned p = (z >> 3) & 15, i = z & 7;
    if (p == 0)
        return;
    if (p == 15) {
        if (y->is_mem)
            da_postmod(s, y->preg, i, size);
        return;
    }
    da_postmod(s, p, i, size);
}

/* multiplier-lane operand value [IM §4.4.2.2, Latency 2]: an
 * accumulator feeding the MULTIPLIER reads the value established three
 * instructions back (a_pipe[2]); memory operands, and accumulators
 * feeding the adder, are unaffected. */
static double da_float_mul(const dsp3210_emu *s, const da_opnd *o)
{
    if (o->from_acc)
        return s->a_pipe[2][o->acc].d;
    return dsp3210_dsp32_to_double(o->raw);
}

/*
 * The accumulator's integer lane.  The INT32/INT16/OC
 * pages are explicit: "the result is stored in the 32 MSBs of an
 * accumulator (aN), the mantissa, and guard bits" / "the upper 16
 * bits" / "the most significant byte" — only the exponent is
 * unpredictable.  That lane (bits 39-8) is also where ic/float16/
 * float32 read their accumulator operand, so float32(int32(x)) is a
 * DEFINED round trip that shipped code uses (the RTM sound-input rate
 * converter splits its phase accumulator with it).
 *
 * This core models accumulators as doubles, so the lane is
 * synthesised: acc_lane_of() extracts the mantissa+guard field a
 * value would have, acc_int_lane_d() builds the value an integer
 * deposit leaves behind (keeping the adder input's exponent — a zero
 * exponent means "zero" throughout the model and would erase the
 * lane; the manual calls the exponent unpredictable, so any choice
 * that keeps the lane intact is faithful).
 */
static int acc_expfield_of(double v)           /* 0 means "zero" */
{
    int e;
    double m;
    if (v == 0.0)
        return 0;
    m = frexp(v, &e) * 2.0;
    e -= 1;                                    /* |m| in [1,2) */
    if (m == -1.0)
        e -= 1;                                /* -1 = -2 * 2^-1 */
    return e + 128;
}

static int32_t acc_lane_of(double v)           /* mantissa+guard bits */
{
    int e;
    double m;
    int64_t f;
    if (v == 0.0)
        return 0;
    m = frexp(v, &e) * 2.0;                    /* |m| in [1,2) */
    if (m >= 0) {
        f = (int64_t)floor((m - 1.0) * 2147483648.0 + 0.5);
        if (f == (int64_t)1 << 31)
            f = 0;                             /* rounded up: lane wraps */
        return (int32_t)f;
    }
    if (m == -1.0)
        m = -2.0;
    f = (int64_t)floor((m + 2.0) * 2147483648.0 + 0.5);
    if (f == (int64_t)1 << 31)
        f = 0;
    return (int32_t)(0x80000000u | (uint32_t)f);
}

static double acc_int_lane_d(int32_t lane, double adder)
{
    int ef = acc_expfield_of(adder);
    int64_t m = (lane >= 0) ? (int64_t)lane + ((int64_t)1 << 31)
                            : (int64_t)lane - ((int64_t)1 << 31);
    if (ef == 0)
        return 0.0;                            /* e = 0 reads as zero */
    return ldexp((double)m, ef - 128 - 31);
}

/* interpret a MAC operand (32-bit float) */
static double da_float(const da_opnd *o)
{
    return o->from_acc ? o->val : dsp3210_dsp32_to_double(o->raw);
}

/* ------------------------------------------------------------------ */
/* companded conversions (ic/oc) [IM §8.2.4.2]
 *
 * µ-law byte (bit 0 first): ~m0 ~m1 ~m2 ~m3 ~n0 ~n1 ~n2 ~s
 *   Y = (-1)^s * ((16.5+M)*2^N - 16.5)
 * A-law byte:               ~m0 m1 ~m2 m3 ~n0 n1 ~n2 ~s   (XOR 0xD5)
 *   Y = (-1)^s * (16.5+M)*2^N   (N>=1);   (-1)^s * (0.5+M)*2   (N=0)
 */

static double mulaw_decode(unsigned b)
{
    unsigned u = ~b & 0xFFu;
    unsigned M = u & 15, N = (u >> 4) & 7, S = (u >> 7) & 1;
    double y = (16.5 + M) * (double)(1u << N) - 16.5;
    return S ? -y : y;
}

static double alaw_decode(unsigned b)
{
    unsigned u = (b ^ 0xD5u) & 0xFFu;
    unsigned M = u & 15, N = (u >> 4) & 7, S = (u >> 7) & 1;
    double y = N ? (16.5 + M) * (double)(1u << N) : (0.5 + M) * 2.0;
    return S ? -y : y;
}

/* nearest-code encode by exhaustive search (256 codes; performance is a
 * non-goal, exactness with respect to the decode formula is) */
static unsigned companded_encode(double y, int alaw)
{
    unsigned b, best = 0;
    double bestd = HUGE_VAL;
    for (b = 0; b < 256; b++) {
        double d = fabs((alaw ? alaw_decode(b) : mulaw_decode(b)) - y);
        if (d < bestd) { bestd = d; best = b; }
    }
    return best;
}

/* float → integer rounding mode [IM Table 8-3]: dauc[4]=0 round-to-
 * nearest (ties up), dauc[5:4]=01 truncate to -∞, 11 truncate to 0 */
static double fp_round(dsp3210_emu *s, double y)
{
    unsigned r = (s->dauc >> 4) & 3;
    if (!(r & 1))
        return floor(y + 0.5);
    if (r == 1)
        return floor(y);
    return trunc(y);
}

/* ------------------------------------------------------------------ */
/* DA special functions (format 5) [IM Table 4-4]                      */

enum {
    G_IC = 0, G_OC = 1, G_FLOAT16 = 2, G_INT16 = 3, G_ROUND = 4,
    G_IFALT = 5, G_IFAEQ = 6, G_IFAGT = 7, G_FLOAT32 = 8, G_INT32 = 9,
    G_IEEE = 12, G_DSP = 13, G_SEED = 14
};

static uint32_t ieee_bits_from_double(double v)
{
    float f = (float)v;
    uint32_t w;
    memcpy(&w, &f, 4);
    return w;
}

static int exec_da_special(dsp3210_emu *s, uint32_t w)
{
    unsigned g = bits(w, 26, 23);
    unsigned n = bits(w, 22, 21);
    unsigned y = bits(w, 13, 7);
    unsigned z = bits(w, 6, 0);
    int zw;
    int ysz = (g == G_IC) ? 1 : (g == G_FLOAT16) ? 2 : 4;
    int zsz = (g == G_OC) ? 1 : (g == G_INT16) ? 2 : 4;
    da_opnd Y;
    uint32_t zbits = 0;
    double res = s->a[n].d;

    if (da_read(s, y, ysz, &Y))
        return DSP3210_STEP_OK;
    zw = da_z_writes(z, &Y);

    /* NOTE (v1 stub deviation): accumulators are modelled as numeric
     * doubles, so instructions whose hardware result is a raw bit
     * pattern parked in accumulator bit positions (oc, int16, int32,
     * ieee) leave the *numeric* result in aN instead.  The Z memory
     * write — which the manual requires to be done in the same
     * instruction anyway — is produced with the correct bits. */
    switch (g) {
    case G_IC: {                               /* byte → float, NZ00 */
        /* IC page: "If Y specifies an accumulator, the 8-bit word must
         * reside in the most significant byte of the accumulator" —
         * the top byte of the integer lane (bits 39-32). */
        unsigned code = Y.from_acc
                      ? (((uint32_t)acc_lane_of(Y.val) >> 24) & 0xFFu)
                      : (Y.raw & 0xFFu);
        unsigned conv = s->dauc & 15;
        if (conv & 4)                          /* x1xx unsigned linear in */
            res = (double)code;
        else if (conv & 1)                     /* x0x1 A-law in */
            res = alaw_decode(code);
        else                                   /* x0x0 µ-law in */
            res = mulaw_decode(code);
        res = da_flags(s, res, 0);
        zbits = dsp3210_double_to_dsp32(res);
        break;
    }
    case G_OC: {                               /* float → byte, no flags */
        double yv = da_float(&Y);
        unsigned conv = s->dauc & 15, code;
        if (conv & 8) {                        /* 1xxx unsigned linear out */
            double t = fp_round(s, yv);
            code = t < 0 ? 0 : t > 255 ? 255 : (unsigned)t;
        } else if (conv & 2) {                 /* 0x1x A-law out */
            code = companded_encode(yv, 1);
        } else {                               /* 0x0x µ-law out */
            code = companded_encode(yv, 0);
        }
        res = acc_int_lane_d((int32_t)(code << 24), yv);  /* integer lane */
        zbits = code;
        break;
    }
    case G_FLOAT16: {                          /* int16 → float, NZ00 */
        int32_t iv = Y.from_acc
                   ? (int32_t)(int16_t)((uint32_t)acc_lane_of(Y.val) >> 16)
                   : (int32_t)(int16_t)(Y.raw & 0xFFFFu);
        res = da_flags(s, (double)iv, 0);
        zbits = dsp3210_double_to_dsp32(res);
        break;
    }
    case G_FLOAT32: {                          /* int32 → float, NZ00 */
        int32_t iv = Y.from_acc ? acc_lane_of(Y.val)
                                : (int32_t)Y.raw;
        res = da_flags(s, (double)iv, 0);
        zbits = dsp3210_double_to_dsp32(res);
        break;
    }
    case G_INT16: {                            /* float → int16, no flags */
        double t = fp_round(s, da_float(&Y));
        int32_t iv = t < -32768.0 ? -32768 : t > 32767.0 ? 32767
                                                         : (int32_t)t;
        res = acc_int_lane_d((int32_t)((uint32_t)iv << 16),
                             da_float(&Y));                /* int lane */
        zbits = (uint32_t)iv & 0xFFFFu;
        break;
    }
    case G_INT32: {                            /* float → int32, no flags */
        double t = fp_round(s, da_float(&Y));
        int32_t iv = t < -2147483648.0 ? INT32_MIN
                   : t >  2147483647.0 ? INT32_MAX : (int32_t)t;
        res = acc_int_lane_d(iv, da_float(&Y));           /* int lane */
        zbits = (uint32_t)iv;
        break;
    }
    case G_ROUND: {                            /* float40 → float32, NZVU */
        uint32_t q = dsp3210_double_to_dsp32(da_float(&Y));
        res = da_flags(s, dsp3210_dsp32_to_double(q), 1);
        zbits = dsp3210_double_to_dsp32(res);
        break;
    }
    case G_IFALT:
    case G_IFAEQ:
    case G_IFAGT: {                            /* conditional load */
        unsigned AN = !!(s->ps & DSP3210_PS_N), AZ = !!(s->ps & DSP3210_PS_Z);
        int cond = (g == G_IFALT) ? AN
                 : (g == G_IFAEQ) ? AZ
                                  : !(AN | AZ);
        if (cond)
            res = da_float(&Y);
        zbits = dsp3210_double_to_dsp32(res);
        if (zw && (s->dauc & 0x40u) && !cond) {
            /* dauc[6] = 1: the store is conditional, but the pointer
             * post-modification still happens [IM IFALT/IFAEQ/IFAGT] */
            da_postmod_only(s, z, &Y, zsz);
            zw = 0;
        }
        break;
    }
    case G_IEEE: {                             /* DSP32 → IEEE, no flags */
        double yv = da_float(&Y);
        zbits = ieee_bits_from_double(yv);
        res = yv;                              /* numeric-acc deviation */
        break;
    }
    case G_DSP: {                              /* IEEE → DSP32, NZVU/NaN */
        uint32_t iw = Y.from_acc ? ieee_bits_from_double(Y.val) : Y.raw;
        unsigned sgn = iw >> 31, exp = (iw >> 23) & 0xFFu;
        uint32_t mant = iw & 0x7FFFFFu;
        if (exp == 255) {                      /* ±∞ / NaN [DSP page] */
            zbits = sgn ? 0x800000FFu : 0x7FFFFFFFu;
            res = dsp3210_dsp32_to_double(zbits);
            s->ps = (uint16_t)((s->ps & ~0xF0u)
                  | (sgn ? DSP3210_PS_N : 0)
                  | ((!sgn || mant) ? DSP3210_PS_V : 0));
            s->ctr = (uint8_t)(((s->ctr << 1) | (sgn ? 1 : 0)) & 0x3F);
            if (mant)
                take_error(s, DSP3210_VEC_NAN);
        } else if (exp == 0) {                 /* zero / denormal → +0 */
            res = 0.0;
            s->ps = (uint16_t)((s->ps & ~0xF0u) | DSP3210_PS_Z
                  | (mant ? DSP3210_PS_U : 0));
            s->ctr = (uint8_t)((s->ctr << 1) & 0x3F);
            zbits = 0;
        } else {
            float f;
            memcpy(&f, &iw, 4);
            res = da_flags(s, (double)f, 1);
            zbits = dsp3210_double_to_dsp32(res);
        }
        break;
    }
    case G_SEED: {                             /* reciprocal seed, NZU V=0 */
        uint32_t yb = Y.from_acc ? dsp3210_double_to_dsp32(Y.val) : Y.raw;
        uint32_t sb = yb ^ 0x7FFFFFFFu;        /* invert all but sign */
        res = dsp3210_dsp32_to_double(sb);
        res = da_flags(s, res, 0);
        zbits = sb;
        break;
    }
    default:                                   /* reserved G codes */
        return DSP3210_STEP_OK;
    }

    s->a[n].d = res;
    if (zw)
        da_store_z(s, z, &Y, zsz, zbits);
    return DSP3210_STEP_OK;
}

/* ------------------------------------------------------------------ */
/* DA multiply/accumulate (formats 1-4) [IM Table 4-2]                 */

static int exec_da(dsp3210_emu *s, uint32_t w)
{
    unsigned fmt = bits(w, 31, 29);
    unsigned m   = bits(w, 28, 26);
    unsigned n   = bits(w, 22, 21);
    unsigned xf  = bits(w, 20, 14);
    unsigned yf  = bits(w, 13, 7);
    unsigned zf  = bits(w, 6, 0);
    unsigned fs  = bits(w, 24, 24);    /* sign before the adder operand */
    unsigned ss  = bits(w, 23, 23);    /* sign of the product term */
    int zw;
    da_opnd X, Y;
    double adder, prod, res;
    int tap;

    if (fmt == 3 && m >= 6)
        return exec_da_special(s, w);

    /* Operand fetch order is X then Y: the X and Y registers are loaded
     * in machine states 1 and 2 of the same instruction cycle
     * [IM §8.2.6, Figure 8-12].  It only matters when one pointer
     * register is post-modified in both fields (which the assembler
     * rejects), but there is no reason not to match the hardware. */
    if (da_read(s, xf, 4, &X))
        return DSP3210_STEP_OK;
    if (da_read(s, yf, 4, &Y))
        return DSP3210_STEP_OK;
    zw = da_z_writes(zf, &Y);

    /* the multiplier's inputs — X always, Y in formats 2/3, and the aM
     * multiplicand of format 1 — read accumulators three instructions
     * stale (Latency 2); adder inputs read them live [IM §4.4.2.2] */
    if (fmt == 1 && m == 6) {
        /* FADD-TAP: aN = [-](Z=Y) {+,-} X */
        adder = da_float(&Y);
        prod  = da_float_mul(s, &X);
        tap = 1;
    } else if (fmt == 1) {
        /* FMULT-ADD-STORE family: aN = [-]Y {+,-} {aM,0,1} * X */
        adder = da_float(&Y);
        prod  = (m <= 3 ? s->a_pipe[2][m].d : m == 4 ? 0.0 : 1.0)
              * da_float_mul(s, &X);
        tap = 0;
    } else {
        /* fmt 2 (tap) / fmt 3 (store): aN = [-]{aM,0,1} {+,-} Y*X */
        adder = (m <= 3 ? s->a[m].d : m == 4 ? 0.0 : 1.0);
        prod  = da_float_mul(s, &Y) * da_float_mul(s, &X);
        tap = (fmt == 2);
    }

    res = (fs ? -adder : adder) + (ss ? -prod : prod);
    res = da_flags(s, res, 1);
    s->a[n].d = res;

    if (zw) {
        uint32_t zbits = tap
            ? (Y.from_acc ? dsp3210_double_to_dsp32(Y.val) : Y.raw)
            : dsp3210_double_to_dsp32(res);
        da_store_z(s, zf, &Y, 4, zbits);
    }
    return DSP3210_STEP_OK;
}

double dsp3210_acc_get(const dsp3210_emu *s, int n)
{
    return s->a[n & 3].d;
}

void dsp3210_acc_set(dsp3210_emu *s, int n, double v)
{
    s->a[n & 3].d = v;
    /* flush the multiplier-lane pipe so host pokes are instantaneous */
    s->a_pipe[0][n & 3].d = v;
    s->a_pipe[1][n & 3].d = v;
    s->a_pipe[2][n & 3].d = v;
}

/* Synthesised from the double at the accumulator's full 31-fraction-
 * bit lane precision (the 24-bit pack would truncate the guard byte —
 * visible on the integer lane an int32/int16/oc deposit leaves
 * there). */
void dsp3210_acc_raw(const dsp3210_emu *s, int n, int64_t *mant_guard,
                     int *exp)
{
    double v = s->a[n & 3].d;
    if (mant_guard)
        *mant_guard = (int64_t)(uint32_t)acc_lane_of(v);
    if (exp)
        *exp = acc_expfield_of(v);
}


/*
 *  m68k op helpers
 *
 *  Copyright (c) 2006-2007 CodeSourcery
 *  Written by Paul Brook
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "cpu.h"
#ifndef CONFIG_USER_ONLY
#include "system/calib-governor.h"
#endif
#include "exec/cputlb.h"
#include "exec/page-protection.h"
#include "exec/target_page.h"
#include "exec/gdbstub.h"
#include "exec/helper-proto.h"
#include "accel/tcg/cpu-loop.h"
#include "accel/tcg/cpu-ldst.h"
#include "qemu/log.h"
#include "system/memory.h"
#include "gdbstub/helpers.h"
#include "fpu/softfloat.h"
#include "qemu/qemu-print.h"
#include "hw/core/irq.h"

#define SIGNBIT (1u << 31)

static int cf_fpu_gdb_get_reg(CPUState *cs, GByteArray *mem_buf, int n)
{
    M68kCPU *cpu = M68K_CPU(cs);
    CPUM68KState *env = &cpu->env;

    if (n < 8) {
        /* Use scratch float_status so any exceptions don't change CPU state */
        float_status s = env->fp_status;
        return gdb_get_reg64(mem_buf, floatx80_to_float64(env->fregs[n].d, &s));
    }
    switch (n) {
    case 8: /* fpcontrol */
        return gdb_get_reg32(mem_buf, env->fpcr);
    case 9: /* fpstatus */
        return gdb_get_reg32(mem_buf, env->fpsr);
    case 10: /* fpiar, not implemented */
        return gdb_get_reg32(mem_buf, 0);
    }
    return 0;
}

static int cf_fpu_gdb_set_reg(CPUState *cs, uint8_t *mem_buf, int n)
{
    M68kCPU *cpu = M68K_CPU(cs);
    CPUM68KState *env = &cpu->env;

    if (n < 8) {
        /* Use scratch float_status so any exceptions don't change CPU state */
        float_status s = env->fp_status;
        env->fregs[n].d = float64_to_floatx80(ldq_be_p(mem_buf), &s);
        return 8;
    }
    switch (n) {
    case 8: /* fpcontrol */
        cpu_m68k_set_fpcr(env, ldl_be_p(mem_buf));
        return 4;
    case 9: /* fpstatus */
        env->fpsr = ldl_be_p(mem_buf);
        return 4;
    case 10: /* fpiar, not implemented */
        return 4;
    }
    return 0;
}

static int m68k_fpu_gdb_get_reg(CPUState *cs, GByteArray *mem_buf, int n)
{
    M68kCPU *cpu = M68K_CPU(cs);
    CPUM68KState *env = &cpu->env;

    if (n < 8) {
        int len = gdb_get_reg16(mem_buf, env->fregs[n].l.upper);
        len += gdb_get_reg16(mem_buf, 0);
        len += gdb_get_reg64(mem_buf, env->fregs[n].l.lower);
        return len;
    }
    switch (n) {
    case 8: /* fpcontrol */
        return gdb_get_reg32(mem_buf, env->fpcr);
    case 9: /* fpstatus */
        return gdb_get_reg32(mem_buf, cpu_m68k_get_fpsr(env));
    case 10: /* fpiar, not implemented */
        return gdb_get_reg32(mem_buf, 0);
    }
    return 0;
}

static int m68k_fpu_gdb_set_reg(CPUState *cs, uint8_t *mem_buf, int n)
{
    M68kCPU *cpu = M68K_CPU(cs);
    CPUM68KState *env = &cpu->env;

    if (n < 8) {
        env->fregs[n].l.upper = lduw_be_p(mem_buf);
        env->fregs[n].l.lower = ldq_be_p(mem_buf + 4);
        return 12;
    }
    switch (n) {
    case 8: /* fpcontrol */
        cpu_m68k_set_fpcr(env, ldl_be_p(mem_buf));
        return 4;
    case 9: /* fpstatus */
        cpu_m68k_set_fpsr(env, ldl_be_p(mem_buf));
        return 4;
    case 10: /* fpiar, not implemented */
        return 4;
    }
    return 0;
}

void m68k_cpu_init_gdb(M68kCPU *cpu)
{
    CPUState *cs = CPU(cpu);
    CPUM68KState *env = &cpu->env;

    if (m68k_feature(env, M68K_FEATURE_CF_FPU)) {
        gdb_register_coprocessor(cs, cf_fpu_gdb_get_reg, cf_fpu_gdb_set_reg,
                                 gdb_find_static_feature("cf-fp.xml"));
    } else if (m68k_feature(env, M68K_FEATURE_FPU)) {
        gdb_register_coprocessor(cs, m68k_fpu_gdb_get_reg, m68k_fpu_gdb_set_reg,
                                 gdb_find_static_feature("m68k-fp.xml"));
    }
    /* TODO: Add [E]MAC registers.  */
}

void HELPER(cf_movec_to)(CPUM68KState *env, uint32_t reg, uint32_t val)
{
    switch (reg) {
    case M68K_CR_CACR:
        env->cacr = val;
        m68k_switch_sp(env);
        break;
    case M68K_CR_ACR0:
    case M68K_CR_ACR1:
    case M68K_CR_ACR2:
    case M68K_CR_ACR3:
        /* TODO: Implement Access Control Registers.  */
        break;
    case M68K_CR_VBR:
        env->vbr = val;
        break;
    /* TODO: Implement control registers.  */
    default:
        cpu_abort(env_cpu(env),
                  "Unimplemented control register write 0x%x = 0x%x\n",
                  reg, val);
    }
}

static void raise_exception_ra(CPUM68KState *env, int tt, uintptr_t raddr)
{
    CPUState *cs = env_cpu(env);

    cs->exception_index = tt;
    cpu_loop_exit_restore(cs, raddr);
}

void HELPER(m68k_movec_to)(CPUM68KState *env, uint32_t reg, uint32_t val)
{
    switch (reg) {
    /* MC680[12346]0 */
    case M68K_CR_SFC:
        env->sfc = val & 7;
        return;
    /* MC680[12346]0 */
    case M68K_CR_DFC:
        env->dfc = val & 7;
        return;
    /* MC680[12346]0 */
    case M68K_CR_VBR:
        env->vbr = val;
        return;
    /* MC680[2346]0 */
    case M68K_CR_CACR:
        if (m68k_feature(env, M68K_FEATURE_M68020)) {
            env->cacr = val & 0x0000000f;
        } else if (m68k_feature(env, M68K_FEATURE_M68030)) {
            env->cacr = val & 0x00003f1f;
            if ((val & 0x8) && env->mmu.icache_valid) {
                /* CI: the instruction cache is cleared */
                env->mmu.icache_valid = false;
                tlb_flush_page(env_cpu(env), env->mmu.icache_vpage);
            }
        } else if (m68k_feature(env, M68K_FEATURE_M68040)) {
            env->cacr = val & 0x80008000;
        } else if (m68k_feature(env, M68K_FEATURE_M68060)) {
            env->cacr = val & 0xf8e0e000;
        } else {
            break;
        }
        m68k_switch_sp(env);
        return;
    /* MC680[46]0 */
    case M68K_CR_TC:
        if (m68k_feature(env, M68K_FEATURE_M68040)
         || m68k_feature(env, M68K_FEATURE_M68060)) {
            /*
             * QEMU's TLB caches the result of translation (including the
             * untranslated MMU-off case), so it must not survive a change
             * of the translation setup. E.g. A/UX jumps back into the Mac
             * ROM with its own mappings cached.
             */
            if (env->mmu.tcr != val) {
                tlb_flush(env_cpu(env));
            }
            env->mmu.tcr = val;
            return;
        }
        break;
    /* MC68040 */
    case M68K_CR_MMUSR:
        if (m68k_feature(env, M68K_FEATURE_M68040)) {
            env->mmu.mmusr = val;
            return;
        }
        break;
    /* MC680[46]0 */
    case M68K_CR_SRP:
        if (m68k_feature(env, M68K_FEATURE_M68040)
         || m68k_feature(env, M68K_FEATURE_M68060)) {
            env->mmu.srp = val;
            return;
        }
        break;
    /* MC680[46]0 */
    case M68K_CR_URP:
        if (m68k_feature(env, M68K_FEATURE_M68040)
         || m68k_feature(env, M68K_FEATURE_M68060)) {
            env->mmu.urp = val;
            return;
        }
        break;
    /* MC680[12346]0 */
    case M68K_CR_USP:
        env->sp[M68K_USP] = val;
        return;
    /* MC680[234]0 */
    case M68K_CR_MSP:
        if (m68k_feature(env, M68K_FEATURE_M68020)
         || m68k_feature(env, M68K_FEATURE_M68030)
         || m68k_feature(env, M68K_FEATURE_M68040)) {
            env->sp[M68K_SSP] = val;
            return;
        }
        break;
    /* MC680[234]0 */
    case M68K_CR_ISP:
        if (m68k_feature(env, M68K_FEATURE_M68020)
         || m68k_feature(env, M68K_FEATURE_M68030)
         || m68k_feature(env, M68K_FEATURE_M68040)) {
            env->sp[M68K_ISP] = val;
            return;
        }
        break;
    /* MC68040/MC68LC040 */
    case M68K_CR_ITT0: /* MC68EC040 only: M68K_CR_IACR0 */
        if (m68k_feature(env, M68K_FEATURE_M68040)) {
            if (env->mmu.ttr[M68K_ITTR0] != val) {
                tlb_flush(env_cpu(env));   /* see M68K_CR_TC */
            }
            env->mmu.ttr[M68K_ITTR0] = val;
            return;
        }
        break;
    /* MC68040/MC68LC040 */
    case M68K_CR_ITT1: /* MC68EC040 only: M68K_CR_IACR1 */
        if (m68k_feature(env, M68K_FEATURE_M68040)) {
            if (env->mmu.ttr[M68K_ITTR1] != val) {
                tlb_flush(env_cpu(env));   /* see M68K_CR_TC */
            }
            env->mmu.ttr[M68K_ITTR1] = val;
            return;
        }
        break;
    /* MC68040/MC68LC040 */
    case M68K_CR_DTT0: /* MC68EC040 only: M68K_CR_DACR0 */
        if (m68k_feature(env, M68K_FEATURE_M68040)) {
            if (env->mmu.ttr[M68K_DTTR0] != val) {
                tlb_flush(env_cpu(env));   /* see M68K_CR_TC */
            }
            env->mmu.ttr[M68K_DTTR0] = val;
            return;
        }
        break;
    /* MC68040/MC68LC040 */
    case M68K_CR_DTT1: /* MC68EC040 only: M68K_CR_DACR1 */
        if (m68k_feature(env, M68K_FEATURE_M68040)) {
            if (env->mmu.ttr[M68K_DTTR1] != val) {
                tlb_flush(env_cpu(env));   /* see M68K_CR_TC */
            }
            env->mmu.ttr[M68K_DTTR1] = val;
            return;
        }
        break;
    case M68K_CR_CAAR:
        /* the caches are not modelled: just keep the value */
        if (m68k_feature(env, M68K_FEATURE_M68020) ||
            m68k_feature(env, M68K_FEATURE_M68030)) {
            env->caar = val;
            return;
        }
        break;
    /* Unimplemented Registers */
    case M68K_CR_PCR:
    case M68K_CR_BUSCR:
        cpu_abort(env_cpu(env),
                  "Unimplemented control register write 0x%x = 0x%x\n",
                  reg, val);
    }

    /* Invalid control registers will generate an exception. */
    raise_exception_ra(env, EXCP_ILLEGAL, 0);
}

uint32_t HELPER(m68k_movec_from)(CPUM68KState *env, uint32_t reg)
{
    switch (reg) {
    /* MC680[12346]0 */
    case M68K_CR_SFC:
        return env->sfc;
    /* MC680[12346]0 */
    case M68K_CR_DFC:
        return env->dfc;
    /* MC680[12346]0 */
    case M68K_CR_VBR:
        return env->vbr;
    /* MC680[2346]0 */
    case M68K_CR_CACR:
        if (m68k_feature(env, M68K_FEATURE_M68020)
         || m68k_feature(env, M68K_FEATURE_M68030)
         || m68k_feature(env, M68K_FEATURE_M68040)
         || m68k_feature(env, M68K_FEATURE_M68060)) {
            return env->cacr;
        }
        break;
    /* MC680[46]0 */
    case M68K_CR_TC:
        if (m68k_feature(env, M68K_FEATURE_M68040)
         || m68k_feature(env, M68K_FEATURE_M68060)) {
            return env->mmu.tcr;
        }
        break;
    /* MC68040 */
    case M68K_CR_MMUSR:
        if (m68k_feature(env, M68K_FEATURE_M68040)) {
            return env->mmu.mmusr;
        }
        break;
    /* MC680[46]0 */
    case M68K_CR_SRP:
        if (m68k_feature(env, M68K_FEATURE_M68040)
         || m68k_feature(env, M68K_FEATURE_M68060)) {
            return env->mmu.srp;
        }
        break;
    /* MC68040/MC68LC040 */
    case M68K_CR_URP:
        if (m68k_feature(env, M68K_FEATURE_M68040)
         || m68k_feature(env, M68K_FEATURE_M68060)) {
            return env->mmu.urp;
        }
        break;
    /* MC680[46]0 */
    case M68K_CR_USP:
        return env->sp[M68K_USP];
    /* MC680[234]0 */
    case M68K_CR_MSP:
        if (m68k_feature(env, M68K_FEATURE_M68020)
         || m68k_feature(env, M68K_FEATURE_M68030)
         || m68k_feature(env, M68K_FEATURE_M68040)) {
            return env->sp[M68K_SSP];
        }
        break;
    /* MC680[234]0 */
    case M68K_CR_ISP:
        if (m68k_feature(env, M68K_FEATURE_M68020)
         || m68k_feature(env, M68K_FEATURE_M68030)
         || m68k_feature(env, M68K_FEATURE_M68040)) {
            return env->sp[M68K_ISP];
        }
        break;
    /* MC68040/MC68LC040 */
    case M68K_CR_ITT0: /* MC68EC040 only: M68K_CR_IACR0 */
        if (m68k_feature(env, M68K_FEATURE_M68040)) {
            return env->mmu.ttr[M68K_ITTR0];
        }
        break;
    /* MC68040/MC68LC040 */
    case M68K_CR_ITT1: /* MC68EC040 only: M68K_CR_IACR1 */
        if (m68k_feature(env, M68K_FEATURE_M68040)) {
            return env->mmu.ttr[M68K_ITTR1];
        }
        break;
    /* MC68040/MC68LC040 */
    case M68K_CR_DTT0: /* MC68EC040 only: M68K_CR_DACR0 */
        if (m68k_feature(env, M68K_FEATURE_M68040)) {
            return env->mmu.ttr[M68K_DTTR0];
        }
        break;
    /* MC68040/MC68LC040 */
    case M68K_CR_DTT1: /* MC68EC040 only: M68K_CR_DACR1 */
        if (m68k_feature(env, M68K_FEATURE_M68040)) {
            return env->mmu.ttr[M68K_DTTR1];
        }
        break;
    case M68K_CR_CAAR:
        if (m68k_feature(env, M68K_FEATURE_M68020) ||
            m68k_feature(env, M68K_FEATURE_M68030)) {
            return env->caar;
        }
        break;
    /* Unimplemented Registers */
    case M68K_CR_PCR:
    case M68K_CR_BUSCR:
        cpu_abort(env_cpu(env), "Unimplemented control register read 0x%x\n",
                  reg);
    }

    /* Invalid control registers will generate an exception. */
    raise_exception_ra(env, EXCP_ILLEGAL, 0);

    return 0;
}

void HELPER(set_macsr)(CPUM68KState *env, uint32_t val)
{
    uint32_t acc;
    int8_t exthigh;
    uint8_t extlow;
    uint64_t regval;
    int i;
    if ((env->macsr ^ val) & (MACSR_FI | MACSR_SU)) {
        for (i = 0; i < 4; i++) {
            regval = env->macc[i];
            exthigh = regval >> 40;
            if (env->macsr & MACSR_FI) {
                acc = regval >> 8;
                extlow = regval;
            } else {
                acc = regval;
                extlow = regval >> 32;
            }
            if (env->macsr & MACSR_FI) {
                regval = (((uint64_t)acc) << 8) | extlow;
                regval |= ((int64_t)exthigh) << 40;
            } else if (env->macsr & MACSR_SU) {
                regval = acc | (((int64_t)extlow) << 32);
                regval |= ((int64_t)exthigh) << 40;
            } else {
                regval = acc | (((uint64_t)extlow) << 32);
                regval |= ((uint64_t)(uint8_t)exthigh) << 40;
            }
            env->macc[i] = regval;
        }
    }
    env->macsr = val;
}

void m68k_switch_sp(CPUM68KState *env)
{
    int new_sp;

    env->sp[env->current_sp] = env->aregs[7];
    if (m68k_feature(env, M68K_FEATURE_M68K)) {
        if (env->sr & SR_S) {
            /* SR:Master-Mode bit unimplemented then ISP is not available */
            if (!m68k_feature(env, M68K_FEATURE_MSP) || env->sr & SR_M) {
                new_sp = M68K_SSP;
            } else {
                new_sp = M68K_ISP;
            }
        } else {
            new_sp = M68K_USP;
        }
    } else {
        new_sp = (env->sr & SR_S && env->cacr & M68K_CACR_EUSP)
                 ? M68K_SSP : M68K_USP;
    }
    env->aregs[7] = env->sp[new_sp];
    env->current_sp = new_sp;
}

#if !defined(CONFIG_USER_ONLY)
/* MMU: 68040 only */

static void m68030_dump_mmu(CPUM68KState *env);

static void print_address_zone(uint32_t logical, uint32_t physical,
                               uint32_t size, int attr)
{
    qemu_printf("%08x - %08x -> %08x - %08x %c ",
                logical, logical + size - 1,
                physical, physical + size - 1,
                attr & 4 ? 'W' : '-');
    size >>= 10;
    if (size < 1024) {
        qemu_printf("(%d KiB)\n", size);
    } else {
        size >>= 10;
        if (size < 1024) {
            qemu_printf("(%d MiB)\n", size);
        } else {
            size >>= 10;
            qemu_printf("(%d GiB)\n", size);
        }
    }
}

static void dump_address_map(CPUM68KState *env, uint32_t root_pointer)
{
    int tic_size, tic_shift;
    uint32_t tib_mask;
    uint32_t tia, tib, tic;
    uint32_t logical = 0xffffffff, physical = 0xffffffff;
    uint32_t first_logical = 0xffffffff, first_physical = 0xffffffff;
    uint32_t last_logical, last_physical;
    int32_t size;
    int last_attr = -1, attr = -1;
    CPUState *cs = env_cpu(env);
    MemTxResult txres;

    if (env->mmu.tcr & M68K_TCR_PAGE_8K) {
        /* 8k page */
        tic_size = 32;
        tic_shift = 13;
        tib_mask = M68K_8K_PAGE_MASK;
    } else {
        /* 4k page */
        tic_size = 64;
        tic_shift = 12;
        tib_mask = M68K_4K_PAGE_MASK;
    }
    for (unsigned i = 0; i < M68K_ROOT_POINTER_ENTRIES; i++) {
        tia = address_space_ldl(cs->as, M68K_POINTER_BASE(root_pointer) + i * 4,
                                MEMTXATTRS_UNSPECIFIED, &txres);
        if (txres != MEMTX_OK || !M68K_UDT_VALID(tia)) {
            continue;
        }
        for (unsigned j = 0; j < M68K_ROOT_POINTER_ENTRIES; j++) {
            tib = address_space_ldl(cs->as, M68K_POINTER_BASE(tia) + j * 4,
                                    MEMTXATTRS_UNSPECIFIED, &txres);
            if (txres != MEMTX_OK || !M68K_UDT_VALID(tib)) {
                continue;
            }
            for (unsigned k = 0; k < tic_size; k++) {
                tic = address_space_ldl(cs->as, (tib & tib_mask) + k * 4,
                                        MEMTXATTRS_UNSPECIFIED, &txres);
                if (txres != MEMTX_OK || !M68K_PDT_VALID(tic)) {
                    continue;
                }
                if (M68K_PDT_INDIRECT(tic)) {
                    tic = address_space_ldl(cs->as, M68K_INDIRECT_POINTER(tic),
                                            MEMTXATTRS_UNSPECIFIED, &txres);
                    if (txres != MEMTX_OK) {
                        continue;
                    }
                }

                last_logical = logical;
                logical = (i << M68K_TTS_ROOT_SHIFT) |
                          (j << M68K_TTS_POINTER_SHIFT) |
                          (k << tic_shift);

                last_physical = physical;
                physical = tic & ~((1 << tic_shift) - 1);

                last_attr = attr;
                attr = tic & ((1 << tic_shift) - 1);

                if ((logical != (last_logical + (1 << tic_shift))) ||
                    (physical != (last_physical + (1 << tic_shift))) ||
                    (attr & 4) != (last_attr & 4)) {

                    if (first_logical != 0xffffffff) {
                        size = last_logical + (1 << tic_shift) -
                               first_logical;
                        print_address_zone(first_logical,
                                           first_physical, size, last_attr);
                    }
                    first_logical = logical;
                    first_physical = physical;
                }
            }
        }
    }
    if (first_logical != logical || (attr & 4) != (last_attr & 4)) {
        size = logical + (1 << tic_shift) - first_logical;
        print_address_zone(first_logical, first_physical, size, last_attr);
    }
}

#define DUMP_CACHEFLAGS(a) \
    switch (a & M68K_DESC_CACHEMODE) { \
    case M68K_DESC_CM_WRTHRU: /* cacheable, write-through */ \
        qemu_printf("T"); \
        break; \
    case M68K_DESC_CM_COPYBK: /* cacheable, copyback */ \
        qemu_printf("C"); \
        break; \
    case M68K_DESC_CM_SERIAL: /* noncachable, serialized */ \
        qemu_printf("S"); \
        break; \
    case M68K_DESC_CM_NCACHE: /* noncachable */ \
        qemu_printf("N"); \
        break; \
    }

static void dump_ttr(uint32_t ttr)
{
    if ((ttr & M68K_TTR_ENABLED) == 0) {
        qemu_printf("disabled\n");
        return;
    }
    qemu_printf("Base: 0x%08x Mask: 0x%08x Control: ",
                ttr & M68K_TTR_ADDR_BASE,
                (ttr & M68K_TTR_ADDR_MASK) << M68K_TTR_ADDR_MASK_SHIFT);
    switch (ttr & M68K_TTR_SFIELD) {
    case M68K_TTR_SFIELD_USER:
        qemu_printf("U");
        break;
    case M68K_TTR_SFIELD_SUPER:
        qemu_printf("S");
        break;
    default:
        qemu_printf("*");
        break;
    }
    DUMP_CACHEFLAGS(ttr);
    if (ttr & M68K_DESC_WRITEPROT) {
        qemu_printf("R");
    } else {
        qemu_printf("W");
    }
    qemu_printf(" U: %d\n", (ttr & M68K_DESC_USERATTR) >>
                               M68K_DESC_USERATTR_SHIFT);
}

void dump_mmu(CPUM68KState *env)
{
    if (m68k_feature(env, M68K_FEATURE_M68030)) {
        m68030_dump_mmu(env);
        return;
    }
    if ((env->mmu.tcr & M68K_TCR_ENABLED) == 0) {
        qemu_printf("Translation disabled\n");
        return;
    }
    qemu_printf("Page Size: ");
    if (env->mmu.tcr & M68K_TCR_PAGE_8K) {
        qemu_printf("8kB\n");
    } else {
        qemu_printf("4kB\n");
    }

    qemu_printf("MMUSR: ");
    if (env->mmu.mmusr & M68K_MMU_B_040) {
        qemu_printf("BUS ERROR\n");
    } else {
        qemu_printf("Phy=%08x Flags: ", env->mmu.mmusr & 0xfffff000);
        /* flags found on the page descriptor */
        if (env->mmu.mmusr & M68K_MMU_G_040) {
            qemu_printf("G"); /* Global */
        } else {
            qemu_printf(".");
        }
        if (env->mmu.mmusr & M68K_MMU_S_040) {
            qemu_printf("S"); /* Supervisor */
        } else {
            qemu_printf(".");
        }
        if (env->mmu.mmusr & M68K_MMU_M_040) {
            qemu_printf("M"); /* Modified */
        } else {
            qemu_printf(".");
        }
        if (env->mmu.mmusr & M68K_MMU_WP_040) {
            qemu_printf("W"); /* Write protect */
        } else {
            qemu_printf(".");
        }
        if (env->mmu.mmusr & M68K_MMU_T_040) {
            qemu_printf("T"); /* Transparent */
        } else {
            qemu_printf(".");
        }
        if (env->mmu.mmusr & M68K_MMU_R_040) {
            qemu_printf("R"); /* Resident */
        } else {
            qemu_printf(".");
        }
        qemu_printf(" Cache: ");
        DUMP_CACHEFLAGS(env->mmu.mmusr);
        qemu_printf(" U: %d\n", (env->mmu.mmusr >> 8) & 3);
        qemu_printf("\n");
    }

    qemu_printf("ITTR0: ");
    dump_ttr(env->mmu.ttr[M68K_ITTR0]);
    qemu_printf("ITTR1: ");
    dump_ttr(env->mmu.ttr[M68K_ITTR1]);
    qemu_printf("DTTR0: ");
    dump_ttr(env->mmu.ttr[M68K_DTTR0]);
    qemu_printf("DTTR1: ");
    dump_ttr(env->mmu.ttr[M68K_DTTR1]);

    qemu_printf("SRP: 0x%08x\n", env->mmu.srp);
    dump_address_map(env, env->mmu.srp);

    qemu_printf("URP: 0x%08x\n", env->mmu.urp);
    dump_address_map(env, env->mmu.urp);
}

static int check_TTR(uint32_t ttr, int *prot, target_ulong addr,
                     int access_type)
{
    uint32_t base, mask;

    /* check if transparent translation is enabled */
    if ((ttr & M68K_TTR_ENABLED) == 0) {
        return 0;
    }

    /* check mode access */
    switch (ttr & M68K_TTR_SFIELD) {
    case M68K_TTR_SFIELD_USER:
        /* match only if user */
        if ((access_type & ACCESS_SUPER) != 0) {
            return 0;
        }
        break;
    case M68K_TTR_SFIELD_SUPER:
        /* match only if supervisor */
        if ((access_type & ACCESS_SUPER) == 0) {
            return 0;
        }
        break;
    default:
        /* all other values disable mode matching (FC2) */
        break;
    }

    /* check address matching */

    base = ttr & M68K_TTR_ADDR_BASE;
    mask = (ttr & M68K_TTR_ADDR_MASK) ^ M68K_TTR_ADDR_MASK;
    mask <<= M68K_TTR_ADDR_MASK_SHIFT;

    if ((addr & mask) != (base & mask)) {
        return 0;
    }

    *prot = PAGE_READ | PAGE_EXEC;
    if ((ttr & M68K_DESC_WRITEPROT) == 0) {
        *prot |= PAGE_WRITE;
    }

    return 1;
}

static int get_physical_address(CPUM68KState *env, hwaddr *physical,
                                int *prot, target_ulong address,
                                int access_type, target_ulong *page_size)
{
    CPUState *cs = env_cpu(env);
    uint32_t entry;
    uint32_t next;
    target_ulong page_mask;
    bool debug = access_type & ACCESS_DEBUG;
    int page_bits;
    int i;
    MemTxResult txres;

    /* Transparent Translation (physical = logical) */
    for (i = 0; i < M68K_MAX_TTR; i++) {
        if (check_TTR(env->mmu.TTR(access_type, i),
                      prot, address, access_type)) {
            if (access_type & ACCESS_PTEST) {
                /* Transparent Translation Register bit */
                env->mmu.mmusr = M68K_MMU_T_040 | M68K_MMU_R_040;
            }
            *physical = address;
            *page_size = TARGET_PAGE_SIZE;
            return 0;
        }
    }

    /* Page Table Root Pointer */
    *prot = PAGE_READ | PAGE_WRITE;
    if (access_type & ACCESS_CODE) {
        *prot |= PAGE_EXEC;
    }
    if (access_type & ACCESS_SUPER) {
        next = env->mmu.srp;
    } else {
        next = env->mmu.urp;
    }

    /* Root Index */
    entry = M68K_POINTER_BASE(next) | M68K_ROOT_INDEX(address);

    next = address_space_ldl(cs->as, entry, MEMTXATTRS_UNSPECIFIED, &txres);
    if (txres != MEMTX_OK) {
        goto txfail;
    }
    if (!M68K_UDT_VALID(next)) {
        return -1;
    }
    if (!(next & M68K_DESC_USED) && !debug) {
        address_space_stl(cs->as, entry, next | M68K_DESC_USED,
                          MEMTXATTRS_UNSPECIFIED, &txres);
        if (txres != MEMTX_OK) {
            goto txfail;
        }
    }
    if (next & M68K_DESC_WRITEPROT) {
        if (access_type & ACCESS_PTEST) {
            env->mmu.mmusr |= M68K_MMU_WP_040;
        }
        *prot &= ~PAGE_WRITE;
        if (access_type & ACCESS_STORE) {
            return -1;
        }
    }

    /* Pointer Index */
    entry = M68K_POINTER_BASE(next) | M68K_POINTER_INDEX(address);

    next = address_space_ldl(cs->as, entry, MEMTXATTRS_UNSPECIFIED, &txres);
    if (txres != MEMTX_OK) {
        goto txfail;
    }
    if (!M68K_UDT_VALID(next)) {
        return -1;
    }
    if (!(next & M68K_DESC_USED) && !debug) {
        address_space_stl(cs->as, entry, next | M68K_DESC_USED,
                          MEMTXATTRS_UNSPECIFIED, &txres);
        if (txres != MEMTX_OK) {
            goto txfail;
        }
    }
    if (next & M68K_DESC_WRITEPROT) {
        if (access_type & ACCESS_PTEST) {
            env->mmu.mmusr |= M68K_MMU_WP_040;
        }
        *prot &= ~PAGE_WRITE;
        if (access_type & ACCESS_STORE) {
            return -1;
        }
    }

    /* Page Index */
    if (env->mmu.tcr & M68K_TCR_PAGE_8K) {
        entry = M68K_8K_PAGE_BASE(next) | M68K_8K_PAGE_INDEX(address);
    } else {
        entry = M68K_4K_PAGE_BASE(next) | M68K_4K_PAGE_INDEX(address);
    }

    next = address_space_ldl(cs->as, entry, MEMTXATTRS_UNSPECIFIED, &txres);
    if (txres != MEMTX_OK) {
        goto txfail;
    }

    if (!M68K_PDT_VALID(next)) {
        return -1;
    }
    if (M68K_PDT_INDIRECT(next)) {
        next = address_space_ldl(cs->as, M68K_INDIRECT_POINTER(next),
                                 MEMTXATTRS_UNSPECIFIED, &txres);
        if (txres != MEMTX_OK) {
            goto txfail;
        }
    }
    if (access_type & ACCESS_STORE) {
        if (next & M68K_DESC_WRITEPROT) {
            if (!(next & M68K_DESC_USED) && !debug) {
                address_space_stl(cs->as, entry, next | M68K_DESC_USED,
                                  MEMTXATTRS_UNSPECIFIED, &txres);
                if (txres != MEMTX_OK) {
                    goto txfail;
                }
            }
        } else if ((next & (M68K_DESC_MODIFIED | M68K_DESC_USED)) !=
                           (M68K_DESC_MODIFIED | M68K_DESC_USED) && !debug) {
            address_space_stl(cs->as, entry,
                              next | (M68K_DESC_MODIFIED | M68K_DESC_USED),
                              MEMTXATTRS_UNSPECIFIED, &txres);
            if (txres != MEMTX_OK) {
                goto txfail;
            }
        }
    } else {
        if (!(next & M68K_DESC_USED) && !debug) {
            address_space_stl(cs->as, entry, next | M68K_DESC_USED,
                              MEMTXATTRS_UNSPECIFIED, &txres);
            if (txres != MEMTX_OK) {
                goto txfail;
            }
        }
    }

    if (env->mmu.tcr & M68K_TCR_PAGE_8K) {
        page_bits = 13;
    } else {
        page_bits = 12;
    }
    *page_size = 1 << page_bits;
    page_mask = ~(*page_size - 1);
    *physical = (next & page_mask) + (address & (*page_size - 1));

    if (access_type & ACCESS_PTEST) {
        env->mmu.mmusr |= next & M68K_MMU_SR_MASK_040;
        env->mmu.mmusr |= *physical & 0xfffff000;
        env->mmu.mmusr |= M68K_MMU_R_040;
    }

    if (next & M68K_DESC_WRITEPROT) {
        *prot &= ~PAGE_WRITE;
        if (access_type & ACCESS_STORE) {
            return -1;
        }
    }
    if (next & M68K_DESC_SUPERONLY) {
        if ((access_type & ACCESS_SUPER) == 0) {
            return -1;
        }
    }

    return 0;

txfail:
    /*
     * A page table load/store failed. TODO: we should really raise a
     * suitable guest fault here if this is not a debug access.
     * For now just return that the translation failed.
     */
    return -1;
}

/* MMU: 68030 on-chip PMMU */

/* 68030 descriptor types */
#define M68030_DT_INVALID   0
#define M68030_DT_PAGE      1
#define M68030_DT_SHORT     2
#define M68030_DT_LONG      3

typedef struct M68030Walk {
    uint32_t physical;
    uint32_t desc_addr;     /* address of the last descriptor fetched */
    bool resolved;          /* a page (or early termination) was found */
} M68030Walk;

static bool m68030_tt_match(CPUM68KState *env, uint32_t addr, int fc,
                            bool write)
{
    int i;

    for (i = 0; i < 2; i++) {
        uint32_t tt = env->mmu.tt030[i];

        if (!(tt & M68K_TT030_E)) {
            continue;
        }
        /* logical address base/mask against A31-A24 */
        if (((addr >> 24) ^ (tt >> 24)) & ~(tt >> 16) & 0xff) {
            continue;
        }
        /* function code base/mask */
        if ((fc ^ (tt >> 4)) & ~tt & 7) {
            continue;
        }
        /* R/W selects reads (1) or writes (0) unless RWM is set */
        if (!(tt & M68K_TT030_RWM) && !(tt & M68K_TT030_RW) != write) {
            continue;
        }
        return true;
    }
    return false;
}

static bool m68030_tc_valid(uint32_t tc)
{
    int ps = extract32(tc, 20, 4);
    int bits = ps + extract32(tc, 16, 4);
    int i;

    if (ps < 8) {
        return false;
    }
    for (i = 12; i >= 0; i -= 4) {
        int ti = extract32(tc, i, 4);

        if (!ti) {
            break;
        }
        bits += ti;
    }
    return bits == 32;
}

static bool m68030_load_desc(CPUState *cs, uint32_t addr, bool long_desc,
                             uint32_t *d0, uint32_t *d1)
{
    MemTxResult txres;

    *d0 = address_space_ldl(cs->as, addr, MEMTXATTRS_UNSPECIFIED, &txres);
    if (txres != MEMTX_OK) {
        return false;
    }
    if (!long_desc) {
        *d1 = *d0;
        return true;
    }
    *d1 = address_space_ldl(cs->as, addr + 4, MEMTXATTRS_UNSPECIFIED, &txres);
    return txres == MEMTX_OK;
}

/*
 * Search the 68030 translation tables for @addr in function code @fc.
 * @levels is the maximum number of descriptors fetched (PTEST #level),
 * @update sets the U and M bits of the descriptors.
 * Returns the MMUSR bits describing the search (W and S are accumulated
 * over all the levels, N is the number of descriptors fetched).
 */
static uint16_t m68030_walk(CPUM68KState *env, uint32_t addr, int fc,
                            bool write, int levels, bool update,
                            M68030Walk *w)
{
    CPUState *cs = env_cpu(env);
    uint32_t tc = env->mmu.tc030;
    int ps = extract32(tc, 20, 4);
    bool super = fc & 4;
    int ti[4];
    int nti = 0;
    uint64_t root;
    uint32_t table, limit_word, d0, d1;
    bool has_limit;
    int dt, rem, step, n = 0;
    uint16_t sr = 0;
    int i;

    for (i = 12; i >= 0; i -= 4) {
        ti[nti] = extract32(tc, i, 4);
        if (!ti[nti]) {
            break;
        }
        nti++;
    }

    w->resolved = false;
    w->desc_addr = 0;
    w->physical = addr;

    /* the root pointer is a long format table descriptor */
    root = ((tc & M68K_TC030_SRE) && super) ? env->mmu.srp030
                                            : env->mmu.crp030;
    limit_word = root >> 32;
    has_limit = true;
    dt = limit_word & 3;
    table = root & ~0xfULL;
    /* the bits of the address that remain to be translated */
    rem = 32 - extract32(tc, 16, 4);
    /* step -1 is the function code lookup level */
    step = (tc & M68K_TC030_FCL) ? -1 : 0;

    for (;;) {
        bool have_next = step < nti;
        bool long_desc, last;
        int width = 0, index = 0, ndt;
        uint32_t daddr;

        if (dt == M68030_DT_INVALID) {
            sr |= M68K_MMUSR030_I;
            break;
        }
        if (have_next) {
            if (step < 0) {
                index = fc;
            } else {
                width = ti[step];
                if (width > rem) {
                    /* inconsistent TC */
                    sr |= M68K_MMUSR030_I;
                    break;
                }
                index = extract32(addr, rem - width, width);
            }
            if (has_limit) {
                int limit = extract32(limit_word, 16, 15);

                if ((limit_word & 0x80000000) ? index < limit
                                              : index > limit) {
                    sr |= M68K_MMUSR030_L | M68K_MMUSR030_I;
                    break;
                }
            }
        }
        if (dt == M68030_DT_PAGE) {
            /* page, or early termination: the rest is the page offset */
            uint32_t offset_mask = rem >= 32 ? ~0u : (1u << rem) - 1;

            w->physical = (table & ~((1u << ps) - 1)) + (addr & offset_mask);
            w->resolved = true;
            break;
        }
        if (!have_next || n >= levels) {
            /* PTEST level reached (tables at the last level are indirect) */
            break;
        }

        long_desc = dt == M68030_DT_LONG;
        daddr = table + (index << (long_desc ? 3 : 2));
        n++;
        if (!m68030_load_desc(cs, daddr, long_desc, &d0, &d1)) {
            sr |= M68K_MMUSR030_B | M68K_MMUSR030_I;
            w->desc_addr = daddr;
            break;
        }
        rem -= width;
        step++;
        last = step == nti;
        ndt = d0 & 3;
        if (last && (ndt == M68030_DT_SHORT || ndt == M68030_DT_LONG)) {
            /* indirect descriptor: points to the page descriptor */
            daddr = d1 & ~3;
            long_desc = ndt == M68030_DT_LONG;
            n++;
            if (!m68030_load_desc(cs, daddr, long_desc, &d0, &d1)) {
                sr |= M68K_MMUSR030_B | M68K_MMUSR030_I;
                w->desc_addr = daddr;
                break;
            }
            ndt = d0 & 3;
            if (ndt != M68030_DT_PAGE) {
                ndt = M68030_DT_INVALID;
            }
        }
        w->desc_addr = daddr;
        if (ndt == M68030_DT_INVALID) {
            sr |= M68K_MMUSR030_I;
            break;
        }
        if (d0 & M68K_DESC_WRITEPROT) {
            sr |= M68K_MMUSR030_W;
        }
        if (long_desc && (d0 & M68K_DESC_SUPERONLY)) {
            sr |= M68K_MMUSR030_S;
        }
        if (update) {
            uint32_t nd0 = d0 | M68K_DESC_USED;

            if (ndt == M68030_DT_PAGE && write &&
                !(sr & M68K_MMUSR030_W) &&
                (super || !(sr & M68K_MMUSR030_S))) {
                nd0 |= M68K_DESC_MODIFIED;
            }
            if (nd0 != d0) {
                address_space_stl(cs->as, daddr, nd0,
                                  MEMTXATTRS_UNSPECIFIED, NULL);
                d0 = nd0;
            }
        }
        if (ndt == M68030_DT_PAGE) {
            if (d0 & M68K_DESC_MODIFIED) {
                sr |= M68K_MMUSR030_M;
            }
            table = d1 & ~0xffu;
            /* early termination long page descriptors have a limit */
            has_limit = long_desc && !last;
        } else {
            table = d1 & ~0xfu;
            has_limit = long_desc;
        }
        limit_word = d0;
        dt = ndt;
    }

    return sr | MIN(n, 7);
}

/* is the access allowed by the result of a table search? */
static bool m68030_access_ok(uint16_t sr, const M68030Walk *w, bool super,
                             bool write)
{
    if (!w->resolved || (sr & M68K_MMUSR030_I)) {
        return false;
    }
    if (!super && (sr & M68K_MMUSR030_S)) {
        return false;
    }
    return !(write && (sr & M68K_MMUSR030_W));
}

static bool m68030_tlb_fill(CPUState *cs, vaddr address, int size,
                            MMUAccessType access_type, int mmu_idx,
                            bool probe, uintptr_t retaddr)
{
    CPUM68KState *env = cpu_env(cs);
    bool super = mmu_idx != MMU_USER_IDX;
    bool write = access_type == MMU_DATA_STORE;
    bool fetch = access_type == MMU_INST_FETCH;
    bool split = env->mmu.tc030 & M68K_TC030_FCL;
    int dfc = super ? 5 : 1;
    int pfc = super ? 6 : 2;
    M68030Walk w;
    uint16_t sr;
    int prot, ps;
    vaddr vpage;
    hwaddr ppage;

    if (m68030_tt_match(env, address, fetch ? pfc : dfc, write)) {
        /* transparent: allow the other kinds of access the TTs match */
        prot = 0;
        if (m68030_tt_match(env, address, dfc, false)) {
            prot |= PAGE_READ;
        }
        if (m68030_tt_match(env, address, dfc, true)) {
            prot |= PAGE_WRITE;
        }
        if (m68030_tt_match(env, address, pfc, false)) {
            prot |= PAGE_EXEC;
        }
        tlb_set_page(cs, address & TARGET_PAGE_MASK,
                     address & TARGET_PAGE_MASK, prot, mmu_idx,
                     TARGET_PAGE_SIZE);
        return true;
    }

    sr = m68030_walk(env, address, fetch ? pfc : dfc, write, 16, true, &w);
    if (!m68030_access_ok(sr, &w, super, write)) {
        if (fetch && env->mmu.icache_valid &&
            (address & TARGET_PAGE_MASK) == env->mmu.icache_vpage) {
            /* still in the instruction cache: see m68030_note_code_page */
            tlb_set_page(cs, env->mmu.icache_vpage, env->mmu.icache_ppage,
                         PAGE_EXEC, mmu_idx, TARGET_PAGE_SIZE);
            return true;
        }
        if (probe) {
            return false;
        }
        /* the 68030 reports MMU faults as bus errors */
        m68020_set_fault(env, address, size, access_type, mmu_idx);
        cs->exception_index = EXCP_ACCESS;
        cpu_loop_exit_restore(cs, retaddr);
    }

    /*
     * Only grant the other kinds of access when they translate the same
     * way: data and program accesses differ with function code lookup,
     * or when a transparent translation register matches one of them.
     * Writes are granted once the page is modified so that M gets set.
     */
    if (fetch) {
        prot = PAGE_EXEC;
    } else {
        prot = write ? PAGE_READ | PAGE_WRITE : PAGE_READ;
    }
    if (!split) {
        if (!m68030_tt_match(env, address, dfc, false)) {
            prot |= PAGE_READ;
        }
        if (!m68030_tt_match(env, address, pfc, false)) {
            prot |= PAGE_EXEC;
        }
    }
    if ((!fetch || !split) && (sr & M68K_MMUSR030_M) &&
        !(sr & M68K_MMUSR030_W) &&
        !m68030_tt_match(env, address, dfc, true)) {
        prot |= PAGE_WRITE;
    }

    ps = extract32(env->mmu.tc030, 20, 4);
    if (ps >= TARGET_PAGE_BITS) {
        vpage = address & TARGET_PAGE_MASK;
        ppage = w.physical & TARGET_PAGE_MASK;
    } else {
        /*
         * The TLB does not cache pages smaller than TARGET_PAGE_SIZE, but
         * it still maps the access within a TARGET_PAGE_SIZE page, so the
         * low address bits must be the same.
         */
        if ((address ^ w.physical) & ~TARGET_PAGE_MASK) {
            qemu_log_mask(LOG_UNIMP, "68030 MMU: %d byte page at %08"
                          VADDR_PRIx " -> %08x is not supported\n",
                          1 << ps, address, w.physical);
        }
        vpage = address;
        ppage = w.physical;
    }
    tlb_set_page(cs, vpage, ppage, prot, mmu_idx, 1 << ps);
    return true;
}

static bool m68030_translate_for_debug(CPUM68KState *env, uint32_t addr,
                                       hwaddr *physical)
{
    int fc = (env->sr & SR_S) ? 5 : 1;
    M68030Walk w;
    uint16_t sr;

    if (m68030_tt_match(env, addr, fc, false)) {
        *physical = addr;
        return true;
    }
    sr = m68030_walk(env, addr, fc, false, 16, false, &w);
    if (!m68030_access_ok(sr, &w, fc & 4, false)) {
        return false;
    }
    *physical = w.physical;
    return true;
}

static void m68030_dump_map(CPUM68KState *env, int fc)
{
    uint32_t page = 1u << extract32(env->mmu.tc030, 20, 4);
    uint32_t addr = 0, first_logical = 0, first_physical = 0;
    uint32_t next_physical = 0;
    int first_attr = -1;

    do {
        M68030Walk w;
        uint16_t sr = m68030_walk(env, addr, fc, false, 16, false, &w);
        int attr = -1;

        if (m68030_access_ok(sr, &w, fc & 4, false)) {
            attr = (sr & M68K_MMUSR030_W) ? 4 : 0;
        }
        if (attr != first_attr ||
            (attr >= 0 && w.physical != next_physical)) {
            if (first_attr >= 0) {
                print_address_zone(first_logical, first_physical,
                                   addr - first_logical, first_attr);
            }
            first_logical = addr;
            first_physical = w.physical;
            first_attr = attr;
        }
        next_physical = w.physical + page;
        addr += page;
    } while (addr != 0);
    if (first_attr >= 0) {
        print_address_zone(first_logical, first_physical,
                           addr - first_logical, first_attr);
    }
}

static void m68030_dump_mmu(CPUM68KState *env)
{
    uint32_t tc = env->mmu.tc030;
    int i;

    qemu_printf("TC: %08x CRP: %016" PRIx64 " SRP: %016" PRIx64 "\n",
                tc, env->mmu.crp030, env->mmu.srp030);
    qemu_printf("TT0: %08x TT1: %08x MMUSR: %04x\n",
                env->mmu.tt030[0], env->mmu.tt030[1], env->mmu.mmusr030);
    for (i = 0; i < 2; i++) {
        uint32_t tt = env->mmu.tt030[i];

        if (tt & M68K_TT030_E) {
            qemu_printf("TT%d: %02x000000 mask %02xffffff FC %d mask %d %s\n",
                        i, tt >> 24, (tt >> 16) & 0xff, (tt >> 4) & 7,
                        tt & 7, tt & M68K_TT030_RWM ? "RW" :
                        tt & M68K_TT030_RW ? "R" : "W");
        }
    }
    if (!(tc & M68K_TC030_E)) {
        qemu_printf("Translation disabled\n");
        return;
    }
    qemu_printf("Page size: %d bytes, IS %d, TI %d/%d/%d/%d%s%s\n",
                1 << extract32(tc, 20, 4), extract32(tc, 16, 4),
                extract32(tc, 12, 4), extract32(tc, 8, 4),
                extract32(tc, 4, 4), extract32(tc, 0, 4),
                tc & M68K_TC030_SRE ? " SRE" : "",
                tc & M68K_TC030_FCL ? " FCL" : "");
    qemu_printf("Supervisor data:\n");
    m68030_dump_map(env, 5);
    if (tc & (M68K_TC030_SRE | M68K_TC030_FCL)) {
        qemu_printf("User data:\n");
        m68030_dump_map(env, 1);
    }
}

/* is the CPU's own MMU translating? */
static bool m68k_mmu_enabled(CPUM68KState *env)
{
    if (m68k_feature(env, M68K_FEATURE_M68030)) {
        return env->mmu.tc030 & M68K_TC030_E;
    }
    return env->mmu.tcr & M68K_TCR_ENABLED;
}

void m68k_cpu_set_addr_map(M68kCPU *cpu, M68kAddrMapFn fn, void *opaque)
{
    if (cpu->addr_map == fn && cpu->addr_map_opaque == opaque) {
        return;
    }
    cpu->addr_map = fn;
    cpu->addr_map_opaque = opaque;
    tlb_flush(CPU(cpu));
}

bool m68k_cpu_translate_for_debug(CPUState *cs, vaddr addr,
                                  TranslateForDebugResult *result)
{
    CPUM68KState *env = cpu_env(cs);
    hwaddr phys_addr;
    int prot;
    int access_type;
    target_ulong page_size;

    if (!m68k_mmu_enabled(env)) {
        /* MMU disabled */
        M68kCPU *cpu = M68K_CPU(cs);

        phys_addr = cpu->addr_map ?
            cpu->addr_map(cpu->addr_map_opaque, addr & TARGET_PAGE_MASK) +
            (addr & ~TARGET_PAGE_MASK) : addr;
        goto success;
    }

    if (m68k_feature(env, M68K_FEATURE_M68030)) {
        if (!m68030_translate_for_debug(env, addr, &phys_addr)) {
            return false;
        }
        goto success;
    }

    access_type = ACCESS_DATA | ACCESS_DEBUG;
    if (env->sr & SR_S) {
        access_type |= ACCESS_SUPER;
    }

    if (get_physical_address(env, &phys_addr, &prot,
                             addr, access_type, &page_size) != 0) {
        return false;
    }

success:
    *result = (TranslateForDebugResult) {
        .physaddr = phys_addr,
        .lg_page_size = TARGET_PAGE_BITS,
        .attrs.debug = 1,
    };
    return true;
}

/*
 * Notify CPU of a pending interrupt.  Prioritization and vectoring should
 * be handled by the interrupt controller.  Real hardware only requests
 * the vector when the interrupt is acknowledged by the CPU.  For
 * simplicity we calculate it when the interrupt is signalled.
 */
void m68k_set_irq_level(M68kCPU *cpu, int level, uint8_t vector)
{
    CPUState *cs = CPU(cpu);
    CPUM68KState *env = &cpu->env;

    env->pending_level = level;
    env->pending_vector = vector;
    if (level) {
        cpu_interrupt(cs, CPU_INTERRUPT_HARD);
    } else {
        cpu_reset_interrupt(cs, CPU_INTERRUPT_HARD);
    }
}

bool m68k_cpu_tlb_fill(CPUState *cs, vaddr address, int size,
                       MMUAccessType qemu_access_type, int mmu_idx,
                       bool probe, uintptr_t retaddr)
{
    CPUM68KState *env = cpu_env(cs);
    hwaddr physical;
    int prot;
    int access_type;
    int ret;
    target_ulong page_size;

    if (!m68k_mmu_enabled(env)) {
        /* MMU disabled: the board may translate (Macintosh II AMU) */
        M68kCPU *cpu = M68K_CPU(cs);
        vaddr page = address & TARGET_PAGE_MASK;

        tlb_set_page(cs, page,
                     cpu->addr_map ?
                         cpu->addr_map(cpu->addr_map_opaque, page) : page,
                     PAGE_READ | PAGE_WRITE | PAGE_EXEC,
                     mmu_idx, TARGET_PAGE_SIZE);
        return true;
    }

    if (m68k_feature(env, M68K_FEATURE_M68030)) {
        return m68030_tlb_fill(cs, address, size, qemu_access_type, mmu_idx,
                               probe, retaddr);
    }

    if (qemu_access_type == MMU_INST_FETCH) {
        access_type = ACCESS_CODE;
    } else {
        access_type = ACCESS_DATA;
        if (qemu_access_type == MMU_DATA_STORE) {
            access_type |= ACCESS_STORE;
        }
    }
    if (mmu_idx != MMU_USER_IDX) {
        access_type |= ACCESS_SUPER;
    }

    ret = get_physical_address(env, &physical, &prot,
                               address, access_type, &page_size);
    if (likely(ret == 0)) {
        tlb_set_page(cs, address & TARGET_PAGE_MASK,
                     physical & TARGET_PAGE_MASK, prot, mmu_idx, page_size);
        return true;
    }

    if (probe) {
        return false;
    }

    /* page fault */
    env->mmu.ssw = M68K_ATC_040;
    switch (size) {
    case 1:
        env->mmu.ssw |= M68K_BA_SIZE_BYTE;
        break;
    case 2:
        env->mmu.ssw |= M68K_BA_SIZE_WORD;
        break;
    case 4:
        env->mmu.ssw |= M68K_BA_SIZE_LONG;
        break;
    }
    if (access_type & ACCESS_SUPER) {
        env->mmu.ssw |= M68K_TM_040_SUPER;
    }
    if (access_type & ACCESS_CODE) {
        env->mmu.ssw |= M68K_TM_040_CODE;
    } else {
        env->mmu.ssw |= M68K_TM_040_DATA;
    }
    if (!(access_type & ACCESS_STORE)) {
        env->mmu.ssw |= M68K_RW_040;
    }

    cs->exception_index = EXCP_ACCESS;
    env->mmu.ar = address;
    cpu_loop_exit_restore(cs, retaddr);
}
#endif /* !CONFIG_USER_ONLY */

uint32_t HELPER(bitrev)(uint32_t x)
{
    x = ((x >> 1) & 0x55555555u) | ((x << 1) & 0xaaaaaaaau);
    x = ((x >> 2) & 0x33333333u) | ((x << 2) & 0xccccccccu);
    x = ((x >> 4) & 0x0f0f0f0fu) | ((x << 4) & 0xf0f0f0f0u);
    return bswap32(x);
}

uint32_t HELPER(ff1)(uint32_t x)
{
    int n;
    for (n = 32; x; n--)
        x >>= 1;
    return n;
}

uint32_t HELPER(sats)(uint32_t val, uint32_t v)
{
    /* The result has the opposite sign to the original value.  */
    if ((int32_t)v < 0) {
        val = (((int32_t)val) >> 31) ^ SIGNBIT;
    }
    return val;
}

void cpu_m68k_set_sr(CPUM68KState *env, uint32_t sr)
{
    env->sr = sr & 0xffe0;
    cpu_m68k_set_ccr(env, sr);
    m68k_switch_sp(env);
}

void HELPER(set_sr)(CPUM68KState *env, uint32_t val)
{
    cpu_m68k_set_sr(env, val);
}

/* MAC unit.  */
/*
 * FIXME: The MAC unit implementation is a bit of a mess.  Some helpers
 * take values,  others take register numbers and manipulate the contents
 * in-place.
 */
void HELPER(mac_move)(CPUM68KState *env, uint32_t dest, uint32_t src)
{
    uint32_t mask;
    env->macc[dest] = env->macc[src];
    mask = MACSR_PAV0 << dest;
    if (env->macsr & (MACSR_PAV0 << src))
        env->macsr |= mask;
    else
        env->macsr &= ~mask;
}

uint64_t HELPER(macmuls)(CPUM68KState *env, uint32_t op1, uint32_t op2)
{
    int64_t product;
    int64_t res;

    product = (uint64_t)op1 * op2;
    res = (product << 24) >> 24;
    if (res != product) {
        env->macsr |= MACSR_V;
        if (env->macsr & MACSR_OMC) {
            /* Make sure the accumulate operation overflows.  */
            if (product < 0)
                res = ~(1ll << 50);
            else
                res = 1ll << 50;
        }
    }
    return res;
}

uint64_t HELPER(macmulu)(CPUM68KState *env, uint32_t op1, uint32_t op2)
{
    uint64_t product;

    product = (uint64_t)op1 * op2;
    if (product & (0xffffffull << 40)) {
        env->macsr |= MACSR_V;
        if (env->macsr & MACSR_OMC) {
            /* Make sure the accumulate operation overflows.  */
            product = 1ll << 50;
        } else {
            product &= ((1ull << 40) - 1);
        }
    }
    return product;
}

uint64_t HELPER(macmulf)(CPUM68KState *env, uint32_t op1, uint32_t op2)
{
    uint64_t product;
    uint32_t remainder;

    product = (uint64_t)op1 * op2;
    if (env->macsr & MACSR_RT) {
        remainder = product & 0xffffff;
        product >>= 24;
        if (remainder > 0x800000)
            product++;
        else if (remainder == 0x800000)
            product += (product & 1);
    } else {
        product >>= 24;
    }
    return product;
}

void HELPER(macsats)(CPUM68KState *env, uint32_t acc)
{
    int64_t tmp;
    int64_t result;
    tmp = env->macc[acc];
    result = ((tmp << 16) >> 16);
    if (result != tmp) {
        env->macsr |= MACSR_V;
    }
    if (env->macsr & MACSR_V) {
        env->macsr |= MACSR_PAV0 << acc;
        if (env->macsr & MACSR_OMC) {
            /*
             * The result is saturated to 32 bits, despite overflow occurring
             * at 48 bits.  Seems weird, but that's what the hardware docs
             * say.
             */
            result = (result >> 63) ^ 0x7fffffff;
        }
    }
    env->macc[acc] = result;
}

void HELPER(macsatu)(CPUM68KState *env, uint32_t acc)
{
    uint64_t val;

    val = env->macc[acc];
    if (val & (0xffffull << 48)) {
        env->macsr |= MACSR_V;
    }
    if (env->macsr & MACSR_V) {
        env->macsr |= MACSR_PAV0 << acc;
        if (env->macsr & MACSR_OMC) {
            if (val > (1ull << 53))
                val = 0;
            else
                val = (1ull << 48) - 1;
        } else {
            val &= ((1ull << 48) - 1);
        }
    }
    env->macc[acc] = val;
}

void HELPER(macsatf)(CPUM68KState *env, uint32_t acc)
{
    int64_t sum;
    int64_t result;

    sum = env->macc[acc];
    result = (sum << 16) >> 16;
    if (result != sum) {
        env->macsr |= MACSR_V;
    }
    if (env->macsr & MACSR_V) {
        env->macsr |= MACSR_PAV0 << acc;
        if (env->macsr & MACSR_OMC) {
            result = (result >> 63) ^ 0x7fffffffffffll;
        }
    }
    env->macc[acc] = result;
}

void HELPER(mac_set_flags)(CPUM68KState *env, uint32_t acc)
{
    uint64_t val;
    val = env->macc[acc];
    if (val == 0) {
        env->macsr |= MACSR_Z;
    } else if (val & (1ull << 47)) {
        env->macsr |= MACSR_N;
    }
    if (env->macsr & (MACSR_PAV0 << acc)) {
        env->macsr |= MACSR_V;
    }
    if (env->macsr & MACSR_FI) {
        val = ((int64_t)val) >> 40;
        if (val != 0 && val != -1)
            env->macsr |= MACSR_EV;
    } else if (env->macsr & MACSR_SU) {
        val = ((int64_t)val) >> 32;
        if (val != 0 && val != -1)
            env->macsr |= MACSR_EV;
    } else {
        if ((val >> 32) != 0)
            env->macsr |= MACSR_EV;
    }
}

#define EXTSIGN(val, index) (     \
    (index == 0) ? (int8_t)(val) : ((index == 1) ? (int16_t)(val) : (val)) \
)

#define COMPUTE_CCR(op, x, n, z, v, c) {                                   \
    switch (op) {                                                          \
    case CC_OP_FLAGS:                                                      \
        /* Everything in place.  */                                        \
        break;                                                             \
    case CC_OP_ADDB:                                                       \
    case CC_OP_ADDW:                                                       \
    case CC_OP_ADDL:                                                       \
        res = n;                                                           \
        src2 = v;                                                          \
        src1 = EXTSIGN(res - src2, op - CC_OP_ADDB);                       \
        c = x;                                                             \
        z = n;                                                             \
        v = (res ^ src1) & ~(src1 ^ src2);                                 \
        break;                                                             \
    case CC_OP_SUBB:                                                       \
    case CC_OP_SUBW:                                                       \
    case CC_OP_SUBL:                                                       \
        res = n;                                                           \
        src2 = v;                                                          \
        src1 = EXTSIGN(res + src2, op - CC_OP_SUBB);                       \
        c = x;                                                             \
        z = n;                                                             \
        v = (res ^ src1) & (src1 ^ src2);                                  \
        break;                                                             \
    case CC_OP_CMPB:                                                       \
    case CC_OP_CMPW:                                                       \
    case CC_OP_CMPL:                                                       \
        src1 = n;                                                          \
        src2 = v;                                                          \
        res = EXTSIGN(src1 - src2, op - CC_OP_CMPB);                       \
        n = res;                                                           \
        z = res;                                                           \
        c = src1 < src2;                                                   \
        v = (res ^ src1) & (src1 ^ src2);                                  \
        break;                                                             \
    case CC_OP_LOGIC:                                                      \
        c = v = 0;                                                         \
        z = n;                                                             \
        break;                                                             \
    default:                                                               \
        cpu_abort(env_cpu(env), "Bad CC_OP %d", op);                       \
    }                                                                      \
} while (0)

uint32_t cpu_m68k_get_ccr(CPUM68KState *env)
{
    uint32_t x, c, n, z, v;
    uint32_t res, src1, src2;

    x = env->cc_x;
    n = env->cc_n;
    z = env->cc_z;
    v = env->cc_v;
    c = env->cc_c;

    COMPUTE_CCR(env->cc_op, x, n, z, v, c);

    n = n >> 31;
    z = (z == 0);
    v = v >> 31;

    return x * CCF_X + n * CCF_N + z * CCF_Z + v * CCF_V + c * CCF_C;
}

uint32_t HELPER(get_ccr)(CPUM68KState *env)
{
    return cpu_m68k_get_ccr(env);
}

void cpu_m68k_set_ccr(CPUM68KState *env, uint32_t ccr)
{
    env->cc_x = (ccr & CCF_X ? 1 : 0);
    env->cc_n = (ccr & CCF_N ? -1 : 0);
    env->cc_z = (ccr & CCF_Z ? 0 : 1);
    env->cc_v = (ccr & CCF_V ? -1 : 0);
    env->cc_c = (ccr & CCF_C ? 1 : 0);
    env->cc_op = CC_OP_FLAGS;
}

void HELPER(set_ccr)(CPUM68KState *env, uint32_t ccr)
{
    cpu_m68k_set_ccr(env, ccr);
}

void HELPER(flush_flags)(CPUM68KState *env, uint32_t cc_op)
{
    uint32_t res, src1, src2;

    COMPUTE_CCR(cc_op, env->cc_x, env->cc_n, env->cc_z, env->cc_v, env->cc_c);
    env->cc_op = CC_OP_FLAGS;
}

uint32_t HELPER(get_macf)(CPUM68KState *env, uint64_t val)
{
    int rem;
    uint32_t result;

    if (env->macsr & MACSR_SU) {
        /* 16-bit rounding.  */
        rem = val & 0xffffff;
        val = (val >> 24) & 0xffffu;
        if (rem > 0x800000)
            val++;
        else if (rem == 0x800000)
            val += (val & 1);
    } else if (env->macsr & MACSR_RT) {
        /* 32-bit rounding.  */
        rem = val & 0xff;
        val >>= 8;
        if (rem > 0x80)
            val++;
        else if (rem == 0x80)
            val += (val & 1);
    } else {
        /* No rounding.  */
        val >>= 8;
    }
    if (env->macsr & MACSR_OMC) {
        /* Saturate.  */
        if (env->macsr & MACSR_SU) {
            if (val != (uint16_t) val) {
                result = ((val >> 63) ^ 0x7fff) & 0xffff;
            } else {
                result = val & 0xffff;
            }
        } else {
            if (val != (uint32_t)val) {
                result = ((uint32_t)(val >> 63) & 0x7fffffff);
            } else {
                result = (uint32_t)val;
            }
        }
    } else {
        /* No saturation.  */
        if (env->macsr & MACSR_SU) {
            result = val & 0xffff;
        } else {
            result = (uint32_t)val;
        }
    }
    return result;
}

uint32_t HELPER(get_macs)(uint64_t val)
{
    if (val == (int32_t)val) {
        return (int32_t)val;
    } else {
        return (val >> 61) ^ ~SIGNBIT;
    }
}

uint32_t HELPER(get_macu)(uint64_t val)
{
    if ((val >> 32) == 0) {
        return (uint32_t)val;
    } else {
        return 0xffffffffu;
    }
}

uint32_t HELPER(get_mac_extf)(CPUM68KState *env, uint32_t acc)
{
    uint32_t val;
    val = env->macc[acc] & 0x00ff;
    val |= (env->macc[acc] >> 32) & 0xff00;
    val |= (env->macc[acc + 1] << 16) & 0x00ff0000;
    val |= (env->macc[acc + 1] >> 16) & 0xff000000;
    return val;
}

uint32_t HELPER(get_mac_exti)(CPUM68KState *env, uint32_t acc)
{
    uint32_t val;
    val = (env->macc[acc] >> 32) & 0xffff;
    val |= (env->macc[acc + 1] >> 16) & 0xffff0000;
    return val;
}

void HELPER(set_mac_extf)(CPUM68KState *env, uint32_t val, uint32_t acc)
{
    int64_t res;
    int32_t tmp;
    res = env->macc[acc] & 0xffffffff00ull;
    tmp = (int16_t)(val & 0xff00);
    res |= ((int64_t)tmp) << 32;
    res |= val & 0xff;
    env->macc[acc] = res;
    res = env->macc[acc + 1] & 0xffffffff00ull;
    tmp = (val & 0xff000000);
    res |= ((int64_t)tmp) << 16;
    res |= (val >> 16) & 0xff;
    env->macc[acc + 1] = res;
}

void HELPER(set_mac_exts)(CPUM68KState *env, uint32_t val, uint32_t acc)
{
    int64_t res;
    int32_t tmp;
    res = (uint32_t)env->macc[acc];
    tmp = (int16_t)val;
    res |= ((int64_t)tmp) << 32;
    env->macc[acc] = res;
    res = (uint32_t)env->macc[acc + 1];
    tmp = val & 0xffff0000;
    res |= (int64_t)tmp << 16;
    env->macc[acc + 1] = res;
}

void HELPER(set_mac_extu)(CPUM68KState *env, uint32_t val, uint32_t acc)
{
    uint64_t res;
    res = (uint32_t)env->macc[acc];
    res |= ((uint64_t)(val & 0xffff)) << 32;
    env->macc[acc] = res;
    res = (uint32_t)env->macc[acc + 1];
    res |= (uint64_t)(val & 0xffff0000) << 16;
    env->macc[acc + 1] = res;
}

#if !defined(CONFIG_USER_ONLY)
void HELPER(ptest)(CPUM68KState *env, uint32_t addr, uint32_t is_read)
{
    hwaddr physical;
    int access_type;
    int prot;
    int ret;
    target_ulong page_size;

    access_type = ACCESS_PTEST;
    if (env->dfc & 4) {
        access_type |= ACCESS_SUPER;
    }
    if ((env->dfc & 3) == 2) {
        access_type |= ACCESS_CODE;
    }
    if (!is_read) {
        access_type |= ACCESS_STORE;
    }

    env->mmu.mmusr = 0;
    env->mmu.ssw = 0;
    ret = get_physical_address(env, &physical, &prot, addr,
                               access_type, &page_size);
    if (ret == 0) {
        tlb_set_page(env_cpu(env), addr & TARGET_PAGE_MASK,
                     physical & TARGET_PAGE_MASK,
                     prot, access_type & ACCESS_SUPER ?
                     MMU_KERNEL_IDX : MMU_USER_IDX, page_size);
    }
}

void HELPER(pflush)(CPUM68KState *env, uint32_t addr, uint32_t opmode)
{
    CPUState *cs = env_cpu(env);

    switch (opmode) {
    case 0: /* Flush page entry if not global */
    case 1: /* Flush page entry */
        tlb_flush_page(cs, addr);
        break;
    case 2: /* Flush all except global entries */
        tlb_flush(cs);
        break;
    case 3: /* Flush all entries */
        tlb_flush(cs);
        break;
    }
}

/*
 * The instruction cache is addressed logically, so the instructions after
 * a TC load still come from the page they ran from before, as long as
 * they are in the cache. The LC and LC II ROMs rely on it: they turn the
 * MMU on while running at their 24-bit address, which the new tables do
 * not map, and jump to their 32-bit address a few instructions later. So
 * when, after a TC load that flushes the ATC, instruction fetches from
 * that page find no translation, they get the old one, until the cache
 * is cleared. Code that the new tables do map (the Mac II ROMs remap the
 * page they run in) takes the new translation.
 */
static void m68030_note_code_page(CPUM68KState *env, uint32_t pc)
{
    M68kCPU *cpu = env_archcpu(env);
    uint32_t vpage = pc & TARGET_PAGE_MASK;
    M68030Walk w;

    env->mmu.icache_valid = false;
    if (!m68k_mmu_enabled(env)) {
        env->mmu.icache_ppage = cpu->addr_map ?
            cpu->addr_map(cpu->addr_map_opaque, vpage) : vpage;
    } else {
        uint16_t sr = m68030_walk(env, vpage, 6, false, 16, false, &w);

        if (!m68030_access_ok(sr, &w, true, false)) {
            return;
        }
        env->mmu.icache_ppage = w.physical & TARGET_PAGE_MASK;
    }
    env->mmu.icache_vpage = vpage;
    env->mmu.icache_valid = true;
}

/*
 * 68030 PMOVE: @ext is the extension word, @addr the effective address.
 * Invalid encodings are rejected by the translator.
 */
void HELPER(pmove030)(CPUM68KState *env, uint32_t ext, uint32_t addr,
                      uint32_t next_pc)
{
    CPUState *cs = env_cpu(env);
    uintptr_t ra = GETPC();
    int preg = extract32(ext, 10, 3);
    bool to_mem = ext & 0x200;
    uint64_t val64;
    uint32_t val;

    /*
     * FD (flush disable) keeps the ATC, which QEMU's TLB stands in for.
     * The Mac ROM's SwapMMUMode relies on it: it loads the new CRP with
     * FD set, and the instructions up to the TC load still run through
     * the old translations.
     */
    bool fd = ext & 0x100;
    switch (ext >> 13) {
    case 0: /* TT0, TT1 */
        if (to_mem) {
            cpu_stl_be_mmuidx_ra(env, addr, env->mmu.tt030[preg & 1],
                                 MMU_KERNEL_IDX, ra);
        } else {
            val = cpu_ldl_be_mmuidx_ra(env, addr, MMU_KERNEL_IDX, ra);
            env->mmu.tt030[preg & 1] = val;
            if (!fd) {
                tlb_flush(cs);
            }
        }
        return;
    case 2:
        if (preg == 0) { /* TC */
            if (to_mem) {
                cpu_stl_be_mmuidx_ra(env, addr, env->mmu.tc030,
                                     MMU_KERNEL_IDX, ra);
                return;
            }
            val = cpu_ldl_be_mmuidx_ra(env, addr, MMU_KERNEL_IDX, ra);
            if (!fd) {
                /* with FD the ATC keeps the old translations anyway */
                m68030_note_code_page(env, next_pc);
                tlb_flush(cs);
            }
            if ((val & M68K_TC030_E) && !m68030_tc_valid(val)) {
                env->mmu.tc030 = val & ~M68K_TC030_E;
                break;
            }
            env->mmu.tc030 = val;
            qemu_log_mask(CPU_LOG_MMU, "68030 TC=%08x CRP=%016" PRIx64
                          " TT0=%08x TT1=%08x\n", val, env->mmu.crp030,
                          env->mmu.tt030[0], env->mmu.tt030[1]);
            return;
        }
        /* SRP (2), CRP (3) */
        if (to_mem) {
            val64 = preg == 2 ? env->mmu.srp030 : env->mmu.crp030;
            cpu_stl_be_mmuidx_ra(env, addr, val64 >> 32, MMU_KERNEL_IDX, ra);
            cpu_stl_be_mmuidx_ra(env, addr + 4, val64, MMU_KERNEL_IDX, ra);
            return;
        }
        val64 = (uint64_t)cpu_ldl_be_mmuidx_ra(env, addr, MMU_KERNEL_IDX,
                                               ra) << 32;
        val64 |= cpu_ldl_be_mmuidx_ra(env, addr + 4, MMU_KERNEL_IDX, ra);
        if (preg == 2) {
            env->mmu.srp030 = val64;
        } else {
            env->mmu.crp030 = val64;
        }
        qemu_log_mask(CPU_LOG_MMU, "68030 %s=%016" PRIx64 " pc=%08x\n",
                      preg == 2 ? "SRP" : "CRP", val64, env->pc);
        if (!fd) {
            tlb_flush(cs);
        }
        if (((val64 >> 32) & 3) == M68030_DT_INVALID) {
            break;
        }
        return;
    case 3: /* MMUSR */
        if (to_mem) {
            cpu_stw_be_mmuidx_ra(env, addr, env->mmu.mmusr030,
                                 MMU_KERNEL_IDX, ra);
        } else {
            env->mmu.mmusr030 = cpu_lduw_be_mmuidx_ra(env, addr,
                                                      MMU_KERNEL_IDX, ra);
        }
        return;
    default:
        g_assert_not_reached();
    }

    /* MMU configuration exception, taken after the instruction */
    cpu_restore_state(cs, ra);
    env->pc = next_pc;
    cs->exception_index = EXCP_MMU_CONF;
    cpu_loop_exit(cs);
}

void HELPER(pflush030)(CPUM68KState *env)
{
    /* PFLUSHA, PFLUSH FC,#mask[,<ea>]: flush everything */
    tlb_flush(env_cpu(env));
}

void HELPER(pload030)(CPUM68KState *env, uint32_t addr, uint32_t fc,
                      uint32_t is_read)
{
    M68030Walk w;

    /* there is no ATC to load: only update the descriptors */
    if ((env->mmu.tc030 & M68K_TC030_E) &&
        !m68030_tt_match(env, addr, fc, !is_read)) {
        m68030_walk(env, addr, fc, !is_read, 16, true, &w);
    }
}

/* returns the address of the last descriptor, for PTEST ...,An */
uint32_t HELPER(ptest030)(CPUM68KState *env, uint32_t addr, uint32_t fc,
                          uint32_t ext)
{
    int level = extract32(ext, 10, 3);
    bool write = !(ext & 0x200);
    M68030Walk w;
    uint16_t sr;

    if (level == 0) {
        /* search the ATC: report what a table search would load into it */
        if (m68030_tt_match(env, addr, fc, write)) {
            env->mmu.mmusr030 = M68K_MMUSR030_T;
            return 0;
        }
        if (!(env->mmu.tc030 & M68K_TC030_E)) {
            env->mmu.mmusr030 = M68K_MMUSR030_I;
            return 0;
        }
        sr = m68030_walk(env, addr, fc, write, 16, false, &w);
        if ((sr & (M68K_MMUSR030_B | M68K_MMUSR030_L)) ||
            (!(fc & 4) && (sr & M68K_MMUSR030_S))) {
            sr |= M68K_MMUSR030_B;
        }
        if (!w.resolved) {
            sr |= M68K_MMUSR030_I;
        }
        env->mmu.mmusr030 = sr & (M68K_MMUSR030_B | M68K_MMUSR030_W |
                                  M68K_MMUSR030_I | M68K_MMUSR030_M);
        return 0;
    }

    /* a table search: transparent translation is not checked */
    sr = m68030_walk(env, addr, fc, write, level, false, &w);
    if (fc & 4) {
        /* S is a supervisor-only violation for user function codes */
        sr &= ~M68K_MMUSR030_S;
    }
    env->mmu.mmusr030 = sr;
    return w.desc_addr;
}

void HELPER(reset)(CPUM68KState *env)
{
    /* the board resets the devices on its reset line */
    qemu_irq_pulse(env_archcpu(env)->reset_out);
}

/* one pass of a `dbcc dN,self` delay loop */
void HELPER(delay_loop)(CPUM68KState *env)
{
    calib_governor_delay_insn(env_cpu(env));
}
#endif /* !CONFIG_USER_ONLY */

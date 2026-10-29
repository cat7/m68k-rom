/*
 * Macintosh Quadra 950
 *
 * The machine itself is built by the shared 68040 Quadra code in
 * mac-quadra.c; this file only describes how the Quadra 950 differs.
 *
 * It is a 33 MHz Quadra 900 with its own ROM (3DC27823), which also runs
 * the Quadra 700 and 900. The ROM tells the three apart from VIA1 port A
 * ID bits 6,4,2,1: Quadra 700 1,0,0,0, Quadra 900 1,1,0,0 and Quadra 950
 * 0,1,0,0. Like the 900 it has an SCC IOP, a SWIM IOP that also runs ADB,
 * two 53C96 SCSI buses and five NuBus slots.
 *
 * Its DAFB came with 1 MB of VRAM and takes 2 MB at most; the model has
 * the full 2 MB, so the 16" monitor gets millions of colours.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "target/m68k/cpu.h"
#include "hw/m68k/mac-quadra.h"
#include "hw/misc/mac_via.h"
#include "standard-headers/asm-m68k/bootinfo-mac.h"

static const MacQuadraModel q950_model = {
    .desc = "Macintosh Quadra 950",
    .calib_mips = 10,
    .rom_filename = "Quadra950.rom",
    .bootinfo_model = MAC_MODEL_Q950,
    .via1_cpuid = VIA1A_CPUID2,
    /* PA0 low at power-on makes the ROM enter its factory test mode */
    .via1_porta_pullup = 0x01,
    /* PB0: front panel key switch not in the "secure" position */
    .via2_portb_pullup = 0x01,
    .memctrl = MAC_MEMCTRL_ORWELL,
    .floppy = MAC_FLOPPY_SWIM1,
    .iop = true,
    /* clock, PRAM and soft power */
    .egret = true,
    .has_machine_id_reg = false,
    .default_easc = false,
    .scsi_drq_dafb = true,
    .esp_base = MAC_QUADRA_IO_BASE + 0x0f000,
    .esp_pdma = MAC_QUADRA_IO_BASE + 0x0f100,
    .esp2_base = MAC_QUADRA_IO_BASE + 0x0f402,
    .esp2_pdma = MAC_QUADRA_IO_BASE + 0x0f502,
    /* slot 0x9 is the in-built framebuffer, 0xa-0xe the five NuBus slots */
    .nubus_slots = BIT(0x9) | BIT(0xa) | BIT(0xb) | BIT(0xc) | BIT(0xd) |
                   BIT(0xe),
    .max_ram_size = 256 * MiB,
    .vram_size = 2 * MiB,
    .fb_width = 832,
    .fb_height = 624,
    .fb_depth = 8,
};

static const TypeInfo q950_machine_types[] = {
    {
        .name       = TYPE_Q950_MACHINE,
        .parent     = TYPE_MAC_QUADRA_MACHINE,
        .class_init = mac_quadra_model_class_init,
        .class_data = &q950_model,
    },
};

DEFINE_TYPES(q950_machine_types)

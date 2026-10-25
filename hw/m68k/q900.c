/*
 * Macintosh Quadra 900
 *
 * The machine itself is built by the shared 68040 Quadra code in
 * mac-quadra.c; this file only describes how the Quadra 900 differs.
 *
 * It uses the Quadra 700's ROM (420DBFF3), which identifies it from VIA1
 * port A ID bits 6,4,2,1 = 1,1,0,0 behind an Orwell memory controller.
 * The ROM's address table for this model has no direct SCC or SWIM:
 * both sit behind I/O processors (SCC IOP at 0x50f0c000, SWIM IOP, which
 * also runs ADB, at 0x50f1e000). It lists two 53C96 SCSI buses, the
 * internal one at 0x50f0f000 and the external one at 0x50f0f402.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "target/m68k/cpu.h"
#include "hw/m68k/mac-quadra.h"
#include "hw/misc/mac_via.h"
#include "standard-headers/asm-m68k/bootinfo-mac.h"

static const MacQuadraModel q900_model = {
    .desc = "Macintosh Quadra 900",
    .rom_filename = "Quadra900.rom",
    .bootinfo_model = MAC_MODEL_Q900,
    .via1_cpuid = VIA1A_CPUID3 | VIA1A_CPUID2,
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
    /* slot 0x9 is the in-built framebuffer, 0xa-0xe the five NuBus slots */
    .nubus_slots = BIT(0x9) | BIT(0xa) | BIT(0xb) | BIT(0xc) | BIT(0xd) |
                   BIT(0xe),
    .max_ram_size = 256 * MiB,
    .fb_width = 832,
    .fb_height = 624,
    .fb_depth = 8,
};

static const TypeInfo q900_machine_types[] = {
    {
        .name       = TYPE_Q900_MACHINE,
        .parent     = TYPE_MAC_QUADRA_MACHINE,
        .class_init = mac_quadra_model_class_init,
        .class_data = &q900_model,
    },
};

DEFINE_TYPES(q900_machine_types)

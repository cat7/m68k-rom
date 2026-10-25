/*
 * Macintosh Quadra 700
 *
 * The machine itself is built by the shared 68040 Quadra code in
 * mac-quadra.c; this file only describes how the Quadra 700 differs.
 *
 * Its ROM (checksum 420DBFF3, shared with the Quadra 900) identifies the
 * model from VIA1 port A ID bits 6,4,2,1 = 1,0,0,0 behind an Orwell
 * memory controller. The ROM's address table for this model lists one
 * 53C96 SCSI bus at 0x50f0f000 and no IOPs.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "target/m68k/cpu.h"
#include "hw/m68k/mac-quadra.h"
#include "hw/misc/mac_via.h"
#include "standard-headers/asm-m68k/bootinfo-mac.h"

static const MacQuadraModel q700_model = {
    .desc = "Macintosh Quadra 700",
    .rom_filename = "Quadra700.rom",
    .bootinfo_model = MAC_MODEL_Q700,
    .via1_cpuid = VIA1A_CPUID3,
    /* PA0 low at power-on makes the ROM enter its factory test mode */
    .via1_porta_pullup = 0x01,
    .memctrl = MAC_MEMCTRL_ORWELL,
    .floppy = MAC_FLOPPY_SWIM1,
    .has_machine_id_reg = false,
    .default_easc = false,
    .scsi_drq_dafb = true,
    .esp_base = MAC_QUADRA_IO_BASE + 0x0f000,
    .esp_pdma = MAC_QUADRA_IO_BASE + 0x0f100,
    /* slot 0x9 is the in-built framebuffer; slot numbering to be verified */
    .nubus_slots = BIT(0x9) | BIT(0xd) | BIT(0xe),
    .max_ram_size = 256 * MiB,
};

static const TypeInfo q700_machine_types[] = {
    {
        .name       = TYPE_Q700_MACHINE,
        .parent     = TYPE_MAC_QUADRA_MACHINE,
        .class_init = mac_quadra_model_class_init,
        .class_data = &q700_model,
    },
};

DEFINE_TYPES(q700_machine_types)

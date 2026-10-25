/*
 * Macintosh Quadra 800
 *
 * The machine itself is built by the shared 68040 Quadra code in
 * mac-quadra.c; this file only describes how the Quadra 800 differs.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "target/m68k/cpu.h"
#include "hw/m68k/mac-quadra.h"
#include "hw/misc/mac_via.h"
#include "standard-headers/asm-m68k/bootinfo-mac.h"

static const MacQuadraModel q800_model = {
    .desc = "Macintosh Quadra 800",
    .rom_filename = "MacROM.bin",
    .bootinfo_model = MAC_MODEL_Q800,
    .via1_cpuid = VIA1A_CPUID_Q800,
    .memctrl = MAC_MEMCTRL_DJMEMC,
    .has_machine_id_reg = true,
    .default_easc = true,
    .esp_base = MAC_QUADRA_IO_BASE + 0x10000,
    .esp_pdma = MAC_QUADRA_IO_BASE + 0x10100,
    /*
     * Slot 0x9 is reserved for use by the in-built framebuffer whilst only
     * slots 0xc, 0xd and 0xe physically exist on the Quadra 800
     */
    .nubus_slots = BIT(0x9) | BIT(0xc) | BIT(0xd) | BIT(0xe),
    .max_ram_size = 1 * GiB,
};

static const TypeInfo q800_machine_types[] = {
    {
        .name       = TYPE_Q800_MACHINE,
        .parent     = TYPE_MAC_QUADRA_MACHINE,
        .class_init = mac_quadra_model_class_init,
        .class_data = &q800_model,
    },
};

DEFINE_TYPES(q800_machine_types)

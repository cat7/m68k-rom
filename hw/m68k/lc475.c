/*
 * Macintosh LC 475 (and its Quadra 605 / Performa 475 siblings)
 *
 * The machine itself is built by the shared 68040 Quadra code in
 * mac-quadra.c; this file only describes how the LC 475 differs.
 *
 * Its ROM (checksum FF7439EE, shared with the Quadra 605 and LC 575)
 * identifies the board from the low word of the machine ID register at
 * 0x5ffffffc: 0x2221 is the LC 475 (Gestalt 89). The ROM's address map
 * for it is the Quadra 800's without the SONIC Ethernet: VIAs, SCC and
 * SWIM at the usual places, one 53C96 SCSI bus at 0x50f10000 and the
 * on-board video at 0xf9800000. The memory controller (MEMCjr) and I/O
 * chip (PrimeTime) are register compatible with djMEMC and IOSB. ADB,
 * the clock and PRAM are behind a Cuda microcontroller on VIA1.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "target/m68k/cpu.h"
#include "hw/m68k/mac-quadra.h"
#include "hw/misc/mac_via.h"
#include "standard-headers/asm-m68k/bootinfo-mac.h"

static const MacQuadraModel lc475_model = {
    .desc = "Macintosh LC 475",
    .rom_filename = "LC475.rom",
    .bootinfo_model = MAC_MODEL_P475,
    .via1_cpuid = VIA1A_CPUID_Q800,
    .memctrl = MAC_MEMCTRL_DJMEMC,
    .floppy = MAC_FLOPPY_SWIM2,
    .has_machine_id_reg = true,
    .machine_id = 0xa55a2221,
    .no_sonic = true,
    .cuda = true,
    .rom_mirror = true,
    .default_easc = true,
    .esp_base = MAC_QUADRA_IO_BASE + 0x10000,
    .esp_pdma = MAC_QUADRA_IO_BASE + 0x10100,
    /* no NuBus: slot 0x9 is the in-built framebuffer, 0xe the PDS slot */
    .nubus_slots = BIT(0x9) | BIT(0xe),
    /* 4 MiB on board plus one SIMM; the ROM handles up to 132 MiB */
    .max_ram_size = 132 * MiB,
    .fb_width = 640,
    .fb_height = 480,
    .fb_depth = 8,
    /* 512 KiB soldered, 1 MiB with the VRAM SIMM */
    .vram_size = 1 * MiB,
};

static const TypeInfo lc475_machine_types[] = {
    {
        .name       = TYPE_LC475_MACHINE,
        .parent     = TYPE_MAC_QUADRA_MACHINE,
        .class_init = mac_quadra_model_class_init,
        .class_data = &lc475_model,
    },
};

DEFINE_TYPES(lc475_machine_types)

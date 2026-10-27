/*
 * SCSI device identity for the 68k Macs
 *
 * Apple's disk utilities (Apple HD SC Setup in particular) only work with
 * drives that Apple shipped, so the hard disks identify as Apple's
 * Seagate ST225N and the CD-ROMs as Apple's Matsushita CR-8005, with the
 * mode pages the Apple drivers expect.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_M68K_MAC_SCSI_COMPAT_H
#define HW_M68K_MAC_SCSI_COMPAT_H

#include "hw/core/boards.h"

static GlobalProperty mac_scsi_compat[] = {
    { "scsi-hd", "quirk_mode_page_vendor_specific_apple", "on" },
    { "scsi-hd", "quirk_mode_page_set_block_size", "on" },
    { "scsi-hd", "vendor", " SEAGATE" },
    { "scsi-hd", "product", "          ST225N" },
    { "scsi-hd", "ver", "1.0 " },
    { "scsi-cd", "quirk_mode_page_apple_vendor", "on" },
    { "scsi-cd", "quirk_mode_sense_rom_use_dbd", "on" },
    { "scsi-cd", "quirk_mode_page_vendor_specific_apple", "on" },
    { "scsi-cd", "quirk_mode_page_set_block_size", "on" },
    { "scsi-cd", "quirk_mode_page_truncated", "on" },
    { "scsi-cd", "vendor", "MATSHITA" },
    { "scsi-cd", "product", "CD-ROM CR-8005" },
    { "scsi-cd", "ver", "1.0k" },
};

static inline void mac_scsi_compat_add(MachineClass *mc)
{
    compat_props_add(mc->compat_props, mac_scsi_compat,
                     G_N_ELEMENTS(mac_scsi_compat));
}

#endif

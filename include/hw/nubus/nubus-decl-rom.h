/*
 * Builder for NuBus declaration ROMs
 *
 * The ROM is built in a buffer using all four byte lanes, so ROM offsets
 * are plain byte offsets.  Lists are written with forward offsets that
 * are patched once their target is known; nubus_decl_rom_finish() then
 * moves everything to the top of the buffer and appends the format block.
 *
 * Reference: Designing Cards and Drivers for the Macintosh Family
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_NUBUS_NUBUS_DECL_ROM_H
#define HW_NUBUS_NUBUS_DECL_ROM_H

/* sResource list entry IDs */
#define NUBUS_sRsrcType         1
#define NUBUS_sRsrcName         2
#define NUBUS_sRsrcIcon         3
#define NUBUS_sRsrcDrvrDir      4
#define NUBUS_sRsrcLoadRec      5
#define NUBUS_sRsrcBootRec      6
#define NUBUS_sRsrcFlags        7
#define NUBUS_sRsrcHWDevId      8
#define NUBUS_MinorBaseOS       10
#define NUBUS_MinorLength       11
#define NUBUS_MajorBaseOS       12
#define NUBUS_MajorLength       13
#define NUBUS_sGammaDir         64
#define NUBUS_BoardId           32
#define NUBUS_PRAMInitData      33
#define NUBUS_PrimaryInit       34
#define NUBUS_VendorInfo        36

/* VendorInfo list */
#define NUBUS_VendorId          1
#define NUBUS_SerialNum         2
#define NUBUS_RevLevel          3
#define NUBUS_PartNum           4
#define NUBUS_Date              5

/* sRsrcDrvrDir */
#define NUBUS_sMacOS68000       1
#define NUBUS_sMacOS68020       2

/* sRsrcFlags */
#define NUBUS_fOpenAtStart      (1 << 1)
#define NUBUS_f32BitMode        (1 << 2)

/* sRsrcType categories */
#define NUBUS_catBoard          0x0001
#define NUBUS_catDisplay        0x0003
#define NUBUS_catNetwork        0x0004
#define NUBUS_typeVideo         0x0001
#define NUBUS_typeEthernet      0x0001
#define NUBUS_drSwApple         0x0001

typedef struct NubusDeclROM {
    uint8_t *buf;
    int size;
    int pos;
} NubusDeclROM;

void nubus_decl_rom_init(NubusDeclROM *r, uint8_t *buf, int size);
void nubus_decl_rom_byte(NubusDeclROM *r, uint8_t v);
void nubus_decl_rom_word(NubusDeclROM *r, uint16_t v);
void nubus_decl_rom_long(NubusDeclROM *r, uint32_t v);
void nubus_decl_rom_bytes(NubusDeclROM *r, const void *data, int len);
void nubus_decl_rom_align(NubusDeclROM *r);
void nubus_decl_rom_cstring(NubusDeclROM *r, const char *s);

/*
 * List entries.  nubus_decl_rom_entry_off() returns the entry's position
 * so that its offset can be set later with nubus_decl_rom_patch_off().
 */
int nubus_decl_rom_entry_off(NubusDeclROM *r, uint8_t id);
void nubus_decl_rom_patch_off(NubusDeclROM *r, int at, int target);
void nubus_decl_rom_entry_data(NubusDeclROM *r, uint8_t id, uint32_t data);
void nubus_decl_rom_end(NubusDeclROM *r);

/* Patch the entry at @at to point here */
static inline void nubus_decl_rom_here(NubusDeclROM *r, int at)
{
    nubus_decl_rom_patch_off(r, at, r->pos);
}

/*
 * Move the ROM to the top of the buffer and add the format block, with
 * the sResource directory at offset @dir.
 */
void nubus_decl_rom_finish(NubusDeclROM *r, int dir);

#endif

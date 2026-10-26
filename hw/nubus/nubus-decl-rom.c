/*
 * Builder for NuBus declaration ROMs
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/bswap.h"
#include "hw/nubus/nubus-decl-rom.h"

#define FORMAT_BLOCK_SIZE   20

void nubus_decl_rom_init(NubusDeclROM *r, uint8_t *buf, int size)
{
    r->buf = buf;
    r->size = size;
    r->pos = 0;
    memset(buf, 0, size);
}

void nubus_decl_rom_byte(NubusDeclROM *r, uint8_t v)
{
    g_assert(r->pos < r->size);
    r->buf[r->pos++] = v;
}

void nubus_decl_rom_word(NubusDeclROM *r, uint16_t v)
{
    nubus_decl_rom_byte(r, v >> 8);
    nubus_decl_rom_byte(r, v);
}

void nubus_decl_rom_long(NubusDeclROM *r, uint32_t v)
{
    nubus_decl_rom_word(r, v >> 16);
    nubus_decl_rom_word(r, v);
}

void nubus_decl_rom_bytes(NubusDeclROM *r, const void *data, int len)
{
    g_assert(r->pos + len <= r->size);
    memcpy(r->buf + r->pos, data, len);
    r->pos += len;
}

void nubus_decl_rom_align(NubusDeclROM *r)
{
    while (r->pos & 1) {
        nubus_decl_rom_byte(r, 0);
    }
}

void nubus_decl_rom_cstring(NubusDeclROM *r, const char *s)
{
    nubus_decl_rom_bytes(r, s, strlen(s) + 1);
    nubus_decl_rom_align(r);
}

int nubus_decl_rom_entry_off(NubusDeclROM *r, uint8_t id)
{
    int at = r->pos;

    nubus_decl_rom_long(r, (uint32_t)id << 24);
    return at;
}

void nubus_decl_rom_patch_off(NubusDeclROM *r, int at, int target)
{
    uint32_t v = ((uint32_t)r->buf[at] << 24) | ((target - at) & 0xffffff);

    stl_be_p(r->buf + at, v);
}

void nubus_decl_rom_entry_data(NubusDeclROM *r, uint8_t id, uint32_t data)
{
    nubus_decl_rom_long(r, ((uint32_t)id << 24) | (data & 0xffffff));
}

void nubus_decl_rom_end(NubusDeclROM *r)
{
    nubus_decl_rom_long(r, 0xff000000);
}

void nubus_decl_rom_finish(NubusDeclROM *r, int dir)
{
    int len = r->pos + FORMAT_BLOCK_SIZE;
    int start = r->size - len;
    uint32_t crc;
    int i;

    g_assert(start >= 0);

    /* Offsets are relative, so the ROM image can simply be moved */
    memmove(r->buf + start, r->buf, r->pos);
    memset(r->buf, 0, start);

    /* Format block, ending at the last byte */
    r->pos = r->size - FORMAT_BLOCK_SIZE;
    nubus_decl_rom_long(r, (uint32_t)(start + dir - r->pos) & 0xffffff);
    nubus_decl_rom_long(r, len);                    /* fhLength */
    nubus_decl_rom_long(r, 0);                      /* fhCRC */
    nubus_decl_rom_byte(r, 1);                      /* fhROMRev */
    nubus_decl_rom_byte(r, 1);                      /* appleFormat */
    nubus_decl_rom_long(r, 0x5a932bc7);             /* fhTstPat */
    nubus_decl_rom_byte(r, 0);                      /* fhReserved */
    nubus_decl_rom_byte(r, 0x0f);                   /* fhByteLanes: all */

    /* CRC: rotate left and add each byte, the CRC field counting as 0 */
    crc = 0;
    for (i = start; i < r->size; i++) {
        crc = rol32(crc, 1);
        if (i < r->size - 12 || i >= r->size - 8) {
            crc += r->buf[i];
        }
    }
    stl_be_p(r->buf + r->size - 12, crc);
}

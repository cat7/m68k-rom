/*
 * QEMU Macintosh SuperDrive floppy disk drive emulator
 *
 * Copyright (c) 2025 Matt Jacobson <mhjacobson@me.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "hw/block/sony_superdrive.h"
#include "system/block-backend.h"
#include "system/block-backend-io.h"
#include "system/blockdev.h"
#include "trace.h"

/* Sony drive register selects (CA2-CA1-CA0-SEL) */
#define SONY_REG_DIRTN       0x0
#define SONY_REG_CSTIN       0x1 /* a.k.a. dipAdr ("disk in place") */
#define SONY_REG_STEP        0x2
#define SONY_REG_WRTPRT      0x3
#define SONY_REG_MOTORON     0x4
#define SONY_REG_TKO         0x5
#define SONY_REG_EJECT_L     0x6
#define SONY_REG_TACH        0x7
#define SONY_REG_RDDATA0     0x8
#define SONY_REG_RDDATA1     0x9
#define SONY_REG_SUPERDRIVE  0xa /* a.k.a. mfmDrvAdr */
#define SONY_REG_GCR         0xb
#define SONY_REG_SINGLE_SIDE 0xc
#define SONY_REG_READY       0xd
#define SONY_REG_INSTALLED   0xe /* a.k.a. DRVIN, a.k.a. drvExstAdr */
#define SONY_REG_TWOMEG      0xf

static const char *const sony_drive_reg_names[] = {
    [SONY_REG_DIRTN] = "DIRTN",
    [SONY_REG_CSTIN] = "CSTIN",
    [SONY_REG_STEP] = "STEP",
    [SONY_REG_WRTPRT] = "WRTPRT",
    [SONY_REG_MOTORON] = "MOTOR_ON",
    [SONY_REG_TKO] = "TKO",
    [SONY_REG_EJECT_L] = "EJECT_L",
    [SONY_REG_TACH] = "TACH",
    [SONY_REG_RDDATA0] = "RDDATA0",
    [SONY_REG_RDDATA1] = "RDDATA1",
    [SONY_REG_SUPERDRIVE] = "SUPERDRIVE",
    [SONY_REG_GCR] = "GCR",
    [SONY_REG_SINGLE_SIDE] = "SINGLE_SIDE",
    [SONY_REG_READY] = "READY",
    [SONY_REG_INSTALLED] = "INSTALLED",
    [SONY_REG_TWOMEG] = "TWOMEG",
};

/* Sony drive write commands (CA2-CA1-CA0-SEL) */
#define SONY_CMD_TRACK_POS   0x0
#define SONY_CMD_TRACK_NEG   0x8
#define SONY_CMD_TRACK_STEP  0x2
#define SONY_CMD_SET_MFM     0x3
#define SONY_CMD_SET_GCR     0xb
#define SONY_CMD_MOTOR_ON    0x4
#define SONY_CMD_MOTOR_OFF   0xc
#define SONY_CMD_EJECT       0xe

static const char *const sony_drive_cmd_names[] = {
    [SONY_CMD_TRACK_POS] = "TRACK_POS",
    [SONY_CMD_TRACK_NEG] = "TRACK_NEG",
    [SONY_CMD_TRACK_STEP] = "TRACK_STEP",
    [SONY_CMD_SET_MFM] = "SET_MFM",
    [SONY_CMD_SET_GCR] = "SET_GCR",
    [SONY_CMD_MOTOR_ON] = "MOTOR_ON",
    [SONY_CMD_MOTOR_OFF] = "MOTOR_OFF",
    [SONY_CMD_EJECT] = "EJECT",
};

#define SONY_GCR_TAG_LENGTH      12
#define SONY_GCR_DATA_LENGTH    512
#define SONY_GCR_SECTOR_LENGTH  (SONY_GCR_TAG_LENGTH + SONY_GCR_DATA_LENGTH)
#define SONY_GCR_ENCODED_LENGTH 699 /* i.e., sony_6and2_packed_len(524) */
#define SONY_GCR_SYNC_LEN         6

static uint8_t sony_drive_sectors_for_track(const SonyDrive *const drive,
                                            const uint8_t track)
{
    if (drive->sectors_per_track) {
        return drive->sectors_per_track;
    } else {
        /*
         * 400K/800K GCR media use zone recording with variable sectors per
         * track; each zone spans 16 tracks.
         */
        static const uint8_t gcr_zone_spt[] = { 12, 11, 10, 9, 8 };
        uint8_t zone = track / 16;

        if (zone >= ARRAY_SIZE(gcr_zone_spt)) {
            zone = ARRAY_SIZE(gcr_zone_spt) - 1;
        }

        return gcr_zone_spt[zone];
    }
}

static void bitset_set(uint8_t *const bitset, const unsigned int index,
                       const bool value)
{
    const uint8_t mask = (uint8_t)(1u << (index & 0x7));

    if (value) {
        bitset[index >> 3] |= mask;
    } else {
        bitset[index >> 3] &= (uint8_t)~mask;
    }
}

static void bitset_set_range(uint8_t *const bitset, const unsigned int index,
                             const unsigned int length, const bool value)
{
    for (unsigned int i = index; i < index + length; i++) {
        bitset_set(bitset, i, value);
    }
}

static bool bitset_get(const uint8_t *const bitset, const unsigned int index)
{
    const uint8_t mask = (uint8_t)(1u << (index & 0x7));
    return (bitset[index >> 3] & mask) != 0;
}

static const uint8_t sony_gcr_encode_table[] = {
    0x96, 0x97, 0x9a, 0x9b, 0x9d, 0x9e, 0x9f, 0xa6,
    0xa7, 0xab, 0xac, 0xad, 0xae, 0xaf, 0xb2, 0xb3,
    0xb4, 0xb5, 0xb6, 0xb7, 0xb9, 0xba, 0xbb, 0xbc,
    0xbd, 0xbe, 0xbf, 0xcb, 0xcd, 0xce, 0xcf, 0xd3,
    0xd6, 0xd7, 0xd9, 0xda, 0xdb, 0xdc, 0xdd, 0xde,
    0xdf, 0xe5, 0xe6, 0xe7, 0xe9, 0xea, 0xeb, 0xec,
    0xed, 0xee, 0xef, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6,
    0xf7, 0xf9, 0xfa, 0xfb, 0xfc, 0xfd, 0xfe, 0xff,
};

/* 0xff is used as filler for bytes the decoder should never see. */
static const uint8_t sony_gcr_decode_table[] = {
    0x00, 0x01, 0xff, 0xff, 0x02, 0x03, 0xff, 0x04,
    0x05, 0x06, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0x07, 0x08, 0xff, 0xff, 0xff, 0x09, 0x0a, 0x0b,
    0x0c, 0x0d, 0xff, 0xff, 0x0e, 0x0f, 0x10, 0x11,
    0x12, 0x13, 0xff, 0x14, 0x15, 0x16, 0x17, 0x18,
    0x19, 0x1a, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0x1b, 0xff, 0x1c,
    0x1d, 0x1e, 0xff, 0xff, 0xff, 0x1f, 0xff, 0xff,
    0x20, 0x21, 0xff, 0x22, 0x23, 0x24, 0x25, 0x26,
    0x27, 0x28, 0xff, 0xff, 0xff, 0xff, 0xff, 0x29,
    0x2a, 0x2b, 0xff, 0x2c, 0x2d, 0x2e, 0x2f, 0x30,
    0x31, 0x32, 0xff, 0xff, 0x33, 0x34, 0x35, 0x36,
    0x37, 0x38, 0xff, 0x39, 0x3a, 0x3b, 0x3c, 0x3d,
    0x3e, 0x3f,
};

static uint8_t sony_gcr_encode_byte(const uint8_t value)
{
    assert((value & 0xc0) == 0);
    return sony_gcr_encode_table[value];
}

/* the 6-bit value of a disk byte, or -1 if it is not a valid GCR byte */
static int sony_gcr_decode_byte(const uint8_t value)
{
    if (value < 0x96) {
        return -1;
    }
    const uint8_t result = sony_gcr_decode_table[value - 0x96];
    return result == 0xff ? -1 : result;
}

static void sony_gcr_encode_bytes(const uint8_t *const in, uint8_t *const out,
                                  const size_t len)
{
    for (size_t i = 0; i < len; i++) {
        out[i] = sony_gcr_encode_byte(in[i]);
    }
}

static bool sony_gcr_decode_bytes(const uint8_t *const in, uint8_t *const out,
                                  const size_t len)
{
    for (size_t i = 0; i < len; i++) {
        const int v = sony_gcr_decode_byte(in[i]);

        if (v < 0) {
            return false;
        }
        out[i] = v;
    }
    return true;
}

static size_t sony_6and2_packed_len(const size_t in_size)
{
    const size_t groups = in_size / 3;
    const size_t rem = in_size % 3;
    return groups * 4 + (rem ? (rem + 1) : 0);
}

static void sony_6and2_pack(const uint8_t *in, const size_t in_size,
                            uint8_t *restrict out, const size_t out_size)
{
    assert(sony_6and2_packed_len(in_size) == out_size);

    for (size_t i = 0; i < in_size; i += 3) {
        /* Allow partial groupings. */
        const bool have_b = (i + 1) < in_size;
        const bool have_c = (i + 2) < in_size;

        const uint8_t a = *in++;
        const uint8_t b = have_b ? *in++ : 0;
        const uint8_t c = have_c ? *in++ : 0;

        /* High two bits (A7:6, B7:6, C7:6), then low six bits. */
        const uint8_t high  = (uint8_t)(((a & 0xC0u) >> 2) |
                                        ((b & 0xC0u) >> 4) |
                                        ((c & 0xC0u) >> 6));
        const uint8_t low_a = (uint8_t)(a & 0x3Fu);
        const uint8_t low_b = (uint8_t)(b & 0x3Fu);
        const uint8_t low_c = (uint8_t)(c & 0x3Fu);

        *out++ = high;
        *out++ = low_a;

        if (have_b) {
            *out++ = low_b;
        }

        if (have_c) {
            *out++ = low_c;
        }
    }
}

static void sony_6and2_unpack(const uint8_t *in, const size_t in_size,
                              uint8_t *restrict out, const size_t out_size)
{
    assert(sony_6and2_packed_len(out_size) == in_size);

    for (size_t i = 0; i < in_size; i += 4) {
        /* Allow partial groupings. */
        const bool have_b = (i + 2) < in_size;
        const bool have_c = (i + 3) < in_size;

        /* High two bits (A7:6, B7:6, C7:6), then low six bits. */
        const uint8_t high = *in++;
        const uint8_t low_a = *in++;
        const uint8_t low_b = have_b ? *in++ : 0;
        const uint8_t low_c = have_c ? *in++ : 0;

        const uint8_t a = (uint8_t)((high << 2) & 0xC0u) | low_a;
        const uint8_t b = (uint8_t)((high << 4) & 0xC0u) | low_b;
        const uint8_t c = (uint8_t)((high << 6) & 0xC0u) | low_c;

        *out++ = a;

        if (have_b) {
            *out++ = b;
        }

        if (have_c) {
            *out++ = c;
        }
    }
}

static void sony_checksum_encode(const uint8_t *in, const size_t in_size,
                                 uint8_t *out, const size_t out_size,
                                 uint8_t *sum_out)
{
    assert(in_size == out_size);

    uint16_t sum_a = 0, sum_b = 0, sum_c = 0;

    for (size_t i = 0; i < in_size; i += 3) {
        const bool have_b = (i + 1) < in_size;
        const bool have_c = (i + 2) < in_size;

        const uint8_t a = *in++;
        const uint8_t b = have_b ? *in++ : 0;
        const uint8_t c = have_c ? *in++ : 0;

        /* Per-group left-rotate on sum_c. */
        sum_c = (uint16_t)((sum_c & 0x00FFu) << 1);
        if (sum_c >= 0x0100u) {
            sum_c++; /* insert 1 on carry */
        }

        /* Add to sum_a; carry from sum_c bumps sum_a. */
        sum_a += a;
        if (sum_c >= 0x0100u) {
            sum_a++;
            sum_c &= 0x00FFu;
        }
        const uint8_t a2 = a ^ (uint8_t)sum_c;

        /* Add to sum_b; carry from sum_a bumps sum_b. */
        if (have_b) {
            sum_b += b;
            if (sum_a >= 0x0100u) {
                sum_b++;
                sum_a &= 0x00FFu;
            }
        }
        const uint8_t b2 = b ^ (uint8_t)sum_a;

        /* Add to sum_c; carry from sum_b bumps sum_c. */
        if (have_c) {
            sum_c += c;
            if (sum_b >= 0x0100u) {
                sum_c++;
                sum_b &= 0x00FFu;
            }
        }
        const uint8_t c2 = c ^ (uint8_t)sum_b;

        *out++ = a2;

        if (have_b) {
            *out++ = b2;
        }

        if (have_c) {
            *out++ = c2;
        }
    }

    *sum_out++ = (uint8_t)sum_a;
    *sum_out++ = (uint8_t)sum_b;
    *sum_out++ = (uint8_t)sum_c;
}

static bool sony_checksum_decode(const uint8_t *in, const size_t in_size,
                                 uint8_t *out, const size_t out_size,
                                 const uint8_t *const sum)
{
    assert(in_size == out_size);

    uint16_t sum_a = 0, sum_b = 0, sum_c = 0;

    for (size_t i = 0; i < in_size; i += 3) {
        const bool have_b = (i + 1) < in_size;
        const bool have_c = (i + 2) < in_size;

        const uint8_t a2 = *in++;
        const uint8_t b2 = have_b ? *in++ : 0;
        const uint8_t c2 = have_c ? *in++ : 0;

        /* Per-group left-rotate on sum_c. */
        sum_c = (uint16_t)((sum_c & 0x00FFu) << 1);
        if (sum_c >= 0x0100u) {
            sum_c++; /* insert 1 on carry */
        }

        /* Add to sum_a; carry from sum_c bumps sum_a. */
        const uint8_t a = a2 ^ (uint8_t)sum_c;
        sum_a += a;
        if (sum_c >= 0x0100u) {
            sum_a++;
            sum_c &= 0x00FFu;
        }

        /* Add to sum_b; carry from sum_a bumps sum_b. */
        const uint8_t b = b2 ^ (uint8_t)sum_a;
        if (have_b) {
            sum_b += b;
            if (sum_a >= 0x0100u) {
                sum_b++;
                sum_a &= 0x00FFu;
            }
        }

        /* Add to sum_c; carry from sum_b bumps sum_c. */
        const uint8_t c = c2 ^ (uint8_t)sum_b;
        if (have_c) {
            sum_c += c;
            if (sum_b >= 0x0100u) {
                sum_c++;
                sum_b &= 0x00FFu;
            }
        }

        *out++ = a;

        if (have_b) {
            *out++ = b;
        }

        if (have_c) {
            *out++ = c;
        }
    }

    if (sum) {
        return sum[0] == (uint8_t)sum_a &&
               sum[1] == (uint8_t)sum_b &&
               sum[2] == (uint8_t)sum_c;
    } else {
        return false;
    }
}

static uint64_t sony_lba(SonyDrive *const drive, const uint8_t track,
                         const uint8_t head, const uint8_t sector)
{
    const uint8_t heads = drive->double_sided ? 2 : 1;

    if (drive->sectors_per_track) {
        return ((uint64_t)track * heads + head)
                * drive->sectors_per_track + sector;
    } else {
        uint64_t lba = 0;

        for (uint8_t i = 0; i < track; i++) {
            const uint8_t sectors = sony_drive_sectors_for_track(drive, i);
            assert(sectors > 0);
            lba += sectors * heads;
        }

        const uint8_t sectors = sony_drive_sectors_for_track(drive, track);
        assert(sectors > 0);
        lba += sectors * head;

        lba += sector;

        return lba;
    }
}

/*
 * Disks are emulated a track at a time, the way the head sees them: each
 * side of the current track is a ring of disk bytes (with the MFM sync
 * marks flagged), and reads and writes move one head position around it.
 * That covers the Mac's two ways of writing: one sector's data field after
 * its address field, and a whole track at once when it formats a disk.
 * When the head leaves the track (step, motor off, eject), every sector
 * with a good address field, data field and checksum or CRC goes back
 * into the image.
 *
 * A ring is as long as a real track: GCR at 2 us bit cells and 394 to 590
 * rpm by zone (self-sync bytes count as one byte here), MFM at 300 rpm
 * and 500 kbit/s (1.44 MB) or 250 kbit/s (720K). MFM tracks have an index
 * pulse where the ring starts, which the Mac waits for when it formats one.
 */
static const uint16_t sony_gcr_track_len[] = { 9520, 8740, 7944, 7142, 6356 };
#define SONY_MFM_TRACK_LEN      12500   /* high density */
#define SONY_MFM_DD_TRACK_LEN   6250
#define SONY_MFM_INDEX_LEN      100     /* about 1.6 ms */
#define SONY_MFM_GAP3           84

static uint16_t sony_track_capacity(const SonyDrive *const drive,
                                    const uint8_t track)
{
    const unsigned zone = MIN(track / 16, ARRAY_SIZE(sony_gcr_track_len) - 1);

    QEMU_BUILD_BUG_ON(SONY_TRACK_MAX < 9520 ||
                      SONY_TRACK_MAX < SONY_MFM_TRACK_LEN);
    if (drive->gcr_encode) {
        return sony_gcr_track_len[zone];
    }
    return drive->hd_media ? SONY_MFM_TRACK_LEN : SONY_MFM_DD_TRACK_LEN;
}

static uint8_t sony_gcr_format_byte(const SonyDrive *const drive)
{
    return drive->double_sided ? 0x22 : 0x02;
}

/* One sector as it sits on the disk: address field, then data field */
static int sony_gcr_build_sector(SonyDrive *const drive, const uint8_t track,
                                 const uint8_t head, const uint8_t sector,
                                 uint8_t *const out)
{
    const uint64_t lba = sony_lba(drive, track, head, sector);
    const uint8_t format_byte = sony_gcr_format_byte(drive);

    uint8_t header_vals[5];
    header_vals[0] = track & 0x3f;
    header_vals[1] = sector;
    header_vals[2] = (head << 5) | ((track >> 6) & 0x1f);
    header_vals[3] = format_byte;
    header_vals[4] = header_vals[0] ^ header_vals[1] ^
                     header_vals[2] ^ header_vals[3];

    /* 12 tag bytes (images have none: made up like a fresh disk), data */
    uint8_t sector_data[SONY_GCR_SECTOR_LENGTH] = {
        track, head, sector, format_byte, 0, 0, (lba >> 8) & 0xff, lba & 0xff,
    };
    if (blk_pread(drive->blk, lba * 512, SONY_GCR_DATA_LENGTH,
                  sector_data + SONY_GCR_TAG_LENGTH, 0) < 0) {
        return -1;
    }

    size_t idx = 0;

    /* sync, then the address field */
    memset(out + idx, 0xff, SONY_GCR_SYNC_LEN);
    idx += SONY_GCR_SYNC_LEN;
    out[idx++] = 0xd5;
    out[idx++] = 0xaa;
    out[idx++] = 0x96;
    sony_gcr_encode_bytes(header_vals, out + idx, sizeof header_vals);
    idx += sizeof header_vals;
    out[idx++] = 0xde;
    out[idx++] = 0xaa;

    /* sync, then the data field (the Mac rewrites from here on) */
    memset(out + idx, 0xff, SONY_GCR_SYNC_LEN);
    idx += SONY_GCR_SYNC_LEN;
    out[idx++] = 0xd5;
    out[idx++] = 0xaa;
    out[idx++] = 0xad;
    out[idx++] = sony_gcr_encode_byte(sector);

    uint8_t sector_checksum[3];
    sony_checksum_encode(sector_data, sizeof sector_data,
                         sector_data, sizeof sector_data, sector_checksum);
    sony_6and2_pack(sector_data, sizeof sector_data,
                    out + idx, SONY_GCR_ENCODED_LENGTH);
    sony_gcr_encode_bytes(out + idx, out + idx, SONY_GCR_ENCODED_LENGTH);
    idx += SONY_GCR_ENCODED_LENGTH;

    sony_6and2_pack(sector_checksum, sizeof sector_checksum, out + idx, 4);
    sony_gcr_encode_bytes(out + idx, out + idx, 4);
    idx += 4;

    out[idx++] = 0xde;
    out[idx++] = 0xaa;
    return idx;
}

/* Has this side of the disk any sectors? (not side 2 of a 400K disk) */
static bool sony_side_exists(const SonyDrive *const drive, const uint8_t head)
{
    return head == 0 || drive->double_sided;
}

/*
 * The Mac formats a double-density disk one- or two-sided as the user
 * chooses, and a real disk has two sides either way. A 400K image has
 * room for one: when the Mac starts writing the second side, make the
 * image an 800K one. Only the format does that, so what the rest of the
 * file held does not matter.
 */
static bool sony_gcr_grow_to_two_sides(SonyDrive *const drive)
{
    Error *err = NULL;
    uint64_t perm, shared;
    int ret;

    if (drive->double_sided || drive->total_sectors != 800) {
        return drive->double_sided;
    }

    /* the resize permission only for as long as it takes */
    blk_get_perm(drive->blk, &perm, &shared);
    ret = blk_set_perm(drive->blk, perm | BLK_PERM_RESIZE, shared, &err);
    if (ret == 0) {
        ret = blk_truncate(drive->blk, 1600 * 512, false, PREALLOC_MODE_OFF,
                           0, &err);
        blk_set_perm(drive->blk, perm, shared, &error_abort);
    }
    if (ret < 0) {
        error_reportf_err(err, "floppy: cannot make the 400K image 800K: ");
        return false;
    }
    drive->double_sided = true;
    drive->total_sectors = 1600;
    trace_sony_drive_gcr_grow(drive);
    return true;
}

/* Put the good GCR sectors of one side of the ring back into the image */
static void sony_gcr_flush_side(SonyDrive *const drive, const uint8_t head)
{
    const uint8_t track = drive->track_cyl;
    const uint8_t sectors = sony_drive_sectors_for_track(drive, track);
    const unsigned len = drive->track_len;
    g_autofree uint8_t *r = g_malloc(len * 2);
    unsigned written = 0, bad = 0;

    /* twice over, so fields that wrap around the ring read straight */
    memcpy(r, drive->track_buf[head], len);
    memcpy(r + len, drive->track_buf[head], len);

    for (unsigned i = 0; i < len; i++) {
        uint8_t hdr[5], sector_data[SONY_GCR_SECTOR_LENGTH];
        uint8_t enc[SONY_GCR_ENCODED_LENGTH], sum_enc[4], sum[3];
        unsigned j, end;

        if (r[i] != 0xd5 || r[i + 1] != 0xaa || r[i + 2] != 0x96 ||
            !sony_gcr_decode_bytes(r + i + 3, hdr, sizeof hdr) ||
            (hdr[0] ^ hdr[1] ^ hdr[2] ^ hdr[3]) != hdr[4]) {
            continue;
        }

        const uint8_t h_track = (hdr[0] & 0x3f) | ((hdr[2] & 0x1f) << 6);
        const uint8_t h_side = (hdr[2] >> 5) & 1;
        const uint8_t h_sector = hdr[1];

        if (h_track != track || h_side != head || h_sector >= sectors) {
            continue;
        }

        /* the data field follows within a few sync bytes */
        end = MIN(i + 10 + 48, 2 * len - (4 + SONY_GCR_ENCODED_LENGTH + 4));
        for (j = i + 10; j < end; j++) {
            if (r[j] == 0xd5 && r[j + 1] == 0xaa && r[j + 2] == 0xad) {
                break;
            }
        }
        if (j >= end || sony_gcr_decode_byte(r[j + 3]) != h_sector ||
            !sony_gcr_decode_bytes(r + j + 4, enc, sizeof enc) ||
            !sony_gcr_decode_bytes(r + j + 4 + sizeof enc, sum_enc, 4)) {
            bad++;
            continue;
        }
        sony_6and2_unpack(enc, sizeof enc, sector_data, sizeof sector_data);
        sony_6and2_unpack(sum_enc, sizeof sum_enc, sum, sizeof sum);
        if (!sony_checksum_decode(sector_data, sizeof sector_data,
                                  sector_data, sizeof sector_data, sum)) {
            bad++;
            continue;
        }

        const uint64_t lba = sony_lba(drive, track, head, h_sector);
        if (lba < drive->total_sectors &&
            blk_pwrite(drive->blk, lba * 512, SONY_GCR_DATA_LENGTH,
                       &sector_data[SONY_GCR_TAG_LENGTH], 0) >= 0) {
            written++;
        }
    }
    trace_sony_drive_track_flush(drive, track, head, written, bad);
}

static uint16_t sony_crc16(uint16_t crc, const uint8_t *p, size_t len)
{
    while (len--) {
        crc ^= *p++ << 8;
        for (int i = 0; i < 8; i++) {
            crc = crc & 0x8000 ? (crc << 1) ^ 0x1021 : crc << 1;
        }
    }
    return crc;
}

/* Put bytes on an MFM ring; @marks: the first @marks bytes are sync marks */
static unsigned sony_mfm_put(SonyDrive *const drive, const uint8_t head,
                             unsigned idx, const uint8_t *p, size_t len,
                             unsigned marks)
{
    for (size_t i = 0; i < len; i++, idx++) {
        drive->track_buf[head][idx] = p[i];
        bitset_set(drive->track_mark[head], idx, i < marks);
    }
    return idx;
}

static unsigned sony_mfm_fill(SonyDrive *const drive, const uint8_t head,
                              unsigned idx, const uint8_t value, size_t len)
{
    memset(drive->track_buf[head] + idx, value, len);
    bitset_set_range(drive->track_mark[head], idx, len, false);
    return idx + len;
}

/* One side of an MFM track, IBM System 34 layout, sectors in order */
static bool sony_mfm_build_side(SonyDrive *const drive, const uint8_t track,
                                const uint8_t head)
{
    static const uint8_t iam[] = { 0xc2, 0xc2, 0xc2, 0xfc };
    unsigned idx = 0;

    idx = sony_mfm_fill(drive, head, idx, 0x4e, 80);
    idx = sony_mfm_fill(drive, head, idx, 0x00, 12);
    idx = sony_mfm_put(drive, head, idx, iam, sizeof iam, 3);
    idx = sony_mfm_fill(drive, head, idx, 0x4e, 50);

    for (uint8_t s = 0; s < drive->sectors_per_track; s++) {
        uint8_t id[10] = { 0xa1, 0xa1, 0xa1, 0xfe, track, head, s + 1, 2 };
        uint8_t data[4 + 512 + 2] = { 0xa1, 0xa1, 0xa1, 0xfb };
        const uint64_t lba = sony_lba(drive, track, head, s);
        uint16_t crc;

        crc = sony_crc16(0xffff, id, 8);
        id[8] = crc >> 8;
        id[9] = crc;
        if (blk_pread(drive->blk, lba * 512, 512, data + 4, 0) < 0) {
            return false;
        }
        crc = sony_crc16(0xffff, data, 4 + 512);
        data[516] = crc >> 8;
        data[517] = crc;

        idx = sony_mfm_fill(drive, head, idx, 0x00, 12);
        idx = sony_mfm_put(drive, head, idx, id, sizeof id, 3);
        idx = sony_mfm_fill(drive, head, idx, 0x4e, 22);
        idx = sony_mfm_fill(drive, head, idx, 0x00, 12);
        idx = sony_mfm_put(drive, head, idx, data, sizeof data, 3);
        idx = sony_mfm_fill(drive, head, idx, 0x4e, SONY_MFM_GAP3);
    }
    assert(idx <= drive->track_len);
    sony_mfm_fill(drive, head, idx, 0x4e, drive->track_len - idx);
    return true;
}

/* Is there a sync mark (three A1s) and then @am at ring offset @i? */
static bool sony_mfm_am_at(const uint8_t *r, const uint8_t *m, unsigned len,
                           unsigned i, uint8_t am)
{
    for (unsigned k = 0; k < 3; k++) {
        if (r[i + k] != 0xa1 || !bitset_get(m, (i + k) % len)) {
            return false;
        }
    }
    return r[i + 3] == am;
}

/* Put the good MFM sectors of one side of the ring back into the image */
static void sony_mfm_flush_side(SonyDrive *const drive, const uint8_t head)
{
    const uint8_t track = drive->track_cyl;
    const unsigned len = drive->track_len;
    const uint8_t *const m = drive->track_mark[head];
    g_autofree uint8_t *r = g_malloc(len * 2);
    unsigned written = 0, bad = 0;

    memcpy(r, drive->track_buf[head], len);
    memcpy(r + len, drive->track_buf[head], len);

    for (unsigned i = 0; i < len; i++) {
        unsigned j, end;

        if (!sony_mfm_am_at(r, m, len, i, 0xfe) ||
            sony_crc16(0xffff, r + i, 10) != 0) {
            continue;
        }
        const uint8_t c = r[i + 4], h = r[i + 5], sec = r[i + 6], n = r[i + 7];

        if (c != track || h != head || n != 2 || sec < 1 ||
            sec > drive->sectors_per_track) {
            continue;
        }

        /* the data field follows the gap after the ID field */
        end = MIN(i + 10 + 60, 2 * len - (4 + 512 + 2));
        for (j = i + 10; j < end; j++) {
            if (sony_mfm_am_at(r, m, len, j, 0xfb)) {
                break;
            }
        }
        if (j >= end || sony_crc16(0xffff, r + j, 4 + 512 + 2) != 0) {
            bad++;
            continue;
        }

        const uint64_t lba = sony_lba(drive, track, head, sec - 1);
        if (lba < drive->total_sectors &&
            blk_pwrite(drive->blk, lba * 512, 512, r + j + 4, 0) >= 0) {
            written++;
        }
    }
    trace_sony_drive_track_flush(drive, track, head, written, bad);
}

/* Write back the current track if the Mac changed it, and let it go */
static void sony_track_release(SonyDrive *const drive)
{
    if (drive->track_loaded) {
        for (uint8_t head = 0; head < 2; head++) {
            if (drive->track_dirty[head] && drive->disk_in) {
                if (drive->gcr_encode) {
                    sony_gcr_flush_side(drive, head);
                } else {
                    sony_mfm_flush_side(drive, head);
                }
            }
            drive->track_dirty[head] = false;
        }
    }
    drive->track_loaded = false;
}

/* The ring for the track under the head, built from the image if need be */
static bool sony_track_load(SonyDrive *const drive)
{
    const uint8_t track = drive->current_track;

    if (!drive->disk_in) {
        return false;
    }
    if (drive->track_loaded && drive->track_cyl == track) {
        return true;
    }
    sony_track_release(drive);

    const uint16_t len = sony_track_capacity(drive, track);

    drive->track_cyl = track;
    drive->track_len = len;
    drive->track_pos = 0;
    for (uint8_t head = 0; head < 2; head++) {
        uint8_t *const ring = drive->track_buf[head];

        memset(ring, drive->gcr_encode ? 0xff : 0x4e, len);
        memset(drive->track_mark[head], 0, sizeof drive->track_mark[head]);
        if (!sony_side_exists(drive, head)) {
            continue;
        }
        if (drive->gcr_encode) {
            const uint8_t sectors = sony_drive_sectors_for_track(drive, track);
            unsigned idx = 0;

            for (uint8_t s = 0; s < sectors; s++) {
                const int n = sony_gcr_build_sector(drive, track, head, s,
                                                    ring + idx);
                if (n < 0) {
                    return false;
                }
                idx += n + 4;       /* a little gap before the next sector */
            }
            assert(idx <= len);
        } else if (!sony_mfm_build_side(drive, track, head)) {
            return false;
        }
    }
    drive->track_loaded = true;
    trace_sony_drive_track_load(drive, track, len);
    return true;
}

static void sony_drive_transfer_reset(SonyDrive *const drive)
{
    sony_track_release(drive);
}

static uint8_t sony_drive_selected_drive_reg(const SonyDrive *const drive)
{
    uint8_t selector = 0;

    if (drive->phases & 0x4) { /* CA2 */
        selector |= 0x8;
    }
    if (drive->phases & 0x2) { /* CA1 */
        selector |= 0x4;
    }
    if (drive->phases & 0x1) { /* CA0 */
        selector |= 0x2;
    }
    if (drive->sel) {
        selector |= 0x1;
    }

    return selector;
}

static void sony_drive_step(SonyDrive *const drive)
{
    if (drive->seek_direction < 0) {
        if (drive->current_track > 0) {
            drive->current_track--;
        }
    } else {
        if (!drive->cylinders || drive->current_track + 1 < drive->cylinders) {
            drive->current_track++;
        }
    }

    sony_drive_transfer_reset(drive);
    trace_sony_drive_step(drive, drive->seek_direction, drive->current_track);
}

static void sony_drive_strobe_drive(SonyDrive *const drive)
{
    const uint8_t selector = sony_drive_selected_drive_reg(drive);
    trace_sony_drive_strobe(drive, selector, sony_drive_cmd_names[selector]);

    switch (selector) {
    case SONY_CMD_TRACK_POS:
        drive->seek_direction = 1;
        break;
    case SONY_CMD_TRACK_NEG:
        drive->seek_direction = -1;
        break;
    case SONY_CMD_TRACK_STEP:
        sony_drive_step(drive);
        break;
    case SONY_CMD_SET_MFM:
        drive->use_gcr = false;
        break;
    case SONY_CMD_SET_GCR:
        drive->use_gcr = true;
        break;
    case SONY_CMD_MOTOR_ON:
        drive->motor_on = true;
        break;
    case SONY_CMD_MOTOR_OFF:
        drive->motor_on = false;
        sony_drive_transfer_reset(drive);
        break;
    case SONY_CMD_EJECT:
        sony_drive_transfer_reset(drive);
        blk_eject(drive->blk, true);
        drive->disk_in = false;
        sony_drive_reset(drive);
        break;
    default:
        break;
    }
}

bool sony_drive_read_sense(SonyDrive *const drive)
{
    const uint8_t reg = sony_drive_selected_drive_reg(drive);
    bool value;

    switch (reg) {
    case SONY_REG_DIRTN:
        /* "Direction". */
        value = drive->seek_direction < 0;
        break;
    case SONY_REG_CSTIN:
        /* "Cassette in". */
        value = !drive->disk_in; /* sense inverted */
        break;
    case SONY_REG_STEP:
        value = true; /* not emulated; our seeks are instantaneous */
        break;
    case SONY_REG_WRTPRT:
        value = !drive->write_protected; /* sense inverted */
        break;
    case SONY_REG_MOTORON:
        value = !drive->motor_on; /* sense inverted */
        break;
    case SONY_REG_TKO:
        /* "Track zero". */
        value = drive->current_track != 0; /* sense inverted */
        break;
    case SONY_REG_EJECT_L:
        /* Not pressing eject switch. */
        value = false;
        break;
    case SONY_REG_TACH:
        /* Tacho functionality not emulated. */
        value = !drive->motor_on;
        break;
    case SONY_REG_RDDATA0:
    case SONY_REG_RDDATA1:
        /*
         * Single-bit reads are not emulated, but on MFM disks this is the
         * index pulse (low), at the start of the ring. The disk turns
         * while the Mac polls it, a byte a poll, unless it moves bytes.
         */
        value = true;
        if (!drive->gcr_encode && drive->motor_on && sony_track_load(drive)) {
            if (!drive->track_moved) {
                drive->track_pos = (drive->track_pos + 1) % drive->track_len;
            }
            drive->track_moved = false;
            value = drive->track_pos >= SONY_MFM_INDEX_LEN;
        }
        break;
    case SONY_REG_SUPERDRIVE:
        value = true;
        break;
    case SONY_REG_GCR:
        value = !drive->use_gcr; /* sense inverted */
        break;
    case SONY_REG_SINGLE_SIDE:
        /* We are a double-sided drive. */
        value = true; /* sense inverted */
        break;
    case SONY_REG_READY:
        value = !(drive->disk_in && drive->motor_on); /* sense inverted */
        break;
    case SONY_REG_INSTALLED:
        value = false; /* sense inverted */
        break;
    case SONY_REG_TWOMEG:
        /* "Two-meg" refers to the unformatted size of HD MFM floppies. */
        value = !(drive->hd_media && drive->disk_in); /* sense inverted */
        break;
    default:
        value = true;
        break;
    }

    trace_sony_drive_read_sense(drive, reg, sony_drive_reg_names[reg], value);
    return value;
}

void sony_drive_set_inputs(SonyDrive *const drive, const uint8_t phases,
                           const bool sel, const bool enabled)
{
    trace_sony_drive_set_inputs(drive, phases, sel, enabled);
    const bool prev_strobe = (drive->phases & 0x8) != 0 && drive->enabled;
    const bool cur_strobe = (phases & 0x8) != 0 && enabled;

    drive->phases = phases;
    drive->sel = sel;
    drive->enabled = enabled;

    if (!prev_strobe && cur_strobe) {
        sony_drive_strobe_drive(drive);
    }
}

bool sony_drive_read_byte(SonyDrive *const drive,
                          uint8_t *const value_out, bool *const is_mark_out)
{
    const uint8_t head = drive->sel ? 1 : 0;
    const unsigned pos = drive->track_pos;

    if (!sony_track_load(drive)) {
        return false;
    }
    uint8_t value = drive->track_buf[head][pos];
    const bool is_mark = bitset_get(drive->track_mark[head], pos);

    drive->track_pos = (pos + 1) % drive->track_len;
    drive->track_moved = true;
    if (drive->gcr_encode && !(value & 0x80)) {
        /* a self-sync pattern the CPU wrote (FF 3F CF ...) reads as FF */
        value = 0xff;
    }
    *value_out = value;
    *is_mark_out = is_mark;
    trace_sony_drive_read_byte(drive, value, is_mark, drive->track_cyl, pos);
    return true;
}

bool sony_drive_write_byte(SonyDrive *const drive, const uint8_t value,
                           const bool is_mark)
{
    const uint8_t head = drive->sel ? 1 : 0;

    if (drive->write_protected || !sony_track_load(drive)) {
        return false;
    }
    if (!sony_side_exists(drive, head) &&
        !(drive->gcr_encode && sony_gcr_grow_to_two_sides(drive))) {
        return false;
    }

    const unsigned pos = drive->track_pos;

    drive->track_buf[head][pos] = value;
    bitset_set(drive->track_mark[head], pos, is_mark);
    drive->track_pos = (pos + 1) % drive->track_len;
    drive->track_moved = true;
    drive->track_dirty[head] = true;
    trace_sony_drive_write_byte(drive, value, drive->track_cyl, pos);
    return true;
}

void sony_drive_set_block_backend(SonyDrive *const drive,
                                  BlockBackend *const block)
{
    sony_drive_transfer_reset(drive);
    drive->blk = block;
    drive->disk_in = blk_is_inserted(block);
    sony_drive_reset(drive);
}

void sony_drive_reset(SonyDrive *const drive)
{
    sony_drive_transfer_reset(drive);

    drive->motor_on = false;
    drive->write_protected = false;
    drive->hd_media = false;
    drive->double_sided = false;
    drive->use_gcr = true;
    drive->gcr_encode = true;
    drive->cylinders = 0;
    drive->sectors_per_track = 0;
    drive->total_sectors = 0;
    drive->current_track = 0;
    drive->seek_direction = 1;

    if (drive->disk_in) {
        const int64_t length = blk_getlength(drive->blk);

        if (length <= 0 || length % 512 != 0) {
            warn_report_once("%s: unsupported floppy size %" PRId64,
                             __func__, length);
            drive->disk_in = false;
        } else {
            drive->write_protected = !blk_is_writable(drive->blk);
            drive->total_sectors = length / 512;

            if (drive->total_sectors == 1600) {
                /* 800 kB (double-sided GCR). */
                drive->cylinders = 80;
                drive->double_sided = true;
                drive->gcr_encode = true;
            } else if (drive->total_sectors == 800) {
                /* 400 kB (single-sided GCR). */
                drive->cylinders = 80;
                drive->double_sided = false;
                drive->gcr_encode = true;
            } else if (drive->total_sectors == 1440) {
                /* 720 kB (double-sided, double-density MFM). */
                drive->cylinders = 80;
                drive->sectors_per_track = 9;
                drive->double_sided = true;
                drive->gcr_encode = false;
            } else if (drive->total_sectors == 2880) {
                /* 1440 kB (double-sided MFM). */
                drive->cylinders = 80;
                drive->sectors_per_track = 18;
                drive->hd_media = true;
                drive->double_sided = true;
                drive->gcr_encode = false;
            } else {
                warn_report_once("%s: unable to determine geometry for media"
                                 " (%" PRId64 " bytes)", __func__, length);
                drive->disk_in = false;
            }
        }
    }
}

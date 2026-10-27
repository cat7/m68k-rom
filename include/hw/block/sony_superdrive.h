/*
 * QEMU Macintosh SuperDrive floppy disk drive emulator
 *
 * Copyright (c) 2025 Matt Jacobson <mhjacobson@me.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_BLOCK_SWIM_SONY_H
#define HW_BLOCK_SWIM_SONY_H

#include <stdbool.h>
#include <stdint.h>
#include "hw/block/block.h"

/* bytes on the longest track (MFM high density, 500 kbit/s at 300 rpm) */
#define SONY_TRACK_MAX      12500

typedef struct {
    BlockBackend *blk;

    uint8_t phases;
    bool sel;
    bool enabled;

    bool motor_on;
    bool disk_in;
    bool write_protected;
    bool hd_media;
    bool double_sided;
    bool use_gcr;
    bool gcr_encode;
    uint8_t cylinders;
    uint8_t sectors_per_track;
    uint8_t current_track;
    int8_t seek_direction;
    uint32_t total_sectors;

    /* the track under the head: one ring of disk bytes a side */
    bool track_loaded;
    bool track_dirty[2];
    bool track_moved;           /* bytes moved since the last index poll */
    uint8_t track_cyl;
    uint16_t track_len;
    uint16_t track_pos;
    uint8_t track_buf[2][SONY_TRACK_MAX];
    uint8_t track_mark[2][SONY_TRACK_MAX / 8 + 1];  /* MFM sync marks */
} SonyDrive;

void sony_drive_set_block_backend(SonyDrive *drive, BlockBackend *block);
void sony_drive_reset(SonyDrive *drive);
bool sony_drive_read_sense(SonyDrive *drive);
void sony_drive_set_inputs(SonyDrive *drive, uint8_t phases,
                           bool sel, bool enabled);
bool sony_drive_read_byte(SonyDrive *drive,
                          uint8_t *value_out, bool *is_mark_out);
bool sony_drive_write_byte(SonyDrive *const drive, const uint8_t value,
                           const bool is_mark);

#endif /* HW_BLOCK_SWIM_SONY_H */

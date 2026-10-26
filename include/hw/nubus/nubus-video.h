/*
 * QEMU NuBus video card
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_NUBUS_NUBUS_VIDEO_H
#define HW_NUBUS_NUBUS_VIDEO_H

#include "hw/nubus/nubus.h"
#include "ui/console.h"
#include "qom/object.h"

#define TYPE_NUBUS_VIDEO "nubus-video"
OBJECT_DECLARE_SIMPLE_TYPE(NubusVideoState, NUBUS_VIDEO)

/* Depths: 1, 2, 4, 8 bpp indexed; 16 (x555) and 32 (x888) bpp direct */
#define NUBUS_VIDEO_NDEPTHS     6

struct NubusVideoState {
    NubusDevice parent_obj;

    MemoryRegion vram;
    MemoryRegion vram_low;      /* the VRAM part of the 24-bit window */
    MemoryRegion low_window;    /* first MB: VRAM, registers, ROM */
    MemoryRegion low_mirror[15];    /* copies of it in the free MBs */
    MemoryRegion regs;
    MemoryRegion rom;

    QemuConsole *con;
    QEMUTimer *vbl_timer;
    uint8_t *vram_ptr;

    /* properties */
    uint32_t width;
    uint32_t height;
    uint32_t vram_size;

    int ndepths;
    bool invalidate;

    /* registers */
    uint32_t mode;
    uint32_t int_enable;
    uint32_t int_status;
    uint32_t clut_index;
    uint32_t clut[256];
};

#endif

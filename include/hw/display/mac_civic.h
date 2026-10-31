/*
 * Apple Civic, the built-in video of the Quadra 660AV and 840AV, with its
 * Sebastian CLUT/DAC
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_DISPLAY_MAC_CIVIC_H
#define HW_DISPLAY_MAC_CIVIC_H

#include "hw/core/sysbus.h"
#include "ui/console.h"
#include "qom/object.h"

#define TYPE_MAC_CIVIC "mac-civic"
OBJECT_DECLARE_SIMPLE_TYPE(MacCivicState, MAC_CIVIC)

/* sysbus MMIO regions */
enum {
    MAC_CIVIC_MMIO_REGS,        /* $50036000 */
    MAC_CIVIC_MMIO_VRAM,        /* $50100000, 2 MB window */
    MAC_CIVIC_MMIO_CLOCK,       /* $50F2E000 */
    MAC_CIVIC_MMIO_DAC,         /* $50F30800 */
};

#define MAC_CIVIC_REG_BITS      (0x700 / 4)

struct MacCivicState {
    SysBusDevice parent_obj;

    MemoryRegion regs_mmio;
    MemoryRegion vram;
    MemoryRegion vram_window;
    MemoryRegion vram_bank_b;     /* with 1 MB: its second half */
    MemoryRegion vram_open_bus;
    MemoryRegion clock_mmio;
    MemoryRegion dac_mmio;
    QemuConsole *con;
    QEMUTimer *vbl_timer;
    qemu_irq vbl_irq;

    uint32_t vram_size;
    uint32_t width, height;     /* picks the monitor */
    uint8_t monitor;

    uint8_t bits[MAC_CIVIC_REG_BITS];  /* one register bit per longword */
    bool vbl;
    int64_t frame_start_ns;

    uint8_t dac_index;
    uint8_t dac_step;
    uint8_t dac_ctrl;
    uint8_t clut[256 * 3];
    uint8_t video_table[256 * 3];  /* for the video input */
    uint8_t clock_regs[4];
    bool invalidate;

    /* video input */
    char *video_in_name;
    int video_in;
    struct HostCam *cam;
    struct SAA7191State *decoder;
    struct SAA7186State *scaler;
    QEMUTimer *video_timer;
    bool vdc_pending;           /* a frame is done (VDCInt) */
    bool video_fresh;           /* a new frame to show */
    uint32_t video_frames;
    uint32_t *video_src;        /* the host's picture, scaled */
    unsigned video_w, video_h;
    uint8_t clut_key[256];      /* the CLUT's 4th bytes: key colours */
};

#endif

/*
 * Built-in video of the Macintosh LC family: the V8 chip (LC, LC II) and
 * Sonora (LC III), both with Apple's Ariel CLUT/DAC
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_DISPLAY_MAC_LC_VIDEO_H
#define HW_DISPLAY_MAC_LC_VIDEO_H

#include "hw/core/sysbus.h"
#include "ui/console.h"
#include "qom/object.h"

#define TYPE_MAC_LC_VIDEO "mac-lc-video"
OBJECT_DECLARE_SIMPLE_TYPE(MacLCVideoState, MAC_LC_VIDEO)

/* sysbus MMIO regions */
enum {
    MAC_LC_VIDEO_MMIO_VRAM,     /* Sonora $60000000, V8 $50F40000 */
    MAC_LC_VIDEO_MMIO_DAC,      /* $50F24000 */
    MAC_LC_VIDEO_MMIO_CTRL,     /* Sonora only: $50F28000 */
};

/*
 * GPIO inputs (V8 only), named "depth": lines 0-2 are the depth bits of
 * the pseudo-VIA's video register ($10)
 */

/* the monitors it can sense */
enum {
    MAC_LC_MONITOR_12_RGB,      /* 512 x 384 */
    MAC_LC_MONITOR_13_RGB,      /* 640 x 480 */
    MAC_LC_MONITOR_PORTRAIT,    /* 640 x 870, monochrome */
    MAC_LC_MONITOR_16_RGB,      /* 832 x 624, Sonora only */
    MAC_LC_MONITOR_VGA,         /* 640 x 480, Sonora only */
    MAC_LC_MONITOR_NB,
};

struct MacLCVideoState {
    SysBusDevice parent_obj;

    MemoryRegion vram;
    MemoryRegion dac_mmio;
    MemoryRegion ctrl_mmio;
    QemuConsole *con;
    QEMUTimer *vbl_timer;
    qemu_irq vbl_irq;

    bool v8;                        /* V8: no timing register, 1 KB rows */
    uint32_t vram_size;
    uint32_t width, height;         /* picks the monitor */
    uint8_t monitor;

    uint8_t mode;                   /* bit 7 blank, bits 0-4 timing */
    uint8_t depth;                  /* 0-4: 1, 2, 4, 8, 16 bpp */
    uint8_t sense_drive;            /* bit 3 clear: drive bits 0-2 */
    uint8_t vtest;
    uint8_t dac_index;
    uint8_t dac_step;
    uint8_t dac_control;
    uint8_t dac_key;
    uint8_t clut[256 * 3];         /* red, green, blue */
    bool vbl;
    bool invalidate;
};

/* V8: the monitor code the pseudo-VIA's video register reads (bits 3-5) */
uint8_t mac_lc_video_v8_monitor_code(MacLCVideoState *s);

#endif

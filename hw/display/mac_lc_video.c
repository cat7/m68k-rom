/*
 * Built-in video of the Macintosh LC family
 *
 * Sonora (LC III) reads its VRAM ($60000000; 512 KB, or 768 KB with the VRAM
 * SIMM) as packed pixels, rows following each other with no padding, and
 * sends them through a 256-entry CLUT, except in 16 bpp, which bypasses
 * it. Its registers, a byte each:
 *
 *   control ($50F28000)            DAC ($50F24000)
 *   0  timing: bit 7 blank,        0  CLUT address
 *      bits 0-4 the monitor's      1  CLUT data, red, green, blue
 *      timing ($02 12" RGB, $06    2  control
 *      13" RGB, $01 15" portrait,  3  colour key
 *      $09 16" RGB, $0B VGA)
 *   1  depth 0-4: 1-16 bpp
 *   2  sense: bits 0-3 written, bit 3 clear drives the sense lines
 *      with bits 0-2; bits 4-6 read the lines
 *   3  test
 *   4-7  beam position
 *
 * Monitor sense follows Apple's scheme: lines A, B, C (bits 2, 1, 0)
 * are pulled up; a monitor grounds some of them or, when it grounds
 * none, ties two of them together (extended sense).
 *
 * V8 (LC, LC II) has the same Ariel DAC but no control registers: the
 * monitor is one of three, which the pseudo-VIA's video register ($10)
 * reads as a code in bits 3-5 (1 15" portrait, 2 12" RGB, 6 13" RGB), and
 * the depth is written to its bits 0-2. Its VRAM (256 KB, or 512 KB with
 * the VRAM SIMM) is at $50F40000; a row takes 512 bytes with 256 KB and
 * 1 KB with 512 KB, whatever the depth. That leaves 640 x 480 at 16
 * colours with 256 KB, and thousands for the 12" RGB with 512 KB.
 *
 * The VBL interrupt goes to the pseudo-VIA as its slot $F line.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "ui/pixel_ops.h"
#include "hw/display/mac_lc_video.h"
#include "trace.h"

#define SONORA_VRAM_WINDOW  (1 * MiB)

#define SENSE_A 4
#define SENSE_B 2
#define SENSE_C 1

typedef struct SonoraMonitor {
    const char *name;
    uint16_t width, height;
    uint8_t timing;             /* what the ROM writes to register 0 */
    uint8_t grounded;           /* sense lines tied to ground */
    uint8_t tied;               /* two lines tied together, or 0 */
    bool mono;                  /* video on the blue gun only */
    uint8_t v8_code;            /* V8 monitor code, 0: not on V8 */
} SonoraMonitor;

static const SonoraMonitor sonora_monitors[MAC_LC_MONITOR_NB] = {
    [MAC_LC_MONITOR_12_RGB] = {
        "12\" RGB", 512, 384, 0x02, SENSE_A | SENSE_C, 0, false, 2
    },
    [MAC_LC_MONITOR_13_RGB] = {
        "13\" RGB", 640, 480, 0x06, SENSE_C, 0, false, 6
    },
    [MAC_LC_MONITOR_PORTRAIT] = {
        "15\" Portrait", 640, 870, 0x01, SENSE_A | SENSE_B, 0, true, 1
    },
    [MAC_LC_MONITOR_16_RGB] = {
        "16\" RGB", 832, 624, 0x09, 0, SENSE_A | SENSE_C
    },
    [MAC_LC_MONITOR_VGA] = {
        "VGA", 640, 480, 0x0b, 0, SENSE_A | SENSE_B
    },
};

static const SonoraMonitor *sv_monitor(MacLCVideoState *s)
{
    return &sonora_monitors[s->monitor];
}

/* the timing the ROM chose, if it is one of the monitors' */
static const SonoraMonitor *sv_timing(MacLCVideoState *s)
{
    int i;

    if (s->v8) {
        return sv_monitor(s);
    }
    for (i = 0; i < MAC_LC_MONITOR_NB; i++) {
        if (sonora_monitors[i].timing == (s->mode & 0x1f)) {
            return &sonora_monitors[i];
        }
    }
    return NULL;
}

static uint8_t sv_sense_lines(MacLCVideoState *s)
{
    const SonoraMonitor *m = sv_monitor(s);
    uint8_t lines = 7 & ~m->grounded;

    if (!(s->sense_drive & 8)) {
        lines &= s->sense_drive & 7;
    }
    if (m->tied && (lines & m->tied) != m->tied) {
        lines &= ~m->tied;
    }
    return lines;
}

/* Display */

static uint32_t sv_clut(MacLCVideoState *s, uint8_t i)
{
    return rgb_to_pixel32(s->clut[i * 3], s->clut[i * 3 + 1],
                          s->clut[i * 3 + 2]);
}

static void sv_draw_line(MacLCVideoState *s, uint32_t *d,
                         const uint8_t *p, int w)
{
    int bpp = 1 << s->depth;
    int x;

    switch (bpp) {
    case 1:
    case 2:
    case 4:
        for (x = 0; x < w; x++) {
            int bit = x * bpp;
            int v = (p[bit >> 3] >> (8 - bpp - (bit & 7))) & ((1 << bpp) - 1);

            /* the index's low bits are ones, as on the real DAC */
            *d++ = sv_clut(s, (v << (8 - bpp)) | (0xff >> bpp));
        }
        break;
    case 8:
        for (x = 0; x < w; x++) {
            *d++ = sv_clut(s, p[x]);
        }
        break;
    case 16:
        for (x = 0; x < w; x++) {
            uint16_t v = lduw_be_p(p + 2 * x);
            uint8_t r = (v >> 10) & 0x1f, g = (v >> 5) & 0x1f, b = v & 0x1f;

            *d++ = rgb_to_pixel32((r << 3) | (r >> 2), (g << 3) | (g >> 2),
                                  (b << 3) | (b >> 2));
        }
        break;
    }
}

static bool sv_update_display(void *opaque)
{
    MacLCVideoState *s = opaque;
    const SonoraMonitor *t = sv_timing(s);
    const SonoraMonitor *m = t ? t : sv_monitor(s);
    DisplaySurface *surface = qemu_console_surface(s->con);
    DirtyBitmapSnapshot *snap;
    uint8_t *vram = memory_region_get_ram_ptr(&s->vram);
    uint32_t rowbytes = s->v8 ? s->vram_size / 512 :
                                (m->width << s->depth) / 8;
    bool on = t && (s->v8 || !(s->mode & 0x80)) && s->depth <= 4 &&
              rowbytes * m->height <= s->vram_size;
    int y, ymin = -1;

    if (surface_width(surface) != m->width ||
        surface_height(surface) != m->height) {
        qemu_console_resize(s->con, m->width, m->height);
        surface = qemu_console_surface(s->con);
        s->invalidate = true;
    }
    if (!on) {
        if (s->invalidate) {
            memset(surface_data(surface), 0,
                   surface_stride(surface) * m->height);
            qemu_console_update(s->con, 0, 0, m->width, m->height);
            s->invalidate = false;
        }
        return true;
    }

    snap = memory_region_snapshot_and_clear_dirty(&s->vram, 0,
                                                  rowbytes * m->height,
                                                  DIRTY_MEMORY_VGA);
    for (y = 0; y < m->height; y++) {
        if (s->invalidate ||
            memory_region_snapshot_get_dirty(&s->vram, snap, y * rowbytes,
                                             rowbytes)) {
            sv_draw_line(s, (uint32_t *)(surface_data(surface) +
                                         y * surface_stride(surface)),
                         vram + y * rowbytes, m->width);
            if (ymin < 0) {
                ymin = y;
            }
        } else if (ymin >= 0) {
            qemu_console_update(s->con, 0, ymin, m->width, y - ymin);
            ymin = -1;
        }
    }
    if (ymin >= 0) {
        qemu_console_update(s->con, 0, ymin, m->width, y - ymin);
    }
    s->invalidate = false;
    g_free(snap);
    return true;
}

static void sv_invalidate_display(void *opaque)
{
    MacLCVideoState *s = opaque;

    s->invalidate = true;
}

static const GraphicHwOps sv_ops = {
    .invalidate = sv_invalidate_display,
    .gfx_update = sv_update_display,
};

/* V8: the depth bits of the pseudo-VIA's video register */
static void sv_set_depth(void *opaque, int n, int level)
{
    MacLCVideoState *s = opaque;
    uint8_t depth = deposit32(s->depth, n, 1, level);

    if (depth != s->depth) {
        s->depth = depth;
        s->invalidate = true;
    }
}

uint8_t mac_lc_video_v8_monitor_code(MacLCVideoState *s)
{
    return sv_monitor(s)->v8_code;
}

/* VBL, 60 times a second */

#define SV_VBL_PERIOD_NS    (NANOSECONDS_PER_SECOND / 60)
#define SV_VBL_LENGTH_NS    (SV_VBL_PERIOD_NS / 20)

static void sv_vbl(void *opaque)
{
    MacLCVideoState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    s->vbl = !s->vbl;
    qemu_set_irq(s->vbl_irq, s->vbl);
    timer_mod(s->vbl_timer, now + (s->vbl ? SV_VBL_LENGTH_NS :
                                   SV_VBL_PERIOD_NS - SV_VBL_LENGTH_NS));
}

/* Registers */

static uint64_t sv_ctrl_read(void *opaque, hwaddr addr, unsigned size)
{
    MacLCVideoState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint32_t line = (now % SV_VBL_PERIOD_NS) * 525 / SV_VBL_PERIOD_NS;
    uint64_t val = 0;

    switch (addr) {
    case 0:
        val = s->mode;
        break;
    case 1:
        val = s->depth;
        break;
    case 2:
        val = (s->sense_drive & 0xf) | (sv_sense_lines(s) << 4);
        break;
    case 3:
        val = s->vtest;
        break;
    case 4:
    case 5:
        /* horizontal position: anywhere */
        val = (now >> 4) & (addr == 4 ? 7 : 0xff);
        break;
    case 6:
        val = (line >> 8) & 3;
        break;
    case 7:
        val = line & 0xff;
        break;
    }
    trace_mac_lc_video_ctrl_read(addr, val);
    return val;
}

static void sv_ctrl_write(void *opaque, hwaddr addr, uint64_t val,
                          unsigned size)
{
    MacLCVideoState *s = opaque;

    trace_mac_lc_video_ctrl_write(addr, val);
    switch (addr) {
    case 0:
        s->mode = val & 0x9f;
        s->invalidate = true;
        break;
    case 1:
        s->depth = val & 7;
        s->invalidate = true;
        break;
    case 2:
        s->sense_drive = val & 0xf;
        break;
    case 3:
        s->vtest = val & 1;
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "sonora video: write 0x%02" PRIx64
                      " to register %" HWADDR_PRIu "\n", val, addr);
        break;
    }
}

static const MemoryRegionOps sv_ctrl_ops = {
    .read = sv_ctrl_read,
    .write = sv_ctrl_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = 1,
    },
};

static uint64_t sv_dac_read(void *opaque, hwaddr addr, unsigned size)
{
    MacLCVideoState *s = opaque;

    switch (addr) {
    case 0:
        return s->dac_index;
    case 1: {
        uint8_t v = s->clut[s->dac_index * 3 + s->dac_step];

        if (++s->dac_step == 3) {
            s->dac_step = 0;
            s->dac_index++;
        }
        return v;
    }
    case 2:
        return s->dac_control;
    case 3:
        return s->dac_key;
    }
    return 0;
}

static void sv_dac_write(void *opaque, hwaddr addr, uint64_t val,
                         unsigned size)
{
    MacLCVideoState *s = opaque;
    uint8_t i;

    switch (addr) {
    case 0:
        s->dac_index = val;
        s->dac_step = 0;
        break;
    case 1:
        i = s->dac_index;
        if (sv_monitor(s)->mono) {
            /* a monochrome monitor shows the blue gun on all three */
            if (s->dac_step == 2) {
                s->clut[i * 3] = s->clut[i * 3 + 1] = s->clut[i * 3 + 2] = val;
            }
        } else {
            s->clut[i * 3 + s->dac_step] = val;
        }
        if (++s->dac_step == 3) {
            s->dac_step = 0;
            s->dac_index++;
        }
        s->invalidate = true;
        break;
    case 2:
        s->dac_control = val;
        break;
    case 3:
        s->dac_key = val;
        break;
    }
}

static const MemoryRegionOps sv_dac_ops = {
    .read = sv_dac_read,
    .write = sv_dac_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = 1,
    },
};

static void sv_reset_hold(Object *obj, ResetType type)
{
    MacLCVideoState *s = MAC_LC_VIDEO(obj);

    s->mode = 0x9f;
    s->depth = 0;
    s->sense_drive = 8;
    s->vtest = 0;
    s->dac_index = 0;
    s->dac_step = 0;
    s->vbl = false;
    s->invalidate = true;
    qemu_set_irq(s->vbl_irq, 0);
    timer_mod(s->vbl_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + SV_VBL_PERIOD_NS);
}

static void sv_realize(DeviceState *dev, Error **errp)
{
    MacLCVideoState *s = MAC_LC_VIDEO(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    int i;

    if (s->v8 ? (s->vram_size != 256 * KiB && s->vram_size != 512 * KiB)
              : (s->vram_size != 512 * KiB && s->vram_size != 768 * KiB)) {
        error_setg(errp, "vram-size must be %s", s->v8 ?
                   "256 KiB or 512 KiB" : "512 KiB or 768 KiB");
        return;
    }
    s->monitor = MAC_LC_MONITOR_13_RGB;
    for (i = 0; i < MAC_LC_MONITOR_NB; i++) {
        if (sonora_monitors[i].width == s->width &&
            sonora_monitors[i].height == s->height) {
            s->monitor = i;
            break;
        }
    }
    if ((i == MAC_LC_MONITOR_NB && s->width) ||
        (s->v8 && !sonora_monitors[s->monitor].v8_code)) {
        error_setg(errp, "no %s monitor has %ux%u: 512x384, 640x480, "
                   "640x870%s", s->v8 ? "V8" : "Sonora", s->width, s->height,
                   s->v8 ? "" : " or 832x624");
        return;
    }

    memory_region_init_ram(&s->vram, OBJECT(dev), "mac-lc-video.vram",
                           s->vram_size, errp);
    if (*errp) {
        return;
    }
    memory_region_set_log(&s->vram, true, DIRTY_MEMORY_VGA);
    sysbus_init_mmio(sbd, &s->vram);
    memory_region_init_io(&s->dac_mmio, OBJECT(dev), &sv_dac_ops, s,
                          "mac-lc-video.dac", 4);
    sysbus_init_mmio(sbd, &s->dac_mmio);
    memory_region_init_io(&s->ctrl_mmio, OBJECT(dev), &sv_ctrl_ops, s,
                          "mac-lc-video.ctrl", 8);
    sysbus_init_mmio(sbd, &s->ctrl_mmio);
    qdev_init_gpio_out_named(dev, &s->vbl_irq, "vbl", 1);
    qdev_init_gpio_in_named(dev, sv_set_depth, "depth", 3);

    s->vbl_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, sv_vbl, s);
    s->con = qemu_graphic_console_create(dev, 0, &sv_ops, s);
    qemu_console_resize(s->con, sonora_monitors[s->monitor].width,
                        sonora_monitors[s->monitor].height);
}

static int sv_post_load(void *opaque, int version_id)
{
    MacLCVideoState *s = opaque;

    s->invalidate = true;
    return 0;
}

static const VMStateDescription vmstate_mac_lc_video = {
    .name = "mac-lc-video",
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = sv_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8(mode, MacLCVideoState),
        VMSTATE_UINT8(depth, MacLCVideoState),
        VMSTATE_UINT8(sense_drive, MacLCVideoState),
        VMSTATE_UINT8(vtest, MacLCVideoState),
        VMSTATE_UINT8(dac_index, MacLCVideoState),
        VMSTATE_UINT8(dac_step, MacLCVideoState),
        VMSTATE_UINT8(dac_control, MacLCVideoState),
        VMSTATE_UINT8(dac_key, MacLCVideoState),
        VMSTATE_BUFFER(clut, MacLCVideoState),
        VMSTATE_BOOL(vbl, MacLCVideoState),
        VMSTATE_TIMER_PTR(vbl_timer, MacLCVideoState),
        VMSTATE_END_OF_LIST()
    }
};

static const Property sv_properties[] = {
    DEFINE_PROP_BOOL("v8", MacLCVideoState, v8, false),
    DEFINE_PROP_SIZE32("vram-size", MacLCVideoState, vram_size,
                       512 * KiB),
    DEFINE_PROP_UINT32("width", MacLCVideoState, width, 0),
    DEFINE_PROP_UINT32("height", MacLCVideoState, height, 0),
};

static void sv_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->desc = "Apple V8/Sonora built-in video (Macintosh LC family)";
    dc->realize = sv_realize;
    dc->vmsd = &vmstate_mac_lc_video;
    rc->phases.hold = sv_reset_hold;
    device_class_set_props(dc, sv_properties);
    set_bit(DEVICE_CATEGORY_DISPLAY, dc->categories);
}

static const TypeInfo sv_info = {
    .name = TYPE_MAC_LC_VIDEO,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MacLCVideoState),
    .class_init = sv_class_init,
};

static void sv_register_types(void)
{
    type_register_static(&sv_info);
}

type_init(sv_register_types)

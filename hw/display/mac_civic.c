/*
 * Apple Civic, the built-in video of the Quadra 660AV and 840AV, with its
 * Sebastian CLUT/DAC
 *
 * Civic's registers ($50036000) are one bit per longword: a register of n
 * bits at offset o has bit i at o + 4 * i. The timing registers give the
 * picture size (HAL/HFP in units of 2 pixels, VAL/VFP in half lines),
 * RowWords and BaseAddr the frame buffer's row length and start in units of
 * 32 bytes. VRAM is at $50100000 (1 MB, 2 MB with the VRAM upgrade).
 *
 * Sebastian ($50F30800): +$00 CLUT address, +$10 CLUT data (red, green,
 * blue, then one more write per entry), +$20 control (bits 0-2: 1, 2, 4,
 * 8, 16 or 32 bits per pixel). Byte wide, in the high byte of a longword.
 *
 * The monitor is picked with -g (its size); Civic reads its sense lines.
 * The VBL interrupt goes to the PSC's slot line 6.
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
#include "hw/display/mac_civic.h"
#include "trace.h"

#define VRAM_WINDOW     (2 * MiB)

/* registers, by the offset of their bit 0 */
#define R_VBLINT        0x000
#define R_ENABLE        0x004
#define R_SENSE_CTL     0x058
#define R_SENSE0        0x05c
#define R_SENSE1        0x060
#define R_SENSE2        0x064
#define R_READ_SENSE    0x080   /* 3 bits */
#define R_ROW_WORDS     0x08c   /* 8 bits */
#define R_BASE_ADDR     0x0c0   /* 9 bits */
#define R_VBL_ENB       0x110
#define R_VBL_CLR       0x120
#define R_HLFLN         0x280   /* 12 bits */
#define R_HAL           0x380
#define R_HFP           0x3c0
#define R_VHLINE        0x480
#define R_VAL           0x580
#define R_VFP           0x640
#define R_CUR_LINE      0x6c0

#define SENSE_A 4
#define SENSE_B 2
#define SENSE_C 1

typedef struct CivicMonitor {
    const char *name;
    uint16_t width, height;
    uint8_t grounded;           /* sense lines tied to ground */
    uint8_t tied;               /* two lines tied together, or 0 */
    uint8_t hz;
    bool mono;
} CivicMonitor;

static const CivicMonitor civic_monitors[] = {
    { "13\" RGB", 640, 480, SENSE_C, 0, 67 },
    { "12\" RGB", 512, 384, SENSE_A | SENSE_C, 0, 60 },
    { "15\" Portrait", 640, 870, SENSE_A | SENSE_B, 0, 75, true },
    { "16\" RGB", 832, 624, 0, SENSE_A | SENSE_C, 75 },
    { "19\" RGB", 1024, 768, 0, SENSE_B | SENSE_C, 75 },
    { "21\" RGB", 1152, 870, SENSE_A | SENSE_B | SENSE_C, 0, 75 },
    { "VGA", 640, 480, 0, SENSE_A | SENSE_B, 60 },
};

static const CivicMonitor *civic_monitor(MacCivicState *s)
{
    return &civic_monitors[s->monitor];
}

static uint32_t civic_reg(MacCivicState *s, unsigned off, unsigned nbits)
{
    uint32_t v = 0;
    unsigned i;

    for (i = 0; i < nbits; i++) {
        v |= (uint32_t)(s->bits[off / 4 + i] & 1) << i;
    }
    return v;
}

/*
 * The ROM drives one sense line low at a time through Sense2-0: 001 none,
 * 101 line A, 011 line B, 000 line C
 */
static uint8_t civic_sense_lines(MacCivicState *s)
{
    const CivicMonitor *m = civic_monitor(s);
    uint8_t lines = 7 & ~m->grounded;

    switch (civic_reg(s, R_SENSE2, 1) << 2 | civic_reg(s, R_SENSE1, 1) << 1 |
            civic_reg(s, R_SENSE0, 1)) {
    case 5:
        lines &= ~SENSE_A;
        break;
    case 3:
        lines &= ~SENSE_B;
        break;
    case 0:
        lines &= ~SENSE_C;
        break;
    }
    if (m->tied && (lines & m->tied) != m->tied) {
        lines &= ~m->tied;
    }
    return lines;
}

/* Display */

static int64_t civic_frame_ns(MacCivicState *s)
{
    return NANOSECONDS_PER_SECOND / civic_monitor(s)->hz;
}

static void civic_size(MacCivicState *s, unsigned *w, unsigned *h)
{
    unsigned hal = civic_reg(s, R_HAL, 12), hfp = civic_reg(s, R_HFP, 12);
    unsigned val = civic_reg(s, R_VAL, 12), vfp = civic_reg(s, R_VFP, 12);

    *w = hfp > hal ? (hfp - hal) * 2 : 0;
    *h = vfp > val ? (vfp - val) / 2 : 0;
    if (!*w || !*h || *w > 1152 || *h > 870) {
        *w = civic_monitor(s)->width;
        *h = civic_monitor(s)->height;
    }
}

static uint32_t civic_clut(MacCivicState *s, uint8_t i)
{
    if (civic_monitor(s)->mono) {
        return rgb_to_pixel32(s->clut[i * 3 + 2], s->clut[i * 3 + 2],
                              s->clut[i * 3 + 2]);
    }
    return rgb_to_pixel32(s->clut[i * 3], s->clut[i * 3 + 1],
                          s->clut[i * 3 + 2]);
}

static void civic_draw_line(MacCivicState *s, uint32_t *d, const uint8_t *p,
                            unsigned w)
{
    unsigned depth = s->dac_ctrl & 7, bpp, x, bit, v;

    switch (depth) {
    case 0:
    case 1:
    case 2:
    case 3:
        bpp = 1 << depth;
        for (x = 0; x < w; x++) {
            bit = x * bpp;
            v = (p[bit >> 3] >> (8 - bpp - (bit & 7))) & ((1 << bpp) - 1);
            /* fewer bits index the top of the CLUT */
            *d++ = civic_clut(s, (v << (8 - bpp)) | (0xff >> bpp));
        }
        break;
    case 4:
        /* direct colour, x1-5-5-5 */
        for (x = 0; x < w; x++, p += 2) {
            v = lduw_be_p(p);
            *d++ = rgb_to_pixel32(((v >> 10) & 0x1f) * 255 / 31,
                                  ((v >> 5) & 0x1f) * 255 / 31,
                                  (v & 0x1f) * 255 / 31);
        }
        break;
    default:
        /* direct colour, x-8-8-8 */
        for (x = 0; x < w; x++, p += 4) {
            *d++ = rgb_to_pixel32(p[1], p[2], p[3]);
        }
        break;
    }
}

static bool civic_update_display(void *opaque)
{
    MacCivicState *s = opaque;
    DisplaySurface *surface = qemu_console_surface(s->con);
    DirtyBitmapSnapshot *snap;
    uint8_t *vram = memory_region_get_ram_ptr(&s->vram);
    uint32_t rowbytes = civic_reg(s, R_ROW_WORDS, 8) * 32;
    uint32_t base = civic_reg(s, R_BASE_ADDR, 9) * 32;
    unsigned w, h;
    int y, ymin = -1;
    bool on;

    civic_size(s, &w, &h);
    on = civic_reg(s, R_ENABLE, 1) && rowbytes &&
         base + (uint64_t)rowbytes * h <= s->vram_size;
    if (surface_width(surface) != w || surface_height(surface) != h) {
        qemu_console_resize(s->con, w, h);
        surface = qemu_console_surface(s->con);
        s->invalidate = true;
    }
    if (!on) {
        if (s->invalidate) {
            memset(surface_data(surface), 0, surface_stride(surface) * h);
            qemu_console_update(s->con, 0, 0, w, h);
            s->invalidate = false;
        }
        return true;
    }

    snap = memory_region_snapshot_and_clear_dirty(&s->vram, base,
                                                  rowbytes * h,
                                                  DIRTY_MEMORY_VGA);
    for (y = 0; y < h; y++) {
        if (s->invalidate ||
            memory_region_snapshot_get_dirty(&s->vram, snap,
                                             base + y * rowbytes, rowbytes)) {
            civic_draw_line(s, (uint32_t *)(surface_data(surface) +
                                            y * surface_stride(surface)),
                            vram + base + y * rowbytes, w);
            if (ymin < 0) {
                ymin = y;
            }
        } else if (ymin >= 0) {
            qemu_console_update(s->con, 0, ymin, w, y - ymin);
            ymin = -1;
        }
    }
    if (ymin >= 0) {
        qemu_console_update(s->con, 0, ymin, w, y - ymin);
    }
    s->invalidate = false;
    g_free(snap);
    return true;
}

static const GraphicHwOps civic_ops = {
    .gfx_update = civic_update_display,
};

/* VBL */

static void civic_update_irq(MacCivicState *s)
{
    qemu_set_irq(s->vbl_irq, s->vbl && civic_reg(s, R_VBL_ENB, 1));
}

static void civic_vbl(void *opaque)
{
    MacCivicState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    s->frame_start_ns = now;
    s->vbl = true;
    civic_update_irq(s);
    timer_mod(s->vbl_timer, now + civic_frame_ns(s));
}

static uint32_t civic_cur_line(MacCivicState *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    unsigned w, h, total;

    civic_size(s, &w, &h);
    total = h + h / 20 + 2;
    return muldiv64(now - s->frame_start_ns, total, civic_frame_ns(s)) % total;
}

/* Civic registers */

static uint64_t civic_reg_read(void *opaque, hwaddr addr, unsigned size)
{
    MacCivicState *s = opaque;
    unsigned off = addr & ~3;
    uint32_t val;

    if (off == R_VBLINT) {
        val = s->vbl;
    } else if (off >= R_READ_SENSE && off < R_READ_SENSE + 12) {
        val = (civic_sense_lines(s) >> ((off - R_READ_SENSE) / 4)) & 1;
    } else if (off >= R_CUR_LINE && off < R_CUR_LINE + 48) {
        val = (civic_cur_line(s) >> ((off - R_CUR_LINE) / 4)) & 1;
    } else {
        val = s->bits[off / 4] & 1;
    }
    trace_mac_civic_read(addr, val);
    return val;
}

static void civic_reg_write(void *opaque, hwaddr addr, uint64_t val,
                            unsigned size)
{
    MacCivicState *s = opaque;
    unsigned off = addr & ~3;

    trace_mac_civic_write(addr, val);
    s->bits[off / 4] = val & 1;
    switch (off) {
    case R_VBL_CLR:
        if (!(val & 1)) {
            s->vbl = false;
        }
        break;
    default:
        s->invalidate = true;
        break;
    }
    civic_update_irq(s);
}

static const MemoryRegionOps civic_reg_ops = {
    .read = civic_reg_read,
    .write = civic_reg_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

/* Sebastian CLUT/DAC */

static uint64_t civic_dac_read(void *opaque, hwaddr addr, unsigned size)
{
    MacCivicState *s = opaque;
    uint8_t val = 0;

    switch (addr & 0x30) {
    case 0x00:
        val = s->dac_index;
        s->dac_step = 0;
        break;
    case 0x10:
        if (s->dac_step < 3) {
            val = s->clut[s->dac_index * 3 + s->dac_step];
        }
        if (++s->dac_step == 4) {
            s->dac_step = 0;
            s->dac_index++;
        }
        break;
    case 0x20:
        val = s->dac_ctrl;
        break;
    }
    trace_mac_civic_dac_read(addr, val);
    return (uint32_t)val << 24;
}

static void civic_dac_write(void *opaque, hwaddr addr, uint64_t val,
                            unsigned size)
{
    MacCivicState *s = opaque;
    uint8_t v = val >> 24;

    trace_mac_civic_dac_write(addr, v);
    switch (addr & 0x30) {
    case 0x00:
        s->dac_index = v;
        s->dac_step = 0;
        break;
    case 0x10:
        if (s->dac_step < 3) {
            s->clut[s->dac_index * 3 + s->dac_step] = v;
        }
        if (++s->dac_step == 4) {
            s->dac_step = 0;
            s->dac_index++;
        }
        s->invalidate = true;
        break;
    case 0x20:
        s->dac_ctrl = v;
        s->invalidate = true;
        break;
    }
}

static const MemoryRegionOps civic_dac_ops = {
    .read = civic_dac_read,
    .write = civic_dac_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

/* the pixel clock generator: only remembered */

static uint64_t civic_clock_read(void *opaque, hwaddr addr, unsigned size)
{
    MacCivicState *s = opaque;

    return s->clock_regs[(addr >> 4) & 3];
}

static void civic_clock_write(void *opaque, hwaddr addr, uint64_t val,
                              unsigned size)
{
    MacCivicState *s = opaque;

    trace_mac_civic_clock_write(addr, val);
    s->clock_regs[(addr >> 4) & 3] = val;
}

static const MemoryRegionOps civic_clock_ops = {
    .read = civic_clock_read,
    .write = civic_clock_write,
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

static uint64_t civic_open_bus_read(void *opaque, hwaddr addr, unsigned size)
{
    return 0;
}

static void civic_open_bus_write(void *opaque, hwaddr addr, uint64_t val,
                                 unsigned size)
{
}

static const MemoryRegionOps civic_open_bus_ops = {
    .read = civic_open_bus_read,
    .write = civic_open_bus_write,
    .endianness = DEVICE_BIG_ENDIAN,
};

static void civic_reset(DeviceState *dev)
{
    MacCivicState *s = MAC_CIVIC(dev);

    memset(s->bits, 0, sizeof(s->bits));
    s->bits[R_SENSE0 / 4] = 1;
    s->vbl = false;
    s->dac_index = s->dac_step = s->dac_ctrl = 0;
    civic_update_irq(s);
    s->frame_start_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    timer_mod(s->vbl_timer, s->frame_start_ns + civic_frame_ns(s));
}

static void civic_realize(DeviceState *dev, Error **errp)
{
    MacCivicState *s = MAC_CIVIC(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    unsigned i;

    if (s->vram_size != 1 * MiB && s->vram_size != 2 * MiB) {
        error_setg(errp, "mac-civic: vram-size must be 1 or 2 MiB");
        return;
    }
    s->monitor = 0;
    for (i = 0; i < ARRAY_SIZE(civic_monitors); i++) {
        if (civic_monitors[i].width == s->width &&
            civic_monitors[i].height == s->height) {
            s->monitor = i;
            break;
        }
    }
    if (i == ARRAY_SIZE(civic_monitors) && s->width) {
        error_setg(errp, "mac-civic: no monitor of %ux%u", s->width,
                   s->height);
        return;
    }

    memory_region_init_ram(&s->vram, OBJECT(dev), "mac-civic.vram",
                           s->vram_size, errp);
    if (*errp) {
        return;
    }
    memory_region_set_log(&s->vram, true, DIRTY_MEMORY_VGA);
    memory_region_init(&s->vram_window, OBJECT(dev), "mac-civic.vram-window",
                       VRAM_WINDOW);
    memory_region_init_io(&s->vram_open_bus, OBJECT(dev), &civic_open_bus_ops,
                          s, "mac-civic.vram-open-bus", VRAM_WINDOW);
    memory_region_add_subregion_overlap(&s->vram_window, 0, &s->vram_open_bus,
                                        -1);
    memory_region_add_subregion(&s->vram_window, 0, &s->vram);
    sysbus_init_mmio(sbd, &s->regs_mmio);
    sysbus_init_mmio(sbd, &s->vram_window);
    sysbus_init_mmio(sbd, &s->clock_mmio);
    sysbus_init_mmio(sbd, &s->dac_mmio);

    s->vbl_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, civic_vbl, s);
    s->con = qemu_graphic_console_create(dev, 0, &civic_ops, s);
    qemu_console_resize(s->con, civic_monitor(s)->width,
                        civic_monitor(s)->height);
}

static void civic_init(Object *obj)
{
    MacCivicState *s = MAC_CIVIC(obj);

    memory_region_init_io(&s->regs_mmio, obj, &civic_reg_ops, s,
                          "mac-civic.regs", 0x700);
    memory_region_init_io(&s->clock_mmio, obj, &civic_clock_ops, s,
                          "mac-civic.clock", 0x2000);
    memory_region_init_io(&s->dac_mmio, obj, &civic_dac_ops, s,
                          "mac-civic.dac", 0x40);
    qdev_init_gpio_out_named(DEVICE(obj), &s->vbl_irq, "vbl", 1);
}

static const VMStateDescription vmstate_civic = {
    .name = "mac-civic",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8_ARRAY(bits, MacCivicState, MAC_CIVIC_REG_BITS),
        VMSTATE_BOOL(vbl, MacCivicState),
        VMSTATE_INT64(frame_start_ns, MacCivicState),
        VMSTATE_UINT8(dac_index, MacCivicState),
        VMSTATE_UINT8(dac_step, MacCivicState),
        VMSTATE_UINT8(dac_ctrl, MacCivicState),
        VMSTATE_UINT8_ARRAY(clut, MacCivicState, 256 * 3),
        VMSTATE_UINT8_ARRAY(clock_regs, MacCivicState, 4),
        VMSTATE_TIMER_PTR(vbl_timer, MacCivicState),
        VMSTATE_END_OF_LIST()
    },
};

static const Property civic_properties[] = {
    DEFINE_PROP_UINT32("vram-size", MacCivicState, vram_size, 1 * MiB),
    DEFINE_PROP_UINT32("width", MacCivicState, width, 0),
    DEFINE_PROP_UINT32("height", MacCivicState, height, 0),
};

static void civic_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = civic_realize;
    device_class_set_legacy_reset(dc, civic_reset);
    dc->vmsd = &vmstate_civic;
    device_class_set_props(dc, civic_properties);
}

static const TypeInfo civic_types[] = {
    {
        .name = TYPE_MAC_CIVIC,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(MacCivicState),
        .instance_init = civic_init,
        .class_init = civic_class_init,
    },
};

DEFINE_TYPES(civic_types)

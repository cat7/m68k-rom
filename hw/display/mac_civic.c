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
 * Video input: the SAA7191 decodes the picture and the SAA7186 scales it
 * into VRAM at $50200800 (rows of 1024 bytes, 1536 with VidInSize), as
 * 16-bit RGB or 8-bit grey. VDCClk 0 keeps it coming; each finished frame
 * sets VDCInt (active low, cleared through VDCClr) on the VBL's slot line.
 * Civic shows the buffer in a window of the screen (VInHAL/VInHFP,
 * VInVAL/VInVFP in the display's timing; VInDoubleLine shows each row
 * twice) while Sebastian's control bit 7 is set; at 8 bits per pixel and
 * fewer only where the screen has a colour whose CLUT entry has its 4th
 * byte set (the key colour). The picture comes from the host: the
 * "video-in" property picks a camera ("camera", "camera:<name>") or colour
 * bars ("bars"); without one the decoder finds no signal.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "ui/pixel_ops.h"
#include "hw/display/mac_civic.h"
#include "hw/display/saa71xx.h"
#include "hostcam.h"
#include "trace.h"

#define VRAM_WINDOW     (2 * MiB)

/* registers, by the offset of their bit 0 */
#define R_VBLINT        0x000
#define R_ENABLE        0x004
#define R_VDC_INT       0x008   /* active low */
#define R_VDC_CLR       0x00c
#define R_VDC_ENB       0x010
#define R_VID_IN_SIZE   0x014
#define R_VDC_CLK       0x018   /* 1: capture stopped */
#define R_SENSE_CTL     0x058
#define R_SENSE0        0x05c
#define R_SENSE1        0x060
#define R_SENSE2        0x064
#define R_READ_SENSE    0x080   /* 3 bits */
#define R_ROW_WORDS     0x08c   /* 8 bits */
#define R_BASE_ADDR     0x0c0   /* 9 bits */
#define R_VBL_ENB       0x110
#define R_VBL_CLR       0x120
#define R_VIN_HAL       0x1c0   /* 12 bits */
#define R_VIN_DOUBLE    0x208
#define R_VIN_HFP       0x240   /* 12 bits */
#define R_HLFLN         0x280   /* 12 bits */
#define R_HAL           0x380
#define R_HFP           0x3c0
#define R_VHLINE        0x480
#define R_VAL           0x580
#define R_VFP           0x640
#define R_VIN_VAL       0x5c0   /* 12 bits */
#define R_VIN_VFP       0x600   /* 12 bits */
#define R_CUR_LINE      0x6c0

/*
 * VRAM is two banks, at $50100000 and $50200000, of 512 KB each with
 * 1 MB of VRAM; the video input's buffer is at $50200800
 */
#define VRAM_BANK_B     0x100000
#define VIDEO_BUF_B     0x800

/* Sebastian control */
#define DAC_CTRL_VIDEO_16       0x08    /* the video buffer is 16-bit RGB */
#define DAC_CTRL_VIDEO_TABLE    0x40
#define DAC_CTRL_VIDEO_ON       0x80    /* show the video window */

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

/* where the video buffer is in s->vram */
static uint32_t civic_video_buf(MacCivicState *s)
{
    return s->vram_size / 2 + VIDEO_BUF_B;
}

/* Video input window */

typedef struct CivicWindow {
    int x0, y0, x1, y1;         /* in screen pixels; x1, y1 excluded */
} CivicWindow;

/*
 * The VIn registers count like the display's: horizontally from HAL in
 * units of 2 pixels (1 with a 16-bit video buffer or screen), vertically
 * from VAL in half lines
 */
static bool civic_video_window(MacCivicState *s, unsigned w, unsigned h,
                               CivicWindow *win)
{
    int hal = civic_reg(s, R_HAL, 12), val = civic_reg(s, R_VAL, 12);
    int k = (s->dac_ctrl & DAC_CTRL_VIDEO_16) || (s->dac_ctrl & 7) == 4 ?
            1 : 2;

    if (!(s->dac_ctrl & DAC_CTRL_VIDEO_ON)) {
        return false;
    }
    win->x0 = MAX((int)civic_reg(s, R_VIN_HAL, 12) + 1 - hal, 0) * k;
    win->x1 = MIN(((int)civic_reg(s, R_VIN_HFP, 12) + 1 - hal) * k, (int)w);
    win->y0 = MAX((int)civic_reg(s, R_VIN_VAL, 12) - val, 0) / 2;
    win->y1 = MIN(((int)civic_reg(s, R_VIN_VFP, 12) - val) / 2, (int)h);
    return win->x0 < win->x1 && win->y0 < win->y1;
}

/* the video buffer's pixel for the window's pixel x, y */
static uint32_t civic_video_pixel(MacCivicState *s, const uint8_t *vram,
                                  uint32_t row, unsigned x)
{
    uint32_t off;
    unsigned v;

    if (s->dac_ctrl & DAC_CTRL_VIDEO_16) {
        off = row + x * 2;
        if (off + 2 > s->vram_size) {
            return 0;
        }
        v = lduw_be_p(vram + off);
        return rgb_to_pixel32(((v >> 10) & 0x1f) * 255 / 31,
                              ((v >> 5) & 0x1f) * 255 / 31,
                              (v & 0x1f) * 255 / 31);
    }
    off = row + x;
    if (off >= s->vram_size) {
        return 0;
    }
    v = vram[off];
    return rgb_to_pixel32(v, v, v);
}

static void civic_draw_video(MacCivicState *s, uint32_t *d, const uint8_t *p,
                             const CivicWindow *win, int y)
{
    uint8_t *vram = memory_region_get_ram_ptr(&s->vram);
    unsigned depth = s->dac_ctrl & 7, bpp = 1 << depth, bit, v;
    uint32_t stride = civic_reg(s, R_VID_IN_SIZE, 1) ? 1536 : 1024;
    uint32_t row;
    int x;

    row = civic_video_buf(s) +
          ((y - win->y0) >> civic_reg(s, R_VIN_DOUBLE, 1)) * stride;
    for (x = win->x0; x < win->x1; x++) {
        if (depth <= 3) {
            bit = x * bpp;
            v = (p[bit >> 3] >> (8 - bpp - (bit & 7))) & ((1 << bpp) - 1);
            if (!s->clut_key[(v << (8 - bpp)) | (0xff >> bpp)]) {
                continue;
            }
        }
        d[x] = civic_video_pixel(s, vram, row, x - win->x0);
    }
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
    bool on, video;
    CivicWindow win;

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
    video = civic_video_window(s, w, h, &win);
    for (y = 0; y < h; y++) {
        bool in_win = video && y >= win.y0 && y < win.y1;

        if (s->invalidate || (in_win && s->video_fresh) ||
            memory_region_snapshot_get_dirty(&s->vram, snap,
                                             base + y * rowbytes, rowbytes)) {
            uint32_t *d = (uint32_t *)(surface_data(surface) +
                                       y * surface_stride(surface));

            civic_draw_line(s, d, vram + base + y * rowbytes, w);
            if (in_win) {
                civic_draw_video(s, d, vram + base + y * rowbytes, &win, y);
            }
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
    s->video_fresh = false;
    g_free(snap);
    return true;
}

static const GraphicHwOps civic_ops = {
    .gfx_update = civic_update_display,
};

/* VBL */

static void civic_update_irq(MacCivicState *s)
{
    qemu_set_irq(s->vbl_irq,
                 (s->vbl && civic_reg(s, R_VBL_ENB, 1)) ||
                 (s->vdc_pending && civic_reg(s, R_VDC_ENB, 1)));
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

/* Video input */

enum {
    VIDEO_IN_NONE,
    VIDEO_IN_BARS,
    VIDEO_IN_CAMERA,
};

/* 75% colour bars, with a white block going across */
static void civic_bars(MacCivicState *s, uint32_t *d, unsigned w,
                       unsigned h)
{
    static const uint32_t bars[8] = {
        0xbfbfbf, 0xbfbf00, 0x00bfbf, 0x00bf00,
        0xbf00bf, 0xbf0000, 0x0000bf, 0x000000,
    };
    unsigned x, y, bx = (s->video_frames * 4) % w;

    for (y = 0; y < h; y++) {
        for (x = 0; x < w; x++) {
            if (y >= h * 3 / 4 && y < h * 7 / 8 && x >= bx && x < bx + w / 8) {
                *d++ = 0xffffff;
            } else {
                *d++ = bars[x * 8 / w];
            }
        }
    }
}

/* the next picture into the buffer, in the scaler's size and format */
static void civic_capture(MacCivicState *s)
{
    uint8_t *vram = memory_region_get_ram_ptr(&s->vram);
    uint32_t stride = civic_reg(s, R_VID_IN_SIZE, 1) ? 1536 : 1024;
    unsigned fmt = saa7186_format(s->scaler);
    unsigned bpp = fmt == SAA7186_FMT_MONO ? 1 :
                   fmt == SAA7186_FMT_RGB888 ? 4 : 2;
    unsigned w = saa7186_width(s->scaler), h = saa7186_lines(s->scaler);
    unsigned x, y;
    int r, g, b, yy;
    uint32_t *src, c;
    uint8_t *d;

    w = MIN(w, stride / bpp);
    if (civic_video_buf(s) + stride > s->vram_size) {
        return;
    }
    h = MIN(h, (s->vram_size - civic_video_buf(s)) / stride);
    if (!w || !h) {
        return;
    }
    if (s->video_w != w || s->video_h != h) {
        g_free(s->video_src);
        s->video_src = g_new0(uint32_t, w * h);
        s->video_w = w;
        s->video_h = h;
    }
    src = s->video_src;
    if (s->video_in == VIDEO_IN_BARS) {
        civic_bars(s, src, w, h);
    } else if (!s->cam || !hostcam_grab(s->cam, src, w, h)) {
        memset(src, 0, w * h * 4);
    }
    trace_mac_civic_video_frame(w, h, fmt, stride);

    for (y = 0; y < h; y++) {
        d = vram + civic_video_buf(s) + y * stride;
        for (x = 0; x < w; x++) {
            c = *src++;
            r = c >> 16;
            g = (c >> 8) & 0xff;
            b = c & 0xff;
            yy = (77 * r + 150 * g + 29 * b) >> 8;
            switch (fmt) {
            case SAA7186_FMT_RGB555:
                stw_be_p(d, (r >> 3) << 10 | (g >> 3) << 5 | b >> 3);
                d += 2;
                break;
            case SAA7186_FMT_RGB888:
                stl_be_p(d, c & 0xffffff);
                d += 4;
                break;
            case SAA7186_FMT_YUV422:
                /* U Y0 V Y1 */
                if (x & 1) {
                    d[0] = 128 + (128 * r - 107 * g - 21 * b) / 256;
                } else {
                    d[0] = 128 + (-43 * r - 85 * g + 128 * b) / 256;
                }
                d[1] = yy;
                d += 2;
                break;
            default:
                *d++ = yy;
                break;
            }
        }
    }
    memory_region_set_dirty(&s->vram, civic_video_buf(s), h * stride);
}

static int64_t civic_video_frame_ns(MacCivicState *s)
{
    return NANOSECONDS_PER_SECOND / (saa7191_50hz(s->decoder) ? 25 : 30);
}

/*
 * A frame goes into the buffer while the capture clock runs and the
 * scaler is on; the decoder only locks onto a source that is there
 */
static void civic_video_tick(void *opaque)
{
    MacCivicState *s = opaque;

    if (s->video_in != VIDEO_IN_NONE && saa7186_running(s->scaler) &&
        !civic_reg(s, R_VDC_CLK, 1)) {
        civic_capture(s);
        s->video_frames++;
        s->video_fresh = true;
        s->vdc_pending = true;
        civic_update_irq(s);
    }
    timer_mod(s->video_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                              civic_video_frame_ns(s));
}

/* Civic registers */

static uint64_t civic_reg_read(void *opaque, hwaddr addr, unsigned size)
{
    MacCivicState *s = opaque;
    unsigned off = addr & ~3;
    uint32_t val;

    if (off == R_VBLINT) {
        val = s->vbl;
    } else if (off == R_VDC_INT) {
        val = !s->vdc_pending;
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
    case R_VDC_CLR:
        if (!(val & 1)) {
            s->vdc_pending = false;
        }
        break;
    case R_VDC_ENB:
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

/*
 * Control bit 6 switches the data port from the CLUT (red, green, blue and
 * one more access per entry) to the video input's table (red, green, blue)
 */
static uint8_t *civic_dac_table(MacCivicState *s)
{
    return s->dac_ctrl & DAC_CTRL_VIDEO_TABLE ? s->video_table : s->clut;
}

static void civic_dac_step(MacCivicState *s)
{
    if (++s->dac_step == (s->dac_ctrl & DAC_CTRL_VIDEO_TABLE ? 3 : 4)) {
        s->dac_step = 0;
        s->dac_index++;
    }
}

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
            val = civic_dac_table(s)[s->dac_index * 3 + s->dac_step];
        } else {
            val = s->clut_key[s->dac_index];
        }
        civic_dac_step(s);
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
            civic_dac_table(s)[s->dac_index * 3 + s->dac_step] = v;
        } else {
            /* the CLUT's 4th byte: the colour shows the video input */
            s->clut_key[s->dac_index] = v;
        }
        civic_dac_step(s);
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
    s->bits[R_VDC_CLK / 4] = 1;
    s->vbl = false;
    s->vdc_pending = false;
    s->dac_index = s->dac_step = s->dac_ctrl = 0;
    memset(s->clut_key, 0, sizeof(s->clut_key));
    civic_update_irq(s);
    s->frame_start_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    timer_mod(s->vbl_timer, s->frame_start_ns + civic_frame_ns(s));
    timer_mod(s->video_timer, s->frame_start_ns + civic_video_frame_ns(s));
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
    if (!s->decoder || !s->scaler) {
        error_setg(errp, "mac-civic: the decoder and scaler links are not set");
        return;
    }
    if (!s->video_in_name || !*s->video_in_name ||
        !strcmp(s->video_in_name, "none")) {
        s->video_in = VIDEO_IN_NONE;
    } else if (!strcmp(s->video_in_name, "bars")) {
        s->video_in = VIDEO_IN_BARS;
    } else if (!strcmp(s->video_in_name, "camera") ||
               g_str_has_prefix(s->video_in_name, "camera:")) {
        Error *err = NULL;
        const char *name = s->video_in_name[6] ? s->video_in_name + 7 :
                           "default";

        s->video_in = VIDEO_IN_CAMERA;
        s->cam = hostcam_open(name, &err);
        if (!s->cam) {
            /* the Mac sees a camera that shows nothing */
            warn_report_err(err);
        }
    } else {
        error_setg(errp, "mac-civic: video-in must be none, bars, camera or "
                   "camera:<name>");
        return;
    }
    s->decoder->signal = s->video_in != VIDEO_IN_NONE;
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
    if (s->vram_size == 1 * MiB) {
        /* bank B */
        memory_region_init_alias(&s->vram_bank_b, OBJECT(dev),
                                 "mac-civic.vram-bank-b", &s->vram,
                                 s->vram_size / 2, s->vram_size / 2);
        memory_region_add_subregion(&s->vram_window, VRAM_BANK_B,
                                    &s->vram_bank_b);
    }
    sysbus_init_mmio(sbd, &s->regs_mmio);
    sysbus_init_mmio(sbd, &s->vram_window);
    sysbus_init_mmio(sbd, &s->clock_mmio);
    sysbus_init_mmio(sbd, &s->dac_mmio);

    s->vbl_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, civic_vbl, s);
    s->video_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, civic_video_tick, s);
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
        VMSTATE_UINT8_ARRAY(video_table, MacCivicState, 256 * 3),
        VMSTATE_UINT8_ARRAY(clut_key, MacCivicState, 256),
        VMSTATE_BOOL(vdc_pending, MacCivicState),
        VMSTATE_TIMER_PTR(video_timer, MacCivicState),
        VMSTATE_UINT8_ARRAY(clock_regs, MacCivicState, 4),
        VMSTATE_TIMER_PTR(vbl_timer, MacCivicState),
        VMSTATE_END_OF_LIST()
    },
};

static const Property civic_properties[] = {
    DEFINE_PROP_UINT32("vram-size", MacCivicState, vram_size, 1 * MiB),
    DEFINE_PROP_UINT32("width", MacCivicState, width, 0),
    DEFINE_PROP_UINT32("height", MacCivicState, height, 0),
    DEFINE_PROP_STRING("video-in", MacCivicState, video_in_name),
    DEFINE_PROP_LINK("decoder", MacCivicState, decoder, TYPE_SAA7191,
                     SAA7191State *),
    DEFINE_PROP_LINK("scaler", MacCivicState, scaler, TYPE_SAA7186,
                     SAA7186State *),
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

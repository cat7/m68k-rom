/*
 * QEMU NuBus video card
 *
 * A simple framebuffer card for 68k Macs with NuBus slots.  It is not a
 * model of any real card: its declaration ROM is generated here and its
 * Mac OS video driver is nubus-video-driver.s.
 *
 * Slot space layout (offsets from $Fs000000, s = slot):
 *
 *   $000000-$0FCFFF  VRAM, first 1012 KiB    \
 *   $0FD000-$0FDFFF  registers                > the 24-bit window
 *   $0FE000-$0FFFFF  declaration ROM         /
 *   $100000-...      all of the VRAM (up to 8 MiB)
 *   ...-$FFFFFF      the 24-bit window again in every free megabyte
 *
 * In 24-bit mode the Mac reaches only $Fs000000-$Fs0FFFFF (as $s00000),
 * so the declaration ROM shows up at the top of that megabyte as well as
 * at the top of the slot.  Like a real card that decodes only 20 address
 * bits, the window repeats through the rest of the slot, so the driver
 * can use $FssFD000 for the registers, which is right in both modes.
 * Frames that fit below the registers start at offset 0, so they also
 * work in 24-bit mode; larger ones (deep direct colour modes) start at
 * $100000 and their pixmaps are flagged as needing 32-bit addressing.
 *
 * The Slot Manager gives the driver the 24-bit base address $00s00000,
 * which 32-bit QuickDraw turns into $Fss00000, the copy of the window
 * at offset $s00000.  So the VPBlock base offsets of the large frames
 * are relative to that copy, and depend on the slot.
 *
 * The frame always starts at VRAM offset 0.  The video modes are the
 * depths 1, 2, 4, 8 (indexed through a 256-entry CLUT), 16 (x555) and
 * 32 (x888) bpp of the one resolution chosen at startup; deeper ones are
 * dropped when the VRAM is too small for them.
 *
 * Registers (32-bit, big-endian, offsets from the register page):
 *
 *   $00 MODE        rw  depth: 0..5 = 1, 2, 4, 8, 16, 32 bpp
 *   $04 ROWBYTES    ro  row bytes of the current depth
 *   $08 BASE        ro  frame offset from the slot base for this depth
 *   $0C WIDTH       ro
 *   $10 HEIGHT      ro
 *   $14 VRAMSIZE    ro
 *   $18 NDEPTHS     ro  number of depths offered (from 1 bpp up)
 *   $1C ID          ro  'QVID'
 *   $20 INT_ENABLE  rw  bit 0: VBL interrupt
 *   $24 INT_STATUS  rw  bit 0: VBL happened; write 1 to clear
 *   $30 CLUT_INDEX  rw
 *   $34 CLUT_DATA   rw  $00RRGGBB at CLUT_INDEX, which then increments
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/timer.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/core/qdev-properties.h"
#include "hw/nubus/nubus.h"
#include "hw/nubus/nubus-decl-rom.h"
#include "hw/nubus/nubus-video.h"
#include "migration/vmstate.h"
#include "ui/console.h"
#include "ui/pixel_ops.h"
#include "nubus-video-driver.h"
#include "trace.h"

#define NV_WINDOW_SIZE      0x100000
#define NV_VRAM_LOW_SIZE    0x0fd000
#define NV_REGS_OFFSET      0x0fd000
#define NV_REGS_SIZE        0x1000
#define NV_ROM_OFFSET       0x0fe000
#define NV_ROM_SIZE         0x2000
#define NV_VRAM_OFFSET      0x100000
#define NV_VRAM_MAX         (8 * MiB)

#define NV_REG_MODE         0x00
#define NV_REG_ROWBYTES     0x04
#define NV_REG_BASE         0x08
#define NV_REG_WIDTH        0x0c
#define NV_REG_HEIGHT       0x10
#define NV_REG_VRAMSIZE     0x14
#define NV_REG_NDEPTHS      0x18
#define NV_REG_ID           0x1c
#define NV_REG_INT_ENABLE   0x20
#define NV_REG_INT_STATUS   0x24
#define NV_REG_CLUT_INDEX   0x30
#define NV_REG_CLUT_DATA    0x34

#define NV_ID               0x51564944  /* 'QVID' */
#define NV_INT_VBL          1
#define NV_VBL_PERIOD_NS    (NANOSECONDS_PER_SECOND / 60)

/* Board ID and DrHw are ours, not assigned by Apple */
#define NV_BOARD_ID         0x7151
#define NV_DRHW             0x7151

#define NV_FIRST_MODE       0x80        /* spID of the 1 bpp mode */
#define NV_DIRECT_DEPTH     4           /* 16 bpp and up */

/* video mode sResource entries */
#define NV_mVidParams       1
#define NV_mPageCnt         3
#define NV_mDevType         4
#define NV_clutType         0
#define NV_directType       2

static const uint8_t nv_bpp[NUBUS_VIDEO_NDEPTHS] = { 1, 2, 4, 8, 16, 32 };

static const struct {
    uint16_t width, height;
} nv_sizes[] = {
    { 512, 384 }, { 640, 480 }, { 640, 870 }, { 800, 600 }, { 832, 624 },
    { 1024, 768 }, { 1152, 870 }, { 1280, 960 }, { 1280, 1024 },
};

static uint32_t nv_rowbytes(NubusVideoState *s, int depth)
{
    return s->width * nv_bpp[depth] / 8;
}

static uint32_t nv_frame_size(NubusVideoState *s, int depth)
{
    return nv_rowbytes(s, depth) * s->height;
}

/* Frame offset from the slot base */
static uint32_t nv_base(NubusVideoState *s, int depth)
{
    return nv_frame_size(s, depth) <= NV_VRAM_LOW_SIZE ? 0 : NV_VRAM_OFFSET;
}

/* Declaration ROM */

static void nv_rom_vpblock(NubusDeclROM *r, NubusVideoState *s, int depth)
{
    bool direct = depth >= NV_DIRECT_DEPTH;
    uint32_t base = nv_base(s, depth);

    if (base) {
        base -= NUBUS_DEVICE(s)->slot * NV_WINDOW_SIZE;
    }
    nubus_decl_rom_long(r, 46);                         /* sBlock size */
    nubus_decl_rom_long(r, base);                       /* vpBaseOffset */
    nubus_decl_rom_word(r, nv_rowbytes(s, depth));      /* vpRowBytes */
    nubus_decl_rom_word(r, 0);                          /* vpBounds */
    nubus_decl_rom_word(r, 0);
    nubus_decl_rom_word(r, s->height);
    nubus_decl_rom_word(r, s->width);
    nubus_decl_rom_word(r, base ? 4 : 0);   /* vpVersion: 4 = baseAddr32 */
    nubus_decl_rom_word(r, 0);                          /* vpPackType */
    nubus_decl_rom_long(r, 0);                          /* vpPackSize */
    nubus_decl_rom_long(r, 72 << 16);                   /* vpHRes */
    nubus_decl_rom_long(r, 72 << 16);                   /* vpVRes */
    nubus_decl_rom_word(r, direct ? 16 : 0);            /* vpPixelType */
    nubus_decl_rom_word(r, nv_bpp[depth]);              /* vpPixelSize */
    nubus_decl_rom_word(r, direct ? 3 : 1);             /* vpCmpCount */
    nubus_decl_rom_word(r, direct ? (depth == 4 ? 5 : 8)
                                  : nv_bpp[depth]);     /* vpCmpSize */
    nubus_decl_rom_long(r, 0);                          /* vpPlaneBytes */
}

static void nubus_video_build_rom(NubusVideoState *s, uint8_t *buf, int size)
{
    NubusDeclROM r;
    int dir, dir_board, dir_video;
    int b_type, b_name, b_vendor, v_id, v_rev, v_part;
    int f_type, f_name, f_drvr, f_base, f_len, d_68000, d_68020;
    int f_mode[NUBUS_VIDEO_NDEPTHS], m_params[NUBUS_VIDEO_NDEPTHS];
    int d;

    nubus_decl_rom_init(&r, buf, size);

    /* sResource directory */
    dir = r.pos;
    dir_board = nubus_decl_rom_entry_off(&r, 0x01);
    dir_video = nubus_decl_rom_entry_off(&r, 0x80);
    nubus_decl_rom_end(&r);

    /* Board sResource */
    nubus_decl_rom_here(&r, dir_board);
    b_type = nubus_decl_rom_entry_off(&r, NUBUS_sRsrcType);
    b_name = nubus_decl_rom_entry_off(&r, NUBUS_sRsrcName);
    nubus_decl_rom_entry_data(&r, NUBUS_BoardId, NV_BOARD_ID);
    b_vendor = nubus_decl_rom_entry_off(&r, NUBUS_VendorInfo);
    nubus_decl_rom_end(&r);

    nubus_decl_rom_here(&r, b_vendor);
    v_id = nubus_decl_rom_entry_off(&r, NUBUS_VendorId);
    v_rev = nubus_decl_rom_entry_off(&r, NUBUS_RevLevel);
    v_part = nubus_decl_rom_entry_off(&r, NUBUS_PartNum);
    nubus_decl_rom_end(&r);

    /* Video sResource */
    nubus_decl_rom_here(&r, dir_video);
    f_type = nubus_decl_rom_entry_off(&r, NUBUS_sRsrcType);
    f_name = nubus_decl_rom_entry_off(&r, NUBUS_sRsrcName);
    f_drvr = nubus_decl_rom_entry_off(&r, NUBUS_sRsrcDrvrDir);
    nubus_decl_rom_entry_data(&r, NUBUS_sRsrcFlags, NUBUS_fOpenAtStart);
    nubus_decl_rom_entry_data(&r, NUBUS_sRsrcHWDevId, 1);
    f_base = nubus_decl_rom_entry_off(&r, NUBUS_MinorBaseOS);
    f_len = nubus_decl_rom_entry_off(&r, NUBUS_MinorLength);
    for (d = 0; d < s->ndepths; d++) {
        f_mode[d] = nubus_decl_rom_entry_off(&r, NV_FIRST_MODE + d);
    }
    nubus_decl_rom_end(&r);

    /* Video modes, one per depth */
    for (d = 0; d < s->ndepths; d++) {
        nubus_decl_rom_here(&r, f_mode[d]);
        m_params[d] = nubus_decl_rom_entry_off(&r, NV_mVidParams);
        nubus_decl_rom_entry_data(&r, NV_mPageCnt, 1);
        nubus_decl_rom_entry_data(&r, NV_mDevType,
                                  d >= NV_DIRECT_DEPTH ? NV_directType
                                                       : NV_clutType);
        nubus_decl_rom_end(&r);
    }

    nubus_decl_rom_here(&r, f_drvr);
    d_68000 = nubus_decl_rom_entry_off(&r, NUBUS_sMacOS68000);
    d_68020 = nubus_decl_rom_entry_off(&r, NUBUS_sMacOS68020);
    nubus_decl_rom_end(&r);

    /* Data */
    nubus_decl_rom_here(&r, b_type);
    nubus_decl_rom_word(&r, NUBUS_catBoard);
    nubus_decl_rom_word(&r, 0);
    nubus_decl_rom_word(&r, 0);
    nubus_decl_rom_word(&r, 0);

    nubus_decl_rom_here(&r, f_type);
    nubus_decl_rom_word(&r, NUBUS_catDisplay);
    nubus_decl_rom_word(&r, NUBUS_typeVideo);
    nubus_decl_rom_word(&r, NUBUS_drSwApple);
    nubus_decl_rom_word(&r, NV_DRHW);

    nubus_decl_rom_here(&r, f_base);
    nubus_decl_rom_long(&r, 0);
    nubus_decl_rom_here(&r, f_len);
    nubus_decl_rom_long(&r, NV_VRAM_LOW_SIZE);

    for (d = 0; d < s->ndepths; d++) {
        nubus_decl_rom_here(&r, m_params[d]);
        nv_rom_vpblock(&r, s, d);
    }

    nubus_decl_rom_here(&r, b_name);
    nubus_decl_rom_cstring(&r, "QEMU NuBus Video");
    nubus_decl_rom_here(&r, f_name);
    nubus_decl_rom_cstring(&r, "Display_Video_QEMU_NuBus");
    nubus_decl_rom_here(&r, v_id);
    nubus_decl_rom_cstring(&r, "QEMU");
    nubus_decl_rom_here(&r, v_rev);
    nubus_decl_rom_cstring(&r, "1.0");
    nubus_decl_rom_here(&r, v_part);
    nubus_decl_rom_cstring(&r, "QEMU NuBus Video");

    /* The driver, for any 68k */
    nubus_decl_rom_here(&r, d_68000);
    nubus_decl_rom_here(&r, d_68020);
    nubus_decl_rom_long(&r, 4 + sizeof(nubus_video_driver));
    nubus_decl_rom_bytes(&r, nubus_video_driver, sizeof(nubus_video_driver));

    nubus_decl_rom_finish(&r, dir);
}

/* Display */

static uint32_t nv_clut_pixel(NubusVideoState *s, uint8_t idx)
{
    uint32_t c = s->clut[idx];

    return rgb_to_pixel32((c >> 16) & 0xff, (c >> 8) & 0xff, c & 0xff);
}

static void nv_draw_line(NubusVideoState *s, uint32_t *d, const uint8_t *p)
{
    int x, w = s->width;
    int bpp = nv_bpp[s->mode];

    switch (bpp) {
    case 1:
    case 2:
    case 4:
        for (x = 0; x < w; x++) {
            int bit = x * bpp;
            int idx = (p[bit >> 3] >> (8 - bpp - (bit & 7))) & ((1 << bpp) - 1);

            *d++ = nv_clut_pixel(s, idx);
        }
        break;
    case 8:
        for (x = 0; x < w; x++) {
            *d++ = nv_clut_pixel(s, p[x]);
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
    case 32:
        for (x = 0; x < w; x++) {
            *d++ = rgb_to_pixel32(p[4 * x + 1], p[4 * x + 2], p[4 * x + 3]);
        }
        break;
    }
}

static bool nubus_video_update_display(void *opaque)
{
    NubusVideoState *s = opaque;
    DisplaySurface *surface = qemu_console_surface(s->con);
    DirtyBitmapSnapshot *snap;
    uint32_t rowbytes = nv_rowbytes(s, s->mode);
    int y, ymin = -1;

    if (surface_width(surface) != s->width ||
        surface_height(surface) != s->height) {
        qemu_console_resize(s->con, s->width, s->height);
        surface = qemu_console_surface(s->con);
        s->invalidate = true;
    }

    snap = memory_region_snapshot_and_clear_dirty(&s->vram, 0,
                                                  rowbytes * s->height,
                                                  DIRTY_MEMORY_VGA);
    for (y = 0; y < s->height; y++) {
        if (s->invalidate ||
            memory_region_snapshot_get_dirty(&s->vram, snap, y * rowbytes,
                                             rowbytes)) {
            nv_draw_line(s, (uint32_t *)(surface_data(surface) +
                                         y * surface_stride(surface)),
                         s->vram_ptr + y * rowbytes);
            if (ymin < 0) {
                ymin = y;
            }
        } else if (ymin >= 0) {
            qemu_console_update(s->con, 0, ymin, s->width, y - ymin);
            ymin = -1;
        }
    }
    if (ymin >= 0) {
        qemu_console_update(s->con, 0, ymin, s->width, y - ymin);
    }
    s->invalidate = false;
    g_free(snap);

    return true;
}

static void nubus_video_invalidate_display(void *opaque)
{
    NubusVideoState *s = opaque;

    s->invalidate = true;
}

static const GraphicHwOps nubus_video_ops = {
    .invalidate = nubus_video_invalidate_display,
    .gfx_update = nubus_video_update_display,
};

/* Interrupts */

static void nv_update_irq(NubusVideoState *s)
{
    nubus_set_irq(NUBUS_DEVICE(s), !!(s->int_status & s->int_enable));
}

static void nv_vbl_schedule(NubusVideoState *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    timer_mod(s->vbl_timer,
              (now + NV_VBL_PERIOD_NS) / NV_VBL_PERIOD_NS * NV_VBL_PERIOD_NS);
}

static void nv_vbl(void *opaque)
{
    NubusVideoState *s = opaque;

    s->int_status |= NV_INT_VBL;
    nv_update_irq(s);
    nv_vbl_schedule(s);
}

/* Registers */

static uint64_t nv_reg_value(NubusVideoState *s, hwaddr addr)
{
    uint32_t val;

    switch (addr) {
    case NV_REG_MODE:
        return s->mode;
    case NV_REG_ROWBYTES:
        return nv_rowbytes(s, s->mode);
    case NV_REG_BASE:
        return nv_base(s, s->mode);
    case NV_REG_WIDTH:
        return s->width;
    case NV_REG_HEIGHT:
        return s->height;
    case NV_REG_VRAMSIZE:
        return s->vram_size;
    case NV_REG_NDEPTHS:
        return s->ndepths;
    case NV_REG_ID:
        return NV_ID;
    case NV_REG_INT_ENABLE:
        return s->int_enable;
    case NV_REG_INT_STATUS:
        return s->int_status;
    case NV_REG_CLUT_INDEX:
        return s->clut_index;
    case NV_REG_CLUT_DATA:
        val = s->clut[s->clut_index];
        s->clut_index = (s->clut_index + 1) & 0xff;
        return val;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "nubus-video: bad register read "
                      "0x%" HWADDR_PRIx "\n", addr);
        return 0;
    }
}

static uint64_t nv_reg_read(void *opaque, hwaddr addr, unsigned size)
{
    NubusVideoState *s = opaque;
    uint64_t val = nv_reg_value(s, addr);

    trace_nubus_video_reg_read(addr, val);
    return val;
}

static void nv_reg_write(void *opaque, hwaddr addr, uint64_t val,
                         unsigned size)
{
    NubusVideoState *s = opaque;

    trace_nubus_video_reg_write(addr, val);

    switch (addr) {
    case NV_REG_MODE:
        if (val < s->ndepths) {
            s->mode = val;
            s->invalidate = true;
        }
        break;
    case NV_REG_INT_ENABLE:
        s->int_enable = val & NV_INT_VBL;
        if (s->int_enable) {
            if (!timer_pending(s->vbl_timer)) {
                nv_vbl_schedule(s);
            }
        } else {
            timer_del(s->vbl_timer);
        }
        nv_update_irq(s);
        break;
    case NV_REG_INT_STATUS:
        s->int_status &= ~val;
        nv_update_irq(s);
        break;
    case NV_REG_CLUT_INDEX:
        s->clut_index = val & 0xff;
        break;
    case NV_REG_CLUT_DATA:
        s->clut[s->clut_index++ & 0xff] = val & 0xffffff;
        s->clut_index &= 0xff;
        s->invalidate = true;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "nubus-video: bad register write "
                      "0x%" HWADDR_PRIx "\n", addr);
        break;
    }
}

static const MemoryRegionOps nv_reg_ops = {
    .read = nv_reg_read,
    .write = nv_reg_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

/* Device */

static void nubus_video_reset(DeviceState *dev)
{
    NubusVideoState *s = NUBUS_VIDEO(dev);
    int i;

    s->mode = 0;
    s->int_enable = 0;
    s->int_status = 0;
    s->clut_index = 0;
    timer_del(s->vbl_timer);
    for (i = 0; i < 256; i++) {
        s->clut[i] = (255 - i) * 0x010101;
    }
    s->clut[1] = 0;     /* 1 bpp: white and black */
    memset(s->vram_ptr, 0, s->vram_size);
    s->invalidate = true;
    nv_update_irq(s);
}

static void nubus_video_realize(DeviceState *dev, Error **errp)
{
    ERRP_GUARD();
    NubusVideoState *s = NUBUS_VIDEO(dev);
    NubusDevice *nd = NUBUS_DEVICE(dev);
    DeviceClass *pdc = DEVICE_CLASS(object_class_get_parent(
                                        object_get_class(OBJECT(dev))));
    bool size_ok = false;
    int i;

    if (nd->romfile) {
        error_setg(errp, "nubus-video makes its own declaration ROM, "
                   "'romfile' is not supported");
        return;
    }

    for (i = 0; i < ARRAY_SIZE(nv_sizes); i++) {
        size_ok |= nv_sizes[i].width == s->width &&
                   nv_sizes[i].height == s->height;
    }
    if (!size_ok) {
        error_setg(errp, "nubus-video: unsupported size %ux%u",
                   s->width, s->height);
        error_append_hint(errp, "Supported sizes:");
        for (i = 0; i < ARRAY_SIZE(nv_sizes); i++) {
            error_append_hint(errp, " %ux%u", nv_sizes[i].width,
                              nv_sizes[i].height);
        }
        error_append_hint(errp, "\n");
        return;
    }

    if (!s->vram_size) {
        /* enough for 32 bpp, at least 1 MiB */
        s->vram_size = MAX(MiB, pow2ceil(nv_frame_size(s, 5)));
        s->vram_size = MIN(s->vram_size, NV_VRAM_MAX);
    }
    if (!is_power_of_2(s->vram_size) || s->vram_size < 512 * KiB ||
        s->vram_size > NV_VRAM_MAX) {
        error_setg(errp, "nubus-video: vram-size must be a power of 2 "
                   "from 512 KiB to 8 MiB");
        return;
    }
    for (s->ndepths = 0; s->ndepths < NUBUS_VIDEO_NDEPTHS; s->ndepths++) {
        if (nv_frame_size(s, s->ndepths) > s->vram_size) {
            break;
        }
    }
    if (s->ndepths < 4) {
        error_setg(errp, "nubus-video: vram-size too small for 8 bpp");
        return;
    }

    pdc->realize(dev, errp);
    if (*errp) {
        return;
    }

    memory_region_init_ram(&s->vram, OBJECT(dev), "nubus-video.vram",
                           s->vram_size, errp);
    if (*errp) {
        return;
    }
    memory_region_set_log(&s->vram, true, DIRTY_MEMORY_VGA);
    s->vram_ptr = memory_region_get_ram_ptr(&s->vram);

    memory_region_init_io(&s->regs, OBJECT(dev), &nv_reg_ops, s,
                          "nubus-video.regs", NV_REGS_SIZE);
    memory_region_init_rom(&s->rom, OBJECT(dev), "nubus-video.decl-rom",
                           NV_ROM_SIZE, errp);
    if (*errp) {
        return;
    }
    nubus_video_build_rom(s, memory_region_get_ram_ptr(&s->rom),
                          NV_ROM_SIZE);

    /* the 24-bit window */
    memory_region_init(&s->low_window, OBJECT(dev), "nubus-video.window",
                       NV_WINDOW_SIZE);
    memory_region_init_alias(&s->vram_low, OBJECT(dev), "nubus-video.vram-low",
                             &s->vram, 0, MIN(s->vram_size, NV_VRAM_LOW_SIZE));
    memory_region_add_subregion(&s->low_window, 0, &s->vram_low);
    memory_region_add_subregion(&s->low_window, NV_REGS_OFFSET, &s->regs);
    memory_region_add_subregion(&s->low_window, NV_ROM_OFFSET, &s->rom);
    memory_region_add_subregion(&nd->slot_mem, 0, &s->low_window);

    memory_region_add_subregion(&nd->slot_mem, NV_VRAM_OFFSET, &s->vram);

    for (i = DIV_ROUND_UP(NV_VRAM_OFFSET + s->vram_size, NV_WINDOW_SIZE);
         i < NUBUS_SLOT_SIZE / NV_WINDOW_SIZE; i++) {
        memory_region_init_alias(&s->low_mirror[i - 1], OBJECT(dev),
                                 "nubus-video.window-mirror", &s->low_window,
                                 0, NV_WINDOW_SIZE);
        memory_region_add_subregion(&nd->slot_mem, i * NV_WINDOW_SIZE,
                                    &s->low_mirror[i - 1]);
    }

    s->vbl_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, nv_vbl, s);
    s->con = qemu_graphic_console_create(dev, 0, &nubus_video_ops, s);
    qemu_console_resize(s->con, s->width, s->height);
}

static int nubus_video_post_load(void *opaque, int version_id)
{
    NubusVideoState *s = opaque;

    s->invalidate = true;
    return 0;
}

static const VMStateDescription vmstate_nubus_video = {
    .name = "nubus-video",
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = nubus_video_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(mode, NubusVideoState),
        VMSTATE_UINT32(int_enable, NubusVideoState),
        VMSTATE_UINT32(int_status, NubusVideoState),
        VMSTATE_UINT32(clut_index, NubusVideoState),
        VMSTATE_UINT32_ARRAY(clut, NubusVideoState, 256),
        VMSTATE_TIMER_PTR(vbl_timer, NubusVideoState),
        VMSTATE_END_OF_LIST()
    }
};

static const Property nubus_video_properties[] = {
    DEFINE_PROP_UINT32("width", NubusVideoState, width, 640),
    DEFINE_PROP_UINT32("height", NubusVideoState, height, 480),
    DEFINE_PROP_SIZE32("vram-size", NubusVideoState, vram_size, 0),
};

static void nubus_video_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = nubus_video_realize;
    dc->desc = "QEMU NuBus video card";
    dc->vmsd = &vmstate_nubus_video;
    device_class_set_legacy_reset(dc, nubus_video_reset);
    device_class_set_props(dc, nubus_video_properties);
    set_bit(DEVICE_CATEGORY_DISPLAY, dc->categories);
}

static const TypeInfo nubus_video_info = {
    .name = TYPE_NUBUS_VIDEO,
    .parent = TYPE_NUBUS_DEVICE,
    .instance_size = sizeof(NubusVideoState),
    .class_init = nubus_video_class_init,
};

static void nubus_video_register_types(void)
{
    type_register_static(&nubus_video_info);
}

type_init(nubus_video_register_types)

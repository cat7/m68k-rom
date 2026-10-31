/*
 * Apple "New Age" floppy disk controller (Quadra 660AV/840AV)
 *
 * An 82077-style controller (command, execution and result phases through
 * a FIFO; main status register; sense interrupt) with Apple's commands for
 * the Sony drive's control lines and for GCR disks, whose sectors it reads
 * and writes already decoded. Data moves through the PSC's DMA channel 3.
 *
 * Registers sit every $40 on byte lane 1; the Mac OS driver only uses two:
 *   +$101  read: main status (RQM, DIO, CB); write: data rate select
 *   +$141  the FIFO
 *
 * Commands, as the ROM's driver uses them (bit 6 of a data command picks
 * MFM, clear for GCR):
 *   $03 specify, $13 configure, $12 perpendicular, $32 (setup)
 *   $07 recalibrate, $0F seek, $04 sense drive status, $08 sense interrupt
 *   $4A/$0A read ID, $46/$06 read, $45/$05 write, $4D/$0D format,
 *   $41/$01 format with data, $5E/$1E raw read (diagnostic: refused)
 *   $9B/$1B drive enable/disable, $9A/$1A motor on/off,
 *   $5C/$1C MFM/GCR mode, $52 eject
 *
 * The emulation is at the sector level: a READ ID reports the sectors of
 * the track in turn, as they pass under the head. Disk changes interrupt
 * with IC=3 (bit 1 set for a removal) once the driver is idle.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "hw/block/mac_newage.h"
#include "system/block-backend.h"
#include "system/blockdev.h"
#include "trace.h"

#define REG_MSR         4
#define REG_FIFO        5

#define MSR_RQM         0x80
#define MSR_DIO         0x40
#define MSR_CB          0x10

#define DSR_SWRESET     0x80

#define ST0_IC_ABNORMAL 0x40
#define ST0_IC_INVALID  0x80
#define ST0_IC_CHANGE   0xc0
#define ST0_SE          0x20
#define ST0_NR          0x08
#define ST0_REMOVED     0x02    /* with IC=3: the disk went out */

#define ST1_MA          0x01
#define ST1_NW          0x02
#define ST1_ND          0x04
#define ST1_OR          0x10

#define CMD_MFM         0x40

#define IDLE_NS         (300 * SCALE_MS)    /* quiet before a change event */
#define EJECT_NS        (500 * SCALE_MS)

/* sense drive status: a SuperDrive (bits 3, 2); media and write enable */
#define SDS_FDHD        0x0c
#define SDS_HD_MEDIA    0x02
#define SDS_WRITABLE    0x40
#define SDS_DD_MEDIA    0x80
#define SDS_NO_DRIVE    0xff

/* GCR: five zones of 16 cylinders */
static unsigned gcr_spt(unsigned cyl)
{
    return 12 - MIN(cyl / 16, 4);
}

static unsigned fmt_heads(int fmt)
{
    return fmt == NEWAGE_FMT_GCR400 ? 1 : 2;
}

static bool fmt_mfm(int fmt)
{
    return fmt == NEWAGE_FMT_MFM720 || fmt == NEWAGE_FMT_MFM1440;
}

static unsigned fmt_spt(int fmt, unsigned cyl)
{
    switch (fmt) {
    case NEWAGE_FMT_MFM720:
        return 9;
    case NEWAGE_FMT_MFM1440:
        return 18;
    default:
        return gcr_spt(cyl);
    }
}

/* the header's format byte (GCR) or size code (MFM) */
static uint8_t fmt_n(int fmt)
{
    switch (fmt) {
    case NEWAGE_FMT_GCR400:
        return 0x02;
    case NEWAGE_FMT_GCR800:
        return 0x22;
    default:
        return 0x02;
    }
}

/* the block of sector @r (0-based) on cylinder @c, head @h */
static uint32_t fmt_lba(int fmt, unsigned c, unsigned h, unsigned r)
{
    uint32_t lba = 0;
    unsigned i;

    if (fmt_mfm(fmt)) {
        return (c * 2 + h) * fmt_spt(fmt, c) + r;
    }
    for (i = 0; i < c; i++) {
        lba += gcr_spt(i) * fmt_heads(fmt);
    }
    return lba + h * gcr_spt(c) + r;
}

static void newage_update_irq(MacNewAgeState *s)
{
    qemu_set_irq(s->irq, s->res_irq || s->sense_n);
}

static void newage_push_sense(MacNewAgeState *s, uint8_t st0, uint8_t pcn)
{
    if (s->sense_n < ARRAY_SIZE(s->sense_st0)) {
        s->sense_st0[s->sense_n] = st0;
        s->sense_pcn[s->sense_n] = pcn;
        s->sense_n++;
    }
    newage_update_irq(s);
}

static void newage_result(MacNewAgeState *s, const uint8_t *res, int n,
                          bool irq)
{
    memcpy(s->res, res, n);
    s->res_len = n;
    s->res_pos = 0;
    s->res_irq = irq;
    trace_mac_newage_result(s->cmd[0], n, res[0], n > 1 ? res[1] : 0,
                            n > 2 ? res[2] : 0);
    newage_update_irq(s);
}

static MacNewAgeDrive *newage_drive(MacNewAgeState *s, uint8_t sel)
{
    unsigned unit = sel & 3;

    return unit < MAC_NEWAGE_MAX_FD ? s->drives[unit] : NULL;
}

static bool newage_ready(MacNewAgeDrive *d)
{
    return d && d->disk_in && d->format != NEWAGE_FMT_NONE;
}

/* the disk in a drive: its format from the image's size */
static void newage_load(MacNewAgeDrive *d)
{
    BlockBackend *blk = d->conf.blk;
    int64_t len;

    d->disk_in = blk && blk_is_inserted(blk);
    d->format = NEWAGE_FMT_NONE;
    d->next_sector = 0;
    if (!d->disk_in) {
        return;
    }
    d->write_protected = !blk_is_writable(blk);
    len = blk_getlength(blk);
    switch (len) {
    case 400 * 1024:
        d->format = NEWAGE_FMT_GCR400;
        break;
    case 800 * 1024:
        d->format = NEWAGE_FMT_GCR800;
        break;
    case 720 * 1024:
        d->format = NEWAGE_FMT_MFM720;
        break;
    case 1440 * 1024:
        d->format = NEWAGE_FMT_MFM1440;
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "mac-newage: unit %d: %" PRId64
                      "-byte image is no floppy format\n", d->unit, len);
        break;
    }
}

/* the end of a data command: ST0-ST2, C, H, R, N */
static void newage_data_result(MacNewAgeState *s, uint8_t st0, uint8_t st1,
                               uint8_t c, uint8_t h, uint8_t r, uint8_t n)
{
    uint8_t res[7] = { st0, st1, 0, c, h, r, n };

    newage_result(s, res, 7, true);
}

/* does the command's mode match the disk? */
static bool newage_mode_ok(MacNewAgeDrive *d, uint8_t cmd)
{
    return !!(cmd & CMD_MFM) == fmt_mfm(d->format);
}

static void newage_read_id(MacNewAgeState *s)
{
    MacNewAgeDrive *d = newage_drive(s, s->cmd[1]);
    uint8_t hd = s->cmd[1] & 7, h = (s->cmd[1] >> 2) & 1;
    unsigned spt, r;

    if (!newage_ready(d)) {
        newage_data_result(s, ST0_IC_ABNORMAL | ST0_NR | hd, 0, 0, h, 0, 0);
        return;
    }
    if (!newage_mode_ok(d, s->cmd[0]) || h >= fmt_heads(d->format)) {
        newage_data_result(s, ST0_IC_ABNORMAL | hd, ST1_MA, d->cyl, h, 0, 0);
        return;
    }
    spt = fmt_spt(d->format, d->cyl);
    r = d->next_sector % spt;
    d->next_sector = (r + 1) % spt;
    newage_data_result(s, hd, 0, d->cyl, h, fmt_mfm(d->format) ? r + 1 : r,
                       fmt_n(d->format));
}

/* READ DATA and WRITE DATA: sectors R..EOT of the track */
static void newage_rw(MacNewAgeState *s, bool write)
{
    MacNewAgeDrive *d = newage_drive(s, s->cmd[1]);
    uint8_t hd = s->cmd[1] & 7;
    uint8_t c = s->cmd[2], h = s->cmd[3], r = s->cmd[4], n = s->cmd[5];
    uint8_t eot = s->cmd[6];
    unsigned first = r, last = eot, spt, base, count, len, moved;
    uint8_t st1 = 0;

    if (!newage_ready(d)) {
        newage_data_result(s, ST0_IC_ABNORMAL | ST0_NR | hd, 0, c, h, r, n);
        return;
    }
    if (!newage_mode_ok(d, s->cmd[0]) || h >= fmt_heads(d->format)) {
        newage_data_result(s, ST0_IC_ABNORMAL | hd, ST1_MA, c, h, r, n);
        return;
    }
    spt = fmt_spt(d->format, c);
    if (fmt_mfm(d->format)) {
        first--;
        last--;
    }
    if (first > last || last >= spt) {
        newage_data_result(s, ST0_IC_ABNORMAL | hd, ST1_ND, c, h, r, n);
        return;
    }
    if (write && d->write_protected) {
        newage_data_result(s, ST0_IC_ABNORMAL | hd, ST1_NW, c, h, r, n);
        return;
    }
    count = last - first + 1;
    len = count * 512;
    base = fmt_lba(d->format, c, h, first);
    trace_mac_newage_rw(d->unit, write, c, h, first, count);
    if (write) {
        moved = s->dma ? s->dma(s->dma_opaque, s->buf, len, false) : 0;
        if (moved < len) {
            st1 = ST1_OR;
            len = moved & ~511;
        }
        if (len && blk_pwrite(d->conf.blk, base * 512LL, len, s->buf, 0) < 0) {
            st1 |= ST1_ND;
        }
    } else {
        if (blk_pread(d->conf.blk, base * 512LL, len, s->buf, 0) < 0) {
            st1 = ST1_ND;
        } else {
            moved = s->dma ? s->dma(s->dma_opaque, s->buf, len, true) : 0;
            if (moved < len) {
                st1 = ST1_OR;
            }
        }
    }
    d->cyl = c;
    d->next_sector = (last + 1) % spt;
    newage_data_result(s, (st1 ? ST0_IC_ABNORMAL : 0) | hd, st1, c, h,
                       fmt_mfm(d->format) ? last + 2 : last + 1, n);
}

/*
 * FORMAT ($4D/$0D: ID fields by DMA, the sectors filled with D) and FORMAT
 * WITH DATA ($41/$01: ID field and 512 bytes a sector)
 */
static void newage_format(MacNewAgeState *s, bool with_data)
{
    MacNewAgeDrive *d = newage_drive(s, s->cmd[1]);
    uint8_t hd = s->cmd[1] & 7, h = (s->cmd[1] >> 2) & 1;
    uint8_t n = s->cmd[2], sc = s->cmd[3], fill = with_data ? 0 : s->cmd[5];
    unsigned per = with_data ? 516 : 4, i, spt, r;
    uint8_t ids[4 * 32];
    uint8_t *sec;
    uint8_t st1 = 0;

    if (!newage_ready(d)) {
        newage_data_result(s, ST0_IC_ABNORMAL | ST0_NR | hd, 0, d ? d->cyl : 0,
                           h, 0, n);
        return;
    }
    if (d->write_protected) {
        newage_data_result(s, ST0_IC_ABNORMAL | hd, ST1_NW, d->cyl, h, 0, n);
        return;
    }
    if (!newage_mode_ok(d, s->cmd[0]) || h >= fmt_heads(d->format) ||
        sc > 32 || sc * per > sizeof(s->buf)) {
        newage_data_result(s, ST0_IC_ABNORMAL | hd, ST1_MA, d->cyl, h, 0, n);
        return;
    }
    if (!s->dma || s->dma(s->dma_opaque, s->buf, sc * per, false) < sc * per) {
        newage_data_result(s, ST0_IC_ABNORMAL | hd, ST1_OR, d->cyl, h, 0, n);
        return;
    }
    spt = fmt_spt(d->format, d->cyl);
    trace_mac_newage_format(d->unit, d->cyl, h, sc, with_data);
    for (i = 0; i < sc; i++) {
        memcpy(ids + i * 4, s->buf + i * per, 4);
    }
    for (i = 0; i < sc; i++) {
        r = ids[i * 4 + 2];
        if (fmt_mfm(d->format)) {
            r--;
        }
        if (r >= spt) {
            continue;       /* GCR's first pass numbers every sector $3F */
        }
        if (with_data) {
            sec = s->buf + i * per + 4;
        } else {
            sec = s->buf + sizeof(s->buf) - 512;
            memset(sec, fill, 512);
        }
        if (blk_pwrite(d->conf.blk,
                       fmt_lba(d->format, d->cyl, h, r) * 512LL, 512, sec,
                       0) < 0) {
            st1 = ST1_ND;
        }
    }
    newage_data_result(s, (st1 ? ST0_IC_ABNORMAL : 0) | hd, st1, d->cyl, h,
                       0, n);
}

static uint8_t newage_drive_status(MacNewAgeState *s, uint8_t sel)
{
    MacNewAgeDrive *d = newage_drive(s, sel);
    uint8_t st = SDS_FDHD;

    if (!d) {
        return SDS_NO_DRIVE;
    }
    if (!d->disk_in) {
        return st | SDS_DD_MEDIA;
    }
    st |= d->format == NEWAGE_FMT_MFM1440 ? SDS_HD_MEDIA : SDS_DD_MEDIA;
    if (!d->write_protected) {
        st |= SDS_WRITABLE;
    }
    return st;
}

static void newage_eject_done(void *opaque)
{
    MacNewAgeState *s = opaque;
    MacNewAgeDrive *d = newage_drive(s, s->eject_unit);

    if (d) {
        if (d->conf.blk && blk_is_inserted(d->conf.blk)) {
            blk_eject(d->conf.blk, true);
        }
        d->disk_in = false;
        d->format = NEWAGE_FMT_NONE;
        s->change[d->unit] = 0;
    }
    newage_push_sense(s, s->eject_unit, 0);
}

static void newage_execute(MacNewAgeState *s)
{
    uint8_t cmd = s->cmd[0], sel = s->cmd[1], res[1];
    MacNewAgeDrive *d;

    trace_mac_newage_command(cmd, s->cmd_len, sel);
    s->cmd_len = s->cmd_pos = 0;
    switch (cmd) {
    case 0x03:          /* specify */
    case 0x13:          /* configure */
    case 0x12:          /* perpendicular mode */
    case 0x32:          /* set up the drive lines */
    case 0x1b:          /* drive disable */
    case 0x1a:          /* motor off */
        break;
    case 0x9b:          /* drive enable */
    case 0x9a:          /* motor on */
    case 0x5c:          /* MFM mode */
    case 0x1c:          /* GCR mode */
        newage_push_sense(s, sel & 3, 0);
        break;
    case 0x07:          /* recalibrate */
    case 0x0f:          /* seek */
        d = newage_drive(s, sel);
        if (!d) {
            newage_push_sense(s, ST0_IC_ABNORMAL | ST0_SE | ST0_NR | (sel & 3),
                              0);
            break;
        }
        d->cyl = cmd == 0x07 ? 0 : MIN(s->cmd[2], 79);
        newage_push_sense(s, ST0_SE | (sel & 3), d->cyl);
        break;
    case 0x04:          /* sense drive status */
        res[0] = newage_drive_status(s, sel);
        newage_result(s, res, 1, false);
        break;
    case 0x08:          /* sense interrupt */
        if (s->sense_n) {
            uint8_t r2[2] = { s->sense_st0[0], s->sense_pcn[0] };

            s->sense_n--;
            memmove(s->sense_st0, s->sense_st0 + 1, s->sense_n);
            memmove(s->sense_pcn, s->sense_pcn + 1, s->sense_n);
            /* PCN only after a seek: the driver reads no more */
            newage_result(s, r2, (r2[0] & ST0_SE) ? 2 : 1, false);
        } else {
            res[0] = ST0_IC_INVALID;
            newage_result(s, res, 1, false);
        }
        break;
    case 0x52:          /* eject */
        s->eject_unit = sel & 3;
        timer_mod(s->eject_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + EJECT_NS);
        break;
    case 0x4a:
    case 0x0a:
        newage_read_id(s);
        break;
    case 0x46:
    case 0x06:
        newage_rw(s, false);
        break;
    case 0x45:
    case 0x05:
        newage_rw(s, true);
        break;
    case 0x4d:
    case 0x0d:
        newage_format(s, false);
        break;
    case 0x41:
    case 0x01:
        newage_format(s, true);
        break;
    case 0x5e:
    case 0x1e:          /* raw read: not here */
        newage_data_result(s, ST0_IC_ABNORMAL | (sel & 7), ST1_MA, s->cmd[2],
                           s->cmd[3], s->cmd[4], s->cmd[5]);
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "mac-newage: command 0x%02x\n", cmd);
        res[0] = ST0_IC_INVALID;
        newage_result(s, res, 1, false);
        break;
    }
}

/* the command's length, with its opcode; 1 for an unknown one */
static unsigned newage_cmd_len(uint8_t cmd)
{
    switch (cmd) {
    case 0x08:
        return 1;
    case 0x12: case 0x32: case 0x07: case 0x04:
    case 0x9b: case 0x1b: case 0x9a: case 0x1a:
    case 0x5c: case 0x1c: case 0x52: case 0x4a: case 0x0a:
        return 2;
    case 0x03: case 0x0f:
        return 3;
    case 0x13:
        return 4;
    case 0x41: case 0x01:
        return 5;
    case 0x4d: case 0x0d:
        return 6;
    case 0x5e: case 0x1e:
        return 8;
    case 0x46: case 0x06: case 0x45: case 0x05:
        return 9;
    }
    return 1;
}

static void newage_soft_reset(MacNewAgeState *s)
{
    s->cmd_len = s->cmd_pos = 0;
    s->res_len = s->res_pos = 0;
    s->res_irq = false;
    s->sense_n = 0;
    timer_del(s->eject_timer);
    /* like the 82077's, after a reset: one interrupt */
    newage_push_sense(s, ST0_IC_CHANGE | ST0_REMOVED, 0);
}

static void newage_poke_idle(MacNewAgeState *s)
{
    int i;

    for (i = 0; i < MAC_NEWAGE_MAX_FD; i++) {
        if (s->change[i]) {
            timer_mod(s->idle_timer,
                      qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + IDLE_NS);
            return;
        }
    }
}

/* quiet for a while: tell the driver about a disk change */
static void newage_idle(void *opaque)
{
    MacNewAgeState *s = opaque;
    int i;

    if (s->cmd_pos || s->res_len || s->sense_n) {
        newage_poke_idle(s);
        return;
    }
    for (i = 0; i < MAC_NEWAGE_MAX_FD; i++) {
        if (s->change[i]) {
            trace_mac_newage_change(i, s->change[i] == 1);
            newage_push_sense(s, ST0_IC_CHANGE | i |
                              (s->change[i] == 1 ? 0 : ST0_REMOVED), 0);
            s->change[i] = 0;
            newage_poke_idle(s);
            return;
        }
    }
}

static uint8_t newage_msr(MacNewAgeState *s)
{
    if (s->res_pos < s->res_len) {
        return MSR_RQM | MSR_DIO | MSR_CB;
    }
    return MSR_RQM | (s->cmd_pos ? MSR_CB : 0);
}

static uint64_t newage_read(void *opaque, hwaddr addr, unsigned size)
{
    MacNewAgeState *s = opaque;
    uint8_t val = 0;

    if ((addr & 0x3f) != 1) {
        return 0;
    }
    switch (addr >> 6) {
    case REG_MSR:
        val = newage_msr(s);
        break;
    case REG_FIFO:
        if (s->res_pos < s->res_len) {
            val = s->res[s->res_pos++];
            if (s->res_irq) {
                s->res_irq = false;
                newage_update_irq(s);
            }
            if (s->res_pos == s->res_len) {
                s->res_len = s->res_pos = 0;
            }
        }
        break;
    }
    trace_mac_newage_read(addr, val);
    return val;
}

static void newage_write(void *opaque, hwaddr addr, uint64_t val,
                         unsigned size)
{
    MacNewAgeState *s = opaque;

    if ((addr & 0x3f) != 1) {
        return;
    }
    trace_mac_newage_write(addr, val);
    newage_poke_idle(s);
    switch (addr >> 6) {
    case REG_MSR:               /* data rate select */
        s->dsr = val & ~DSR_SWRESET;
        if (val & DSR_SWRESET) {
            newage_soft_reset(s);
        }
        break;
    case REG_FIFO:
        if (s->res_len) {
            /* a new command ends an unread result */
            s->res_len = s->res_pos = 0;
            s->res_irq = false;
            newage_update_irq(s);
        }
        if (!s->cmd_pos) {
            s->cmd_len = newage_cmd_len(val);
        }
        s->cmd[s->cmd_pos++] = val;
        if (s->cmd_pos >= s->cmd_len) {
            newage_execute(s);
        }
        break;
    }
}

static const MemoryRegionOps newage_ops = {
    .read = newage_read,
    .write = newage_write,
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

/* --- the drives ------------------------------------------------------- */

static void newage_change_cb(void *opaque, bool load, Error **errp)
{
    MacNewAgeDrive *d = opaque;
    MacNewAgeState *s = d->ctrl;

    if (!load) {
        blk_set_perm(d->conf.blk, 0, BLK_PERM_ALL, &error_abort);
    } else if (!blkconf_apply_backend_options(&d->conf,
                   !blk_supports_write_perm(d->conf.blk), false, errp)) {
        return;
    }
    newage_load(d);
    s->change[d->unit] = d->disk_in ? 1 : 2;
    newage_poke_idle(s);
}

static const BlockDevOps newage_block_ops = {
    .change_media_cb = newage_change_cb,
};

static void newage_drive_realize(DeviceState *dev, Error **errp)
{
    MacNewAgeDrive *d = MAC_NEWAGE_DRIVE(dev);
    MacNewAgeState *s = MAC_NEWAGE(dev->parent_bus->parent);

    if (d->unit < 0 || d->unit >= MAC_NEWAGE_MAX_FD) {
        error_setg(errp, "unit %d out of range (0-%d)", d->unit,
                   MAC_NEWAGE_MAX_FD - 1);
        return;
    }
    if (s->drives[d->unit]) {
        error_setg(errp, "floppy unit %d already in use", d->unit);
        return;
    }
    if (!d->conf.blk) {
        d->conf.blk = blk_new(qemu_get_aio_context(), 0, BLK_PERM_ALL);
        if (blk_attach_dev(d->conf.blk, dev) < 0) {
            error_setg(errp, "cannot attach the drive");
            return;
        }
    }
    if (!blkconf_blocksizes(&d->conf, errp)) {
        return;
    }
    if (d->conf.logical_block_size != 512 ||
        d->conf.physical_block_size != 512) {
        error_setg(errp, "physical and logical block size must be 512 for"
                   " floppy");
        return;
    }
    d->conf.rerror = BLOCKDEV_ON_ERROR_AUTO;
    d->conf.werror = BLOCKDEV_ON_ERROR_AUTO;
    if (!blkconf_apply_backend_options(&d->conf,
            !blk_supports_write_perm(d->conf.blk), false, errp)) {
        return;
    }
    if (!blk_is_inserted(d->conf.blk)) {
        /* an empty drive asks for nothing, so a locked disk can go in */
        blk_set_perm(d->conf.blk, 0, BLK_PERM_ALL, &error_abort);
    }
    d->ctrl = s;
    s->drives[d->unit] = d;
    blk_set_dev_ops(d->conf.blk, &newage_block_ops, d);
    newage_load(d);
}

static void newage_drive_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    static const Property props[] = {
        DEFINE_BLOCK_PROPERTIES(MacNewAgeDrive, conf),
        DEFINE_PROP_INT32("unit", MacNewAgeDrive, unit, -1),
    };

    dc->bus_type = TYPE_MAC_NEWAGE_BUS;
    dc->realize = newage_drive_realize;
    dc->desc = "Apple SuperDrive on the New Age floppy controller";
    device_class_set_props(dc, props);
    set_bit(DEVICE_CATEGORY_STORAGE, dc->categories);
}

/* --- the controller ---------------------------------------------------- */

static void newage_reset_hold(Object *obj, ResetType type)
{
    MacNewAgeState *s = MAC_NEWAGE(obj);
    int i;

    s->cmd_len = s->cmd_pos = 0;
    s->res_len = s->res_pos = 0;
    s->res_irq = false;
    s->sense_n = 0;
    s->dsr = 0;
    timer_del(s->eject_timer);
    timer_del(s->idle_timer);
    for (i = 0; i < MAC_NEWAGE_MAX_FD; i++) {
        s->change[i] = 0;
        if (s->drives[i]) {
            newage_load(s->drives[i]);
            s->drives[i]->cyl = 0;
            /* a disk in at power on: inserted, once the driver is up */
            s->change[i] = s->drives[i]->disk_in ? 1 : 0;
        }
    }
    newage_update_irq(s);
}

static void newage_init(Object *obj)
{
    MacNewAgeState *s = MAC_NEWAGE(obj);

    memory_region_init_io(&s->mmio, obj, &newage_ops, s, "mac-newage",
                          0x2000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->idle_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, newage_idle, s);
    s->eject_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, newage_eject_done, s);
}

static void newage_realize(DeviceState *dev, Error **errp)
{
    MacNewAgeState *s = MAC_NEWAGE(dev);
    DriveInfo *dinfo;
    DeviceState *drive;

    s->bus = qbus_new(TYPE_MAC_NEWAGE_BUS, dev, NULL);
    /* the 840AV has the one internal drive */
    dinfo = drive_get(IF_FLOPPY, 0, 0);
    drive = qdev_new(TYPE_MAC_NEWAGE_DRIVE);
    qdev_prop_set_int32(drive, "unit", 0);
    if (dinfo) {
        qdev_prop_set_drive_err(drive, "drive", blk_by_legacy_dinfo(dinfo),
                                &error_fatal);
    }
    qdev_realize_and_unref(drive, s->bus, &error_fatal);
}

static void newage_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->realize = newage_realize;
    dc->desc = "Apple New Age floppy disk controller";
    rc->phases.hold = newage_reset_hold;
    set_bit(DEVICE_CATEGORY_STORAGE, dc->categories);
}

static void newage_bus_class_init(ObjectClass *oc, const void *data)
{
    BusClass *bc = BUS_CLASS(oc);

    bc->max_dev = MAC_NEWAGE_MAX_FD;
}

static const TypeInfo newage_types[] = {
    {
        .name = TYPE_MAC_NEWAGE,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(MacNewAgeState),
        .instance_init = newage_init,
        .class_init = newage_class_init,
    }, {
        .name = TYPE_MAC_NEWAGE_BUS,
        .parent = TYPE_BUS,
        .class_init = newage_bus_class_init,
    }, {
        .name = TYPE_MAC_NEWAGE_DRIVE,
        .parent = TYPE_DEVICE,
        .instance_size = sizeof(MacNewAgeDrive),
        .class_init = newage_drive_class_init,
    },
};

DEFINE_TYPES(newage_types)

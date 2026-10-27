/*
 * Apple I/O Processor (IOP)
 *
 * The Macintosh IIfx, Quadra 900 and Quadra 950 put the SCC and the SWIM
 * floppy controller (which also runs ADB) behind IOPs: 65C02 based
 * controllers with 32 KiB of RAM shared with the host. The host reaches
 * that RAM through an address register and an auto-incrementing data
 * port, and the two sides talk through message channels in it:
 *
 *   0x200        highest host-to-IOP ("send") channel
 *   0x201+n      state of send channel n
 *   0x220+32n    message of send channel n (the reply overwrites it)
 *   0x300        highest IOP-to-host ("receive") channel
 *   0x301+n      state of receive channel n
 *   0x31f        alive flag, set to 0xff by the running IOP
 *   0x320+32n    message of receive channel n
 *
 * Channel states are idle (0), new (1), received (2) and complete (3).
 * The host signals the IOP by writing IRQ to the status register; the
 * IOP signals the host with INT0 (a send message completed) and INT1
 * (a new message to receive).
 *
 * The host downloads 6502 code into the RAM before starting the IOP.
 * This model does not run it; it implements the message protocol
 * directly ("high-level emulation"): ADB on channel 2 and the floppy
 * drives on channel 1 of the SWIM IOP.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/mac_iop.h"
#include "migration/vmstate.h"
#include "qapi/error.h"
#include "system/address-spaces.h"
#include "system/block-backend.h"
#include "hw/core/boards.h"
#include "trace.h"

/* host registers (byte wide, on even addresses) */
#define IOP_REG_ADDR_HI     0x0
#define IOP_REG_ADDR_LO     0x2
#define IOP_REG_STATUS      0x4
#define IOP_REG_DATA        0x8

/* status/control register */
#define IOP_BYPASS          0x01
#define IOP_AUTOINC         0x02
#define IOP_RUN             0x04
#define IOP_IRQ             0x08
#define IOP_INT0            0x10
#define IOP_INT1            0x20
#define IOP_HWINT           0x40
#define IOP_DMAINACTIVE     0x80

#define IOP_ADDR_MAX_SEND_CHAN  0x0200
#define IOP_ADDR_SEND_STATE     0x0201
#define IOP_ADDR_PATCH_CTRL     0x021f
#define IOP_ADDR_SEND_MSG       0x0220
#define IOP_ADDR_MAX_RECV_CHAN  0x0300
#define IOP_ADDR_RECV_STATE     0x0301
#define IOP_ADDR_ALIVE          0x031f
#define IOP_ADDR_RECV_MSG       0x0320

#define IOP_MSG_IDLE        0
#define IOP_MSG_NEW         1
#define IOP_MSG_RCVD        2
#define IOP_MSG_COMPLETE    3

static void mac_iop_update_irq(MacIOPState *s)
{
    qemu_set_irq(s->irq, !!(s->status & (IOP_INT0 | IOP_INT1)));
}

/*
 * Raise INT0 or INT1. The SWIM IOP's line goes to an edge-triggered VIA
 * input, so every new interrupt makes a fresh edge even if the host has
 * not acknowledged the previous one yet.
 */
static void mac_iop_raise(MacIOPState *s, uint8_t bit)
{
    s->status |= bit;
    qemu_irq_lower(s->irq);
    qemu_irq_raise(s->irq);
}

static bool mac_iop_running(MacIOPState *s)
{
    return s->status & IOP_RUN;
}

/*
 * IOP-to-host messages wait in a small queue per channel until the host
 * has taken the previous one.
 */
static void mac_iop_deliver(MacIOPState *s)
{
    int chan;

    for (chan = 0; chan < MAC_IOP_NUM_CHAN; chan++) {
        uint8_t *state = &s->ram[IOP_ADDR_RECV_STATE + chan];
        MacIOPQueue *q = &s->recv_queue[chan];

        if (*state == IOP_MSG_IDLE && q->count) {
            uint8_t *msg = &s->ram[IOP_ADDR_RECV_MSG + chan * MAC_IOP_MSG_LEN];

            memcpy(msg, q->msg[q->head], MAC_IOP_MSG_LEN);
            q->head = (q->head + 1) % MAC_IOP_QUEUE_LEN;
            q->count--;
            trace_mac_iop_recv(s->kind, chan, msg[0], msg[1], msg[2], msg[3]);
            *state = IOP_MSG_NEW;
            mac_iop_raise(s, IOP_INT1);
        }
    }
    mac_iop_update_irq(s);
}

static void mac_iop_queue_recv(MacIOPState *s, int chan, const uint8_t *msg)
{
    MacIOPQueue *q = &s->recv_queue[chan];

    if (q->count == MAC_IOP_QUEUE_LEN) {
        qemu_log_mask(LOG_GUEST_ERROR, "mac-iop %d: channel %d overflow\n",
                      s->kind, chan);
        return;
    }
    memcpy(q->msg[(q->head + q->count) % MAC_IOP_QUEUE_LEN], msg,
           MAC_IOP_MSG_LEN);
    q->count++;
    mac_iop_deliver(s);
}

/*
 * ADB on the SWIM IOP, channel 2. Both directions use
 *
 *   flags, count, command, data[count]
 *
 * The host sends explicit commands and the autopoll device mask; the
 * IOP answers each explicit command, and reports autopoll data, with a
 * message on the receive side of the same channel.
 */
#define IOP_ADB_CHAN            2

#define ADB_IOP_EXPLICIT        0x80
#define ADB_IOP_AUTOPOLL        0x40
#define ADB_IOP_SET_AUTOPOLL    0x20
#define ADB_IOP_SRQ             0x04
#define ADB_IOP_TIMEOUT         0x02

static void mac_iop_adb_send(MacIOPState *s, uint8_t *msg)
{
    uint8_t reply[MAC_IOP_MSG_LEN] = { 0 };
    uint8_t flags = msg[0];
    int count = MIN(msg[1], 8);
    int olen;

    if (flags & ADB_IOP_SET_AUTOPOLL) {
        /* the Mac OS driver stores the device mask as a big endian word */
        uint16_t mask = (msg[3] << 8) | msg[4];

        s->adb_autopoll = flags & ADB_IOP_AUTOPOLL;
        adb_set_autopoll_mask(&s->adb_bus, mask);
        adb_set_autopoll_enabled(&s->adb_bus, s->adb_autopoll && mask);
        return;
    }
    if (!(flags & ADB_IOP_EXPLICIT)) {
        return;
    }

    adb_autopoll_block(&s->adb_bus);
    olen = adb_request(&s->adb_bus, reply + 3, &msg[2], count + 1);
    adb_autopoll_unblock(&s->adb_bus);

    reply[0] = ADB_IOP_EXPLICIT | (s->adb_autopoll ? ADB_IOP_AUTOPOLL : 0);
    if (olen < 0) {
        reply[0] |= ADB_IOP_TIMEOUT;
        olen = 0;
    }
    reply[1] = olen;
    reply[2] = msg[2];
    mac_iop_queue_recv(s, IOP_ADB_CHAN, reply);
}

static void mac_iop_adb_poll(void *opaque)
{
    MacIOPState *s = opaque;
    uint8_t msg[MAC_IOP_MSG_LEN] = { 0 };
    int olen;

    /* one report at a time, like the real thing */
    if (s->recv_queue[IOP_ADB_CHAN].count ||
        s->ram[IOP_ADDR_RECV_STATE + IOP_ADB_CHAN] != IOP_MSG_IDLE) {
        return;
    }
    olen = adb_poll(&s->adb_bus, msg + 2, s->adb_bus.autopoll_mask);
    if (olen > 0) {
        msg[0] = ADB_IOP_AUTOPOLL;
        msg[1] = olen - 1;
        mac_iop_queue_recv(s, IOP_ADB_CHAN, msg);
    }
}


/*
 * Floppy drives on the SWIM IOP, channel 1, as the Mac OS ".Sony" driver
 * for IOP machines uses them. A request is
 *
 *   +0 kind, +1 drive number, +2 error (OSErr, the reply's)
 *
 * and the IOP answers in place. Reads and writes carry the host buffer
 * at +4, the first block at +8, the block count at +12 and file tags at
 * +16; the IOP moves the data to or from Mac memory itself. Drive status
 * replies, and the insert messages the IOP sends on its own channel 1,
 * carry the Sony driver's status fields:
 *
 *   +4  track (word)          +6 write protected ($FF), disk in place,
 *                                installed, sides ($FF: two)
 *   +10 two-sided format, needs flush, disk errors (word)
 *   +14 drive info (long)     +22 format of the disk (bit mask)
 *   +24 formats the drive takes: bit 0 HD20, 1 400K, 2 800K, 3 720K,
 *       4 1440K
 */
#define IOP_FD_CHAN             1

enum {
    FD_INIT = 1,
    FD_SHUTDOWN,
    FD_START_POLL,
    FD_STOP_POLL,
    FD_SET_TAG_ADDR,
    FD_STATUS,
    FD_EJECT,
    FD_FORMAT,
    FD_FORMAT_VERIFY,
    FD_WRITE,
    FD_READ,
    FD_READ_VERIFY,
};

/* the IOP's insert message */
#define FD_EVT_INSERTED         1

#define FD_DRIVE_SUPERDRIVE     4       /* FDHD: 400K to 1440K */
#define FD_FORMATS_SUPERDRIVE   0x1e

#define OS_NOERR                0
#define OS_PARAMERR             (-50)
#define OS_WPRERR               (-44)
#define OS_OFFLINERR            (-65)
#define OS_NSDRVERR             (-56)
#define OS_IOERR                (-36)
#define OS_DATAVERERR           (-68)

static void put_be16(uint8_t *p, uint16_t v)
{
    p[0] = v >> 8;
    p[1] = v;
}

static void put_be32(uint8_t *p, uint32_t v)
{
    put_be16(p, v >> 16);
    put_be16(p + 2, v);
}

static uint32_t get_be32(const uint8_t *p)
{
    return (p[0] << 24) | (p[1] << 16) | (p[2] << 8) | p[3];
}

/* drive n (1, 2) to index, or -1 */
static int mac_iop_fd_index(MacIOPState *s, int drive)
{
    int i = drive - 1;

    if (i < 0 || i >= MAC_IOP_NUM_FD || (i > 0 && !s->fd_blk[i])) {
        return -1;
    }
    return i;
}

static bool mac_iop_fd_has_disk(MacIOPState *s, int i)
{
    return s->fd_in[i] && s->fd_blk[i] && blk_is_inserted(s->fd_blk[i]);
}

static uint32_t mac_iop_fd_blocks(MacIOPState *s, int i)
{
    return mac_iop_fd_has_disk(s, i) ? blk_getlength(s->fd_blk[i]) / 512 : 0;
}

/* the format bit of a disk of @blocks */
static uint16_t mac_iop_fd_format(uint32_t blocks)
{
    switch (blocks) {
    case 800:
        return BIT(1);
    case 1600:
        return BIT(2);
    case 1440:
        return BIT(3);
    case 2880:
        return BIT(4);
    }
    return 0;
}

/*
 * The formats the disk in the drive takes: a SuperDrive formats
 * double-density disks 400K, 800K or 720K, and high-density ones 1440K
 */
static uint16_t mac_iop_fd_media_formats(uint32_t blocks)
{
    if (blocks == 2880) {
        return BIT(4);
    }
    return blocks ? BIT(1) | BIT(2) | BIT(3) : FD_FORMATS_SUPERDRIVE;
}

static void mac_iop_fd_status(MacIOPState *s, int i, uint8_t *msg)
{
    uint32_t blocks = mac_iop_fd_blocks(s, i);
    bool disk = blocks != 0;

    memset(msg + 4, 0, MAC_IOP_MSG_LEN - 4);
    msg[6] = disk && !blk_is_writable(s->fd_blk[i]) ? 0xff : 0;
    msg[7] = disk ? 2 : 0;
    msg[8] = 1;
    msg[9] = 0xff;
    msg[10] = disk && blocks != 800 ? 0xff : 0;
    put_be32(msg + 14, FD_DRIVE_SUPERDRIVE | (i ? 0x100 : 0));
    put_be16(msg + 22, mac_iop_fd_format(blocks));
    put_be16(msg + 24, mac_iop_fd_media_formats(blocks));
}

/* Mac memory for a buffer the host gave; in 24-bit mode only A0-A23 count */
static hwaddr mac_iop_host_addr(uint32_t addr, uint32_t len)
{
    uint64_t ram = current_machine->ram_size;

    if (addr + (uint64_t)len > ram && (addr & 0xffffff) + len <= ram) {
        addr &= 0xffffff;
    }
    return addr;
}

static int mac_iop_fd_rw(MacIOPState *s, int i, uint8_t *msg, int kind)
{
    uint32_t buf = get_be32(msg + 4);
    uint32_t block = get_be32(msg + 8);
    uint32_t count = get_be32(msg + 12);
    uint32_t blocks = mac_iop_fd_blocks(s, i);
    hwaddr host = mac_iop_host_addr(buf, count * 512);
    uint8_t data[512], mem[512];
    uint32_t n;

    if (!blocks) {
        return OS_OFFLINERR;
    }
    if (block > blocks || count > blocks - block) {
        return OS_PARAMERR;
    }
    if (kind == FD_WRITE && !blk_is_writable(s->fd_blk[i])) {
        return OS_WPRERR;
    }
    for (n = 0; n < count; n++, host += 512) {
        int64_t off = (int64_t)(block + n) * 512;

        if (kind == FD_WRITE) {
            address_space_read(&address_space_memory, host,
                               MEMTXATTRS_UNSPECIFIED, data, 512);
            if (blk_pwrite(s->fd_blk[i], off, 512, data, 0) < 0) {
                return OS_IOERR;
            }
            continue;
        }
        if (blk_pread(s->fd_blk[i], off, 512, data, 0) < 0) {
            return OS_IOERR;
        }
        if (kind == FD_READ) {
            address_space_write(&address_space_memory, host,
                                MEMTXATTRS_UNSPECIFIED, data, 512);
        } else {
            address_space_read(&address_space_memory, host,
                               MEMTXATTRS_UNSPECIFIED, mem, 512);
            if (memcmp(data, mem, 512)) {
                return OS_DATAVERERR;
            }
        }
    }
    /* image files have no tags */
    memset(msg + 16, 0, 12);
    return OS_NOERR;
}

/*
 * "Formatting" an image: the contents go. @format is the Sony driver's
 * format number, a place in the list of formats the disk takes (1 =
 * the first bit of mac_iop_fd_media_formats()). A real double-density
 * disk has two sides however it was formatted before, so an 800K format
 * of a 400K image makes it an 800K one.
 */
static int mac_iop_fd_format_disk(MacIOPState *s, int i, int format)
{
    uint32_t blocks = mac_iop_fd_blocks(s, i);
    uint16_t formats = mac_iop_fd_media_formats(blocks);
    uint8_t zero[512] = { 0 };
    uint32_t n;
    int bit;

    if (!blocks) {
        return OS_OFFLINERR;
    }
    if (!blk_is_writable(s->fd_blk[i])) {
        return OS_WPRERR;
    }
    for (bit = 0; bit < 16 && format > 0; bit++) {
        if ((formats & BIT(bit)) && --format == 0) {
            break;
        }
    }
    if (bit == 2 && blocks == 800) {
        BlockBackend *blk = s->fd_blk[i];
        uint64_t perm, shared;
        Error *err = NULL;
        int ret;

        /* the resize permission only for as long as it takes */
        blk_get_perm(blk, &perm, &shared);
        ret = blk_set_perm(blk, perm | BLK_PERM_RESIZE, shared, &err);
        if (ret == 0) {
            ret = blk_truncate(blk, 1600 * 512, false, PREALLOC_MODE_OFF, 0,
                               &err);
            blk_set_perm(blk, perm, shared, &error_abort);
        }
        if (ret < 0) {
            error_reportf_err(err, "floppy: cannot make the 400K image 800K: ");
            return OS_IOERR;
        }
        blocks = 1600;
    }
    for (n = 0; n < blocks; n++) {
        if (blk_pwrite(s->fd_blk[i], (int64_t)n * 512, 512, zero, 0) < 0) {
            return OS_IOERR;
        }
    }
    return OS_NOERR;
}

static void mac_iop_fd_report_insert(MacIOPState *s, int i)
{
    uint8_t msg[MAC_IOP_MSG_LEN] = { 0 };

    if (!s->fd_polling || !mac_iop_fd_has_disk(s, i)) {
        return;
    }
    mac_iop_fd_status(s, i, msg);
    msg[0] = FD_EVT_INSERTED;
    msg[1] = i + 1;
    mac_iop_queue_recv(s, IOP_FD_CHAN, msg);
}

/*
 * The Sony driver answers an insert message with the result of posting
 * the disk-inserted event. Early in startup the Event Manager does not
 * take disk events yet (evtNotEnb), and the driver then forgets the disk:
 * like the IOP firmware, tell it again a little later. The ROM boots from
 * drive 1 without the event, which is why only the second drive needs it.
 */
#define FD_RETRY_NS     (500 * SCALE_MS)

static void mac_iop_fd_insert_done(MacIOPState *s, const uint8_t *msg)
{
    int i = msg[1] - 1;

    if (msg[0] != FD_EVT_INSERTED || i < 0 || i >= MAC_IOP_NUM_FD) {
        return;
    }
    s->fd_retry[i] = msg[2] || msg[3];
    if (s->fd_retry[i]) {
        timer_mod(s->fd_retry_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + FD_RETRY_NS);
    }
}

static void mac_iop_fd_retry(void *opaque)
{
    MacIOPState *s = opaque;

    for (int i = 0; i < MAC_IOP_NUM_FD; i++) {
        if (s->fd_retry[i]) {
            s->fd_retry[i] = false;
            mac_iop_fd_report_insert(s, i);
        }
    }
}

static void mac_iop_fd_send(MacIOPState *s, uint8_t *msg)
{
    int kind = msg[0];
    int i = mac_iop_fd_index(s, msg[1]);
    int err = OS_NOERR;

    switch (kind) {
    case FD_INIT:
        /* drive kinds of drives 0-4: an internal SuperDrive, maybe two */
        memset(msg + 4, 0, 5);
        for (i = 0; i < MAC_IOP_NUM_FD; i++) {
            if (i == 0 || s->fd_blk[i]) {
                msg[5 + i] = FD_DRIVE_SUPERDRIVE;
            }
        }
        break;
    case FD_START_POLL:
        s->fd_polling = true;
        for (i = 0; i < MAC_IOP_NUM_FD; i++) {
            mac_iop_fd_report_insert(s, i);
        }
        break;
    case FD_STOP_POLL:
    case FD_SHUTDOWN:
        s->fd_polling = false;
        break;
    case FD_STATUS:
    case FD_EJECT:
        if (i < 0) {
            err = OS_NSDRVERR;
            break;
        }
        if (kind == FD_EJECT) {
            if (!mac_iop_fd_has_disk(s, i)) {
                err = OS_OFFLINERR;
            }
            s->fd_in[i] = false;
        }
        mac_iop_fd_status(s, i, msg);
        break;
    case FD_FORMAT:
    case FD_FORMAT_VERIFY:
    case FD_WRITE:
    case FD_READ:
    case FD_READ_VERIFY:
        if (i < 0) {
            err = OS_NSDRVERR;
        } else if (kind == FD_FORMAT) {
            err = mac_iop_fd_format_disk(s, i, msg[4] << 8 | msg[5]);
        } else if (kind == FD_FORMAT_VERIFY) {
            err = mac_iop_fd_blocks(s, i) ? OS_NOERR : OS_OFFLINERR;
        } else {
            err = mac_iop_fd_rw(s, i, msg, kind);
        }
        break;
    default:
        /* tag addresses, caches, raw data: nothing to do here */
        break;
    }
    trace_mac_iop_floppy(kind, msg[1], err);
    put_be16(msg + 2, err);
}

/* write permission only for disks that allow it, so read-only ones fit */
static int mac_iop_fd_set_perm(BlockBackend *blk, Error **errp)
{
    return blk_set_perm(blk, BLK_PERM_CONSISTENT_READ |
                        (blk_supports_write_perm(blk) ? BLK_PERM_WRITE : 0),
                        BLK_PERM_ALL, errp);
}

static void mac_iop_fd_change(void *opaque, bool load, Error **errp)
{
    MacIOPFloppyRef *ref = opaque;
    MacIOPState *s = ref->iop;
    int i = ref->index;
    BlockBackend *blk = s->fd_blk[i];

    if (!load) {
        blk_set_perm(blk, 0, BLK_PERM_ALL, &error_abort);
        s->fd_in[i] = false;
        return;
    }
    if (mac_iop_fd_set_perm(blk, errp) < 0) {
        return;
    }
    s->fd_in[i] = blk_is_inserted(blk);
    mac_iop_fd_report_insert(s, i);
}

static const BlockDevOps mac_iop_fd_ops = {
    .change_media_cb = mac_iop_fd_change,
};
static void mac_iop_handle_send(MacIOPState *s, int chan, uint8_t *msg)
{
    if (s->kind == MAC_IOP_ISM && chan == IOP_ADB_CHAN) {
        mac_iop_adb_send(s, msg);
        return;
    }
    if (s->kind == MAC_IOP_ISM && chan == IOP_FD_CHAN) {
        mac_iop_fd_send(s, msg);
        return;
    }
    qemu_log_mask(LOG_UNIMP, "mac-iop %d: send chan %d %02x %02x %02x %02x "
                  "%02x %02x %02x %02x\n", s->kind, chan, msg[0], msg[1],
                  msg[2], msg[3], msg[4], msg[5], msg[6], msg[7]);
}

/* The host poked the IOP: look at the channels */
static void mac_iop_poll(MacIOPState *s)
{
    int chan;

    if (!mac_iop_running(s)) {
        return;
    }

    for (chan = 0; chan < MAC_IOP_NUM_CHAN; chan++) {
        uint8_t *state = &s->ram[IOP_ADDR_RECV_STATE + chan];

        if (*state == IOP_MSG_COMPLETE) {
            uint8_t *msg = &s->ram[IOP_ADDR_RECV_MSG + chan * MAC_IOP_MSG_LEN];

            trace_mac_iop_recv_done(s->kind, chan);
            *state = IOP_MSG_IDLE;
            if (s->kind == MAC_IOP_ISM && chan == IOP_FD_CHAN) {
                mac_iop_fd_insert_done(s, msg);
            }
            /*
             * The Mac OS ADB driver replies to each ADB message with its
             * next command (or the autopoll setup), rather than sending it
             */
            if (s->kind == MAC_IOP_ISM && chan == IOP_ADB_CHAN) {
                trace_mac_iop_send(s->kind, chan, msg[0], msg[1], msg[2],
                                   msg[3]);
                mac_iop_adb_send(s, msg);
            }
        }
    }

    for (chan = 0; chan < MAC_IOP_NUM_CHAN; chan++) {
        uint8_t *state = &s->ram[IOP_ADDR_SEND_STATE + chan];

        if (*state == IOP_MSG_NEW) {
            uint8_t *msg = &s->ram[IOP_ADDR_SEND_MSG + chan * MAC_IOP_MSG_LEN];

            trace_mac_iop_send(s->kind, chan, msg[0], msg[1], msg[2], msg[3]);
            mac_iop_handle_send(s, chan, msg);
            *state = IOP_MSG_COMPLETE;
            mac_iop_raise(s, IOP_INT0);
        }
    }

    mac_iop_deliver(s);
}

static uint8_t mac_iop_ram_read(MacIOPState *s, uint16_t addr)
{
    addr &= MAC_IOP_RAM_SIZE - 1;
    if (addr == IOP_ADDR_ALIVE && mac_iop_running(s)) {
        /* the IOP keeps setting it while it runs */
        s->ram[addr] = 0xff;
    }
    return s->ram[addr];
}

static uint64_t mac_iop_read(void *opaque, hwaddr addr, unsigned size)
{
    MacIOPState *s = opaque;
    uint64_t val = 0;
    unsigned i;

    switch (addr & ~1) {
    case IOP_REG_ADDR_HI:
        val = s->ram_addr >> 8;
        break;
    case IOP_REG_ADDR_LO:
        val = s->ram_addr & 0xff;
        break;
    case IOP_REG_STATUS:
        val = s->status | IOP_DMAINACTIVE;
        break;
    case IOP_REG_DATA:
        for (i = 0; i < size; i++) {
            val = (val << 8) | mac_iop_ram_read(s, s->ram_addr + i);
        }
        if (s->status & IOP_AUTOINC) {
            s->ram_addr = (s->ram_addr + size) & (MAC_IOP_RAM_SIZE - 1);
        }
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "mac-iop: read of unknown register 0x%"
                      HWADDR_PRIx "\n", addr);
    }

    trace_mac_iop_read(s->kind, addr, val, size);
    return val;
}

static void mac_iop_write(void *opaque, hwaddr addr, uint64_t val,
                          unsigned size)
{
    MacIOPState *s = opaque;
    uint8_t old;
    unsigned i;

    trace_mac_iop_write(s->kind, addr, val, size);

    switch (addr & ~1) {
    case IOP_REG_ADDR_HI:
        s->ram_addr = ((val & 0xff) << 8) | (s->ram_addr & 0xff);
        break;
    case IOP_REG_ADDR_LO:
        s->ram_addr = (s->ram_addr & 0xff00) | (val & 0xff);
        break;
    case IOP_REG_STATUS:
        old = s->status;
        /* INT0 and INT1 are acknowledged by writing 1 */
        s->status &= ~(val & (IOP_INT0 | IOP_INT1));
        s->status = (s->status & ~(IOP_BYPASS | IOP_AUTOINC | IOP_RUN)) |
                    (val & (IOP_BYPASS | IOP_AUTOINC | IOP_RUN));
        if ((old ^ s->status) & IOP_RUN) {
            trace_mac_iop_run(s->kind, !!(s->status & IOP_RUN));
            if (s->status & IOP_RUN) {
                /* what the IOP kernel sets up when it starts */
                s->ram[IOP_ADDR_MAX_SEND_CHAN] = MAC_IOP_NUM_CHAN - 1;
                s->ram[IOP_ADDR_MAX_RECV_CHAN] = MAC_IOP_NUM_CHAN - 1;
                s->ram[IOP_ADDR_ALIVE] = 0xff;
            } else {
                s->status &= ~(IOP_INT0 | IOP_INT1);
                memset(s->recv_queue, 0, sizeof(s->recv_queue));
            }
        }
        if (val & IOP_IRQ) {
            mac_iop_poll(s);
        }
        mac_iop_update_irq(s);
        break;
    case IOP_REG_DATA:
        for (i = 0; i < size; i++) {
            s->ram[(s->ram_addr + i) & (MAC_IOP_RAM_SIZE - 1)] =
                val >> (8 * (size - 1 - i));
        }
        if (s->status & IOP_AUTOINC) {
            s->ram_addr = (s->ram_addr + size) & (MAC_IOP_RAM_SIZE - 1);
        }
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "mac-iop: write of unknown register 0x%"
                      HWADDR_PRIx "\n", addr);
    }
}

static const MemoryRegionOps mac_iop_ops = {
    .read = mac_iop_read,
    .write = mac_iop_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static void mac_iop_timer(void *opaque)
{
    MacIOPState *s = opaque;

    mac_iop_poll(s);
}

static void mac_iop_reset_hold(Object *obj, ResetType type)
{
    MacIOPState *s = MAC_IOP(obj);

    memset(s->ram, 0, sizeof(s->ram));
    s->ram_addr = 0;
    s->status = 0;
    s->adb_autopoll = false;
    memset(s->recv_queue, 0, sizeof(s->recv_queue));
    adb_set_autopoll_enabled(&s->adb_bus, false);
    s->fd_polling = false;
    for (int i = 0; i < MAC_IOP_NUM_FD; i++) {
        s->fd_in[i] = s->fd_blk[i] && blk_is_inserted(s->fd_blk[i]);
    }
    timer_del(s->timer);
    timer_del(s->fd_retry_timer);
    memset(s->fd_retry, 0, sizeof(s->fd_retry));
    mac_iop_update_irq(s);
}

static void mac_iop_realize(DeviceState *dev, Error **errp)
{
    MacIOPState *s = MAC_IOP(dev);

    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, mac_iop_timer, s);
    s->fd_retry_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, mac_iop_fd_retry, s);
    adb_register_autopoll_callback(&s->adb_bus, mac_iop_adb_poll, s);

    for (int i = 0; i < MAC_IOP_NUM_FD; i++) {
        BlockBackend *blk = s->fd_blk[i];

        if (!blk) {
            continue;
        }
        if (mac_iop_fd_set_perm(blk, errp) < 0) {
            return;
        }
        s->fd_ref[i].iop = s;
        s->fd_ref[i].index = i;
        blk_set_dev_ops(blk, &mac_iop_fd_ops, &s->fd_ref[i]);
    }
}

static void mac_iop_init(Object *obj)
{
    MacIOPState *s = MAC_IOP(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->mem, obj, &mac_iop_ops, s, "mac-iop",
                          MAC_IOP_REG_SIZE);
    sysbus_init_mmio(sbd, &s->mem);
    sysbus_init_irq(sbd, &s->irq);

    qbus_init((BusState *)&s->adb_bus, sizeof(s->adb_bus), TYPE_ADB_BUS,
              DEVICE(obj), "adb.0");
}

static const VMStateDescription vmstate_mac_iop = {
    .name = "mac-iop",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8_ARRAY(ram, MacIOPState, MAC_IOP_RAM_SIZE),
        VMSTATE_UINT16(ram_addr, MacIOPState),
        VMSTATE_UINT8(status, MacIOPState),
        VMSTATE_TIMER_PTR(timer, MacIOPState),
        VMSTATE_BOOL(adb_autopoll, MacIOPState),
        VMSTATE_BOOL_ARRAY(fd_in, MacIOPState, MAC_IOP_NUM_FD),
        VMSTATE_BOOL(fd_polling, MacIOPState),
        VMSTATE_TIMER_PTR(fd_retry_timer, MacIOPState),
        VMSTATE_BOOL_ARRAY(fd_retry, MacIOPState, MAC_IOP_NUM_FD),
        VMSTATE_END_OF_LIST()
    }
};

static const Property mac_iop_properties[] = {
    DEFINE_PROP_UINT8("kind", MacIOPState, kind, MAC_IOP_SCC),
    DEFINE_PROP_DRIVE("drive0", MacIOPState, fd_blk[0]),
    DEFINE_PROP_DRIVE("drive1", MacIOPState, fd_blk[1]),
};

static void mac_iop_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->realize = mac_iop_realize;
    dc->vmsd = &vmstate_mac_iop;
    rc->phases.hold = mac_iop_reset_hold;
    device_class_set_props(dc, mac_iop_properties);
}

static const TypeInfo mac_iop_types[] = {
    {
        .name          = TYPE_MAC_IOP,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(MacIOPState),
        .instance_init = mac_iop_init,
        .class_init    = mac_iop_class_init,
    },
};

DEFINE_TYPES(mac_iop_types)

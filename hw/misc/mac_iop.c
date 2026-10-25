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
 * directly ("high-level emulation").
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

static void mac_iop_handle_send(MacIOPState *s, int chan, uint8_t *msg)
{
    if (s->kind == MAC_IOP_ISM && chan == IOP_ADB_CHAN) {
        mac_iop_adb_send(s, msg);
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
    timer_del(s->timer);
    mac_iop_update_irq(s);
}

static void mac_iop_realize(DeviceState *dev, Error **errp)
{
    MacIOPState *s = MAC_IOP(dev);

    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, mac_iop_timer, s);
    adb_register_autopoll_callback(&s->adb_bus, mac_iop_adb_poll, s);
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
        VMSTATE_END_OF_LIST()
    }
};

static const Property mac_iop_properties[] = {
    DEFINE_PROP_UINT8("kind", MacIOPState, kind, MAC_IOP_SCC),
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

/*
 * AMD Am79C940 MACE Ethernet controller, as wired up in the Quadra AV
 * Macs
 *
 * 32 byte registers, one every 16 bytes. The chip's FIFOs are emptied and
 * filled by the PSC's DMA engine, so frames go straight between the PSC
 * and the network here; the registers give the setup, the addresses and
 * the interrupt status.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "net/eth.h"
#include "hw/net/mac_mace.h"
#include "trace.h"

enum {
    R_RCVFIFO = 0, R_XMTFIFO, R_XMTFC, R_XMTFS, R_XMTRC, R_RCVFC, R_RCVFS,
    R_FIFOFC, R_IR, R_IMR, R_PR, R_BIUCC, R_FIFOCC, R_MACCC, R_PLSCC,
    R_PHYCC, R_CHIPID0, R_CHIPID1, R_IAC, R_LADRF = 20, R_PADR, R_MPC = 24,
    R_RNTPC = 26, R_RCVCC, R_UTR = 29,
};

/* IR */
#define IR_RCVINT   0x02
#define IR_XMTINT   0x01
#define IR_MPCO     0x04
/* MACCC */
#define MACCC_ENRCV 0x01
#define MACCC_ENXMT 0x02
#define MACCC_PROM  0x80
/* BIUCC */
#define BIUCC_SWRST 0x01
/* IAC */
#define IAC_ADDRCHG 0x80
#define IAC_PHYADDR 0x04
#define IAC_LOGADDR 0x02
/* XMTFS */
#define XMTFS_XMTSV 0x80
/* RCVFC */
#define RCVFC_ASTRP 0x01
/* UTR */
#define UTR_LOOP    0x06
/* PR */
#define PR_XMTSV    0x80

#define MACE_CHIPID 0x0940

static void mace_update_irq(MacMACEState *s)
{
    qemu_set_irq(s->irq, !!(s->regs[R_IR] & ~s->regs[R_IMR] & 0xff));
}

static void mace_soft_reset(MacMACEState *s)
{
    memset(s->regs, 0, sizeof(s->regs));
    s->regs[R_XMTFC] = 0x01;        /* APAD */
    s->regs[R_RCVFC] = RCVFC_ASTRP;
    s->regs[R_BIUCC] = 0x00;
    s->regs[R_FIFOCC] = 0x0f;
    s->regs[R_PLSCC] = 0x00;
    s->regs[R_PHYCC] = 0x00;
    s->regs[R_IMR] = 0x00;
    s->addr_index = 0;
    mace_update_irq(s);
}

static bool mace_deliver(MacMACEState *s, const uint8_t *buf, size_t size);

bool mac_mace_can_transmit(MacMACEState *s)
{
    return s->regs[R_MACCC] & MACCC_ENXMT;
}

void mac_mace_transmit(MacMACEState *s, const uint8_t *buf, size_t len)
{
    uint8_t frame[2048];

    trace_mac_mace_transmit(len);
    if (len > sizeof(frame)) {
        len = sizeof(frame);
    }
    memcpy(frame, buf, len);
    /* APAD: pad short frames */
    if (len < 60) {
        memset(frame + len, 0, 60 - len);
        len = 60;
    }
    if (s->regs[R_UTR] & UTR_LOOP) {
        /* loopback (the driver's self test): straight back to the receiver */
        mace_deliver(s, frame, len);
    } else {
        qemu_send_packet(qemu_get_queue(s->nic), frame, len);
    }
    s->regs[R_XMTFS] = XMTFS_XMTSV;
    s->regs[R_XMTRC] = 0;
    s->regs[R_PR] |= PR_XMTSV;
    s->regs[R_IR] |= IR_XMTINT;
    mace_update_irq(s);
}

void mac_mace_rx_ready(MacMACEState *s)
{
    qemu_flush_queued_packets(qemu_get_queue(s->nic));
}

static bool mace_wanted(MacMACEState *s, const uint8_t *buf)
{
    unsigned idx;

    if (s->regs[R_MACCC] & MACCC_PROM) {
        return true;
    }
    if (!memcmp(buf, s->padr, 6)) {
        return true;
    }
    if (is_broadcast_ether_addr(buf)) {
        return true;
    }
    if (buf[0] & 1) {
        idx = net_crc32_le(buf, ETH_ALEN) >> 26;
        return s->ladrf[idx >> 3] & (1 << (idx & 7));
    }
    return false;
}

static bool mace_can_receive(NetClientState *nc)
{
    MacMACEState *s = qemu_get_nic_opaque(nc);

    return (s->regs[R_MACCC] & MACCC_ENRCV) && s->rx;
}

static ssize_t mace_receive(NetClientState *nc, const uint8_t *buf,
                            size_t size)
{
    MacMACEState *s = qemu_get_nic_opaque(nc);

    if (s->regs[R_UTR] & UTR_LOOP) {
        return size;            /* the wire is cut off in loopback */
    }
    return mace_deliver(s, buf, size) ? size : 0;
}

/* a frame for the receiver; false: no room for it yet */
static bool mace_deliver(MacMACEState *s, const uint8_t *buf, size_t size)
{
    uint8_t frame[2048];
    uint8_t status[4];
    size_t len = size;

    if (!(s->regs[R_MACCC] & MACCC_ENRCV) || size < 14) {
        return true;
    }
    if (!mace_wanted(s, buf)) {
        return true;
    }
    if (len > sizeof(frame) - 4) {
        len = sizeof(frame) - 4;
    }
    memcpy(frame, buf, len);
    if (len < 60) {
        memset(frame + len, 0, 60 - len);
        len = 60;
    }
    /* without ASTRP_RCV the chip passes the FCS on to the host */
    if (!(s->regs[R_RCVFC] & RCVFC_ASTRP)) {
        stl_le_p(frame + len, net_crc32_le(frame, len) ^ 0xffffffff);
        len += 4;
    }

    status[0] = len & 0xff;
    status[1] = (len >> 8) & 0x0f;
    status[2] = s->regs[R_RNTPC];
    status[3] = s->regs[R_RCVCC];
    trace_mac_mace_receive(len);
    if (!s->rx(s->rx_opaque, frame, len, status)) {
        /* no buffer: the DMA engine will tell us when there is one */
        return false;
    }
    s->regs[R_IR] |= IR_RCVINT;
    mace_update_irq(s);
    return true;
}

static uint64_t mace_read(void *opaque, hwaddr addr, unsigned size)
{
    MacMACEState *s = opaque;
    unsigned reg = (addr >> 4) & 31;
    uint8_t val;

    switch (reg) {
    case R_IR:
        val = s->regs[R_IR];
        s->regs[R_IR] = 0;
        mace_update_irq(s);
        break;
    case R_XMTFS:
        val = s->regs[R_XMTFS];
        s->regs[R_XMTFS] = 0;
        s->regs[R_PR] &= ~PR_XMTSV;
        break;
    case R_CHIPID0:
        val = MACE_CHIPID & 0xff;
        break;
    case R_CHIPID1:
        val = MACE_CHIPID >> 8;
        break;
    case R_MPC:
    case R_RNTPC:
    case R_RCVCC:
        val = s->regs[reg];
        s->regs[reg] = 0;
        break;
    case R_LADRF:
        val = s->ladrf[s->addr_index++ & 7];
        break;
    case R_PADR:
        val = s->padr[s->addr_index++ % 6];
        break;
    case R_RCVFIFO:
    case R_RCVFS:
        val = 0;
        break;
    default:
        val = s->regs[reg];
        break;
    }
    trace_mac_mace_read(reg, val);
    return val;
}

static void mace_write(void *opaque, hwaddr addr, uint64_t val,
                       unsigned size)
{
    MacMACEState *s = opaque;
    unsigned reg = (addr >> 4) & 31;

    trace_mac_mace_write(reg, val);
    switch (reg) {
    case R_BIUCC:
        if (val & BIUCC_SWRST) {
            mace_soft_reset(s);
            return;
        }
        s->regs[reg] = val;
        break;
    case R_IAC:
        /* the address change takes at once: ADDRCHG reads back clear */
        s->regs[reg] = val & ~IAC_ADDRCHG;
        if (val & (IAC_PHYADDR | IAC_LOGADDR)) {
            s->addr_index = 0;
        }
        break;
    case R_LADRF:
        s->ladrf[s->addr_index++ & 7] = val;
        break;
    case R_PADR:
        s->padr[s->addr_index++ % 6] = val;
        break;
    case R_IMR:
        s->regs[reg] = val;
        mace_update_irq(s);
        break;
    case R_MACCC:
        s->regs[reg] = val;
        if (val & MACCC_ENRCV) {
            qemu_flush_queued_packets(qemu_get_queue(s->nic));
        }
        break;
    case R_XMTFIFO:
    case R_IR:
    case R_CHIPID0:
    case R_CHIPID1:
        break;
    default:
        s->regs[reg] = val;
        break;
    }
}

static const MemoryRegionOps mace_ops = {
    .read = mace_read,
    .write = mace_write,
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

static NetClientInfo mace_net_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = mace_can_receive,
    .receive = mace_receive,
};

static void mace_reset(DeviceState *dev)
{
    MacMACEState *s = MAC_MACE(dev);

    mace_soft_reset(s);
    memcpy(s->padr, s->conf.macaddr.a, 6);
    memset(s->ladrf, 0, sizeof(s->ladrf));
}

static void mace_realize(DeviceState *dev, Error **errp)
{
    MacMACEState *s = MAC_MACE(dev);

    qemu_macaddr_default_if_unset(&s->conf.macaddr);
    s->nic = qemu_new_nic(&mace_net_info, &s->conf,
                          object_get_typename(OBJECT(dev)), dev->id,
                          &dev->mem_reentrancy_guard, s);
    qemu_format_nic_info_str(qemu_get_queue(s->nic), s->conf.macaddr.a);
}

static void mace_init(Object *obj)
{
    MacMACEState *s = MAC_MACE(obj);

    memory_region_init_io(&s->mmio, obj, &mace_ops, s, "mac-mace", 0x200);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static const VMStateDescription vmstate_mace = {
    .name = "mac-mace",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8_ARRAY(regs, MacMACEState, 32),
        VMSTATE_UINT8_ARRAY(padr, MacMACEState, 6),
        VMSTATE_UINT8_ARRAY(ladrf, MacMACEState, 8),
        VMSTATE_UINT8(addr_index, MacMACEState),
        VMSTATE_END_OF_LIST()
    },
};

static const Property mace_properties[] = {
    DEFINE_NIC_PROPERTIES(MacMACEState, conf),
};

static void mace_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = mace_realize;
    device_class_set_legacy_reset(dc, mace_reset);
    dc->vmsd = &vmstate_mace;
    device_class_set_props(dc, mace_properties);
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
}

static const TypeInfo mace_types[] = {
    {
        .name = TYPE_MAC_MACE,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(MacMACEState),
        .instance_init = mace_init,
        .class_init = mace_class_init,
    },
};

DEFINE_TYPES(mace_types)

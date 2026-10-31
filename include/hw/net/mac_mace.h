/*
 * AMD Am79C940 MACE Ethernet controller, as wired up in the Quadra AV
 * Macs: the PSC's DMA engine moves the frames
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_NET_MAC_MACE_H
#define HW_NET_MAC_MACE_H

#include "hw/core/sysbus.h"
#include "net/net.h"
#include "qom/object.h"

#define TYPE_MAC_MACE "mac-mace"
OBJECT_DECLARE_SIMPLE_TYPE(MacMACEState, MAC_MACE)

/*
 * A received frame for the DMA engine: the frame and the four receive
 * status bytes (count low, status with count bits 8-11, runt and
 * collision counts). Returns false when there is no room for it.
 */
typedef bool (*MacMACERxFunc)(void *opaque, const uint8_t *buf, size_t len,
                              const uint8_t status[4]);

struct MacMACEState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    NICState *nic;
    NICConf conf;
    qemu_irq irq;

    MacMACERxFunc rx;
    void *rx_opaque;

    uint8_t regs[32];
    uint8_t padr[6];
    uint8_t ladrf[8];
    uint8_t addr_index;
};

/* hand a frame from the DMA engine to the wire */
void mac_mace_transmit(MacMACEState *s, const uint8_t *buf, size_t len);

/* whether the transmitter takes frames (ENXMT) */
bool mac_mace_can_transmit(MacMACEState *s);

/* the DMA engine has room for frames again */
void mac_mace_rx_ready(MacMACEState *s);

#endif

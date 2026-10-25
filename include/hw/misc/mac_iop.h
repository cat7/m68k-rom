/*
 * Apple I/O Processor (IOP) as used in the Macintosh IIfx, Quadra 900
 * and Quadra 950: high-level model of the host interface and the
 * message protocol the Mac OS IOP Manager uses.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_MAC_IOP_H
#define HW_MISC_MAC_IOP_H

#include "hw/core/sysbus.h"
#include "hw/input/adb.h"
#include "qom/object.h"

#define MAC_IOP_RAM_SIZE    0x8000
#define MAC_IOP_REG_SIZE    0x20

#define MAC_IOP_NUM_CHAN    8
#define MAC_IOP_MSG_LEN     32

typedef enum MacIOPKind {
    MAC_IOP_SCC = 0,        /* serial: the SCC behind it */
    MAC_IOP_ISM = 1,        /* SWIM floppy controller and ADB */
} MacIOPKind;

#define MAC_IOP_QUEUE_LEN   8

typedef struct MacIOPQueue {
    uint8_t msg[MAC_IOP_QUEUE_LEN][MAC_IOP_MSG_LEN];
    uint8_t head;
    uint8_t count;
} MacIOPQueue;

#define TYPE_MAC_IOP "mac-iop"
OBJECT_DECLARE_SIMPLE_TYPE(MacIOPState, MAC_IOP)

struct MacIOPState {
    SysBusDevice parent_obj;

    MemoryRegion mem;
    qemu_irq irq;

    uint8_t kind;
    uint8_t ram[MAC_IOP_RAM_SIZE];
    uint16_t ram_addr;
    uint8_t status;

    QEMUTimer *timer;           /* the "firmware" heartbeat */

    /* ADB (ISM IOP) */
    ADBBusState adb_bus;
    bool adb_autopoll;

    /* IOP-to-host messages not yet in the receive channels */
    MacIOPQueue recv_queue[MAC_IOP_NUM_CHAN];
};

#endif

/*
 * Apple OSS (Operating System Support) chip of the Macintosh IIfx
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_MAC_OSS_H
#define HW_MISC_MAC_OSS_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define MAC_OSS_SIZE        0x2000
#define MAC_OSS_NUM_SRC     16

/* interrupt sources (bit in the pending register = level register index) */
enum {
    OSS_SRC_SLOT9 = 0,          /* NuBus slots $9-$E: 0-5 */
    OSS_SRC_SWIM_IOP = 6,
    OSS_SRC_SCC_IOP = 7,
    OSS_SRC_ASC = 8,
    OSS_SRC_SCSI = 9,
    OSS_SRC_60HZ = 10,
    OSS_SRC_VIA1 = 11,
    OSS_SRC_RTC = 12,
    OSS_SRC_ADB = 13,
    OSS_SRC_PARITY = 14,
};

#define TYPE_MAC_OSS "mac-oss"
OBJECT_DECLARE_SIMPLE_TYPE(MacOSSState, MAC_OSS)

struct MacOSSState {
    SysBusDevice parent_obj;

    MemoryRegion mem;
    qemu_irq ipl;               /* the 68k interrupt level, 0-7 */
    qemu_irq overlay;           /* high: ROM at address 0 */

    uint8_t level[MAC_OSS_NUM_SRC];
    uint16_t inputs;            /* source lines (except 60 Hz) */
    bool tick;                  /* 60 Hz pending, until acknowledged */
    uint8_t rom_ctrl;
    uint8_t counter_ctrl;
    uint64_t counter_hold;

    QEMUTimer *timer;
    int64_t next_tick;
};

#endif

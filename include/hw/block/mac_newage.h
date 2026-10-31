/*
 * Apple "New Age" floppy disk controller (Quadra 660AV/840AV)
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_BLOCK_MAC_NEWAGE_H
#define HW_BLOCK_MAC_NEWAGE_H

#include "hw/core/sysbus.h"
#include "hw/block/block.h"
#include "qom/object.h"

#define TYPE_MAC_NEWAGE         "mac-newage"
#define TYPE_MAC_NEWAGE_BUS     "mac-newage-bus"
#define TYPE_MAC_NEWAGE_DRIVE   "mac-newage-drive"

OBJECT_DECLARE_SIMPLE_TYPE(MacNewAgeState, MAC_NEWAGE)
OBJECT_DECLARE_SIMPLE_TYPE(MacNewAgeDrive, MAC_NEWAGE_DRIVE)

#define MAC_NEWAGE_MAX_FD   2

/*
 * The DMA channel: move up to @len bytes between @buf and memory
 * (@to_mem: disk to memory). Returns the bytes moved.
 */
typedef uint32_t (*MacNewAgeDMAFunc)(void *opaque, uint8_t *buf,
                                     uint32_t len, bool to_mem);

/* the disk formats */
enum {
    NEWAGE_FMT_NONE,
    NEWAGE_FMT_GCR400,
    NEWAGE_FMT_GCR800,
    NEWAGE_FMT_MFM720,
    NEWAGE_FMT_MFM1440,
};

struct MacNewAgeDrive {
    DeviceState parent_obj;

    BlockConf conf;
    int32_t unit;
    MacNewAgeState *ctrl;

    int format;                 /* NEWAGE_FMT_*, of the disk inside */
    bool disk_in;
    bool write_protected;
    uint8_t cyl;                /* where the head is */
    uint8_t next_sector;        /* the next sector to pass under the head */
};

struct MacNewAgeState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    BusState *bus;
    qemu_irq irq;

    MacNewAgeDMAFunc dma;
    void *dma_opaque;

    MacNewAgeDrive *drives[MAC_NEWAGE_MAX_FD];

    /* command and result phases */
    uint8_t cmd[16];
    uint8_t cmd_len;
    uint8_t cmd_pos;
    uint8_t res[8];
    uint8_t res_len;
    uint8_t res_pos;
    bool res_irq;               /* a command's result phase interrupts */

    /* interrupt status for SENSE INTERRUPT: ST0 and PCN */
    uint8_t sense_st0[8];
    uint8_t sense_pcn[8];
    uint8_t sense_n;

    uint8_t dsr;

    /* disk changes, reported once the driver has gone quiet */
    uint8_t change[MAC_NEWAGE_MAX_FD];
    QEMUTimer *idle_timer;
    QEMUTimer *eject_timer;
    uint8_t eject_unit;

    uint8_t buf[18 * 1024 + 512];
};

#endif

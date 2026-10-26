/*
 * NCR 5380 / 53C80 SCSI bus controller
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_SCSI_NCR5380_H
#define HW_SCSI_NCR5380_H

#include "hw/core/sysbus.h"
#include "hw/scsi/scsi.h"
#include "qom/object.h"

#define TYPE_NCR5380 "ncr5380"
OBJECT_DECLARE_SIMPLE_TYPE(NCR5380State, NCR5380)

/* Unnamed GPIO outputs, both active high */
#define NCR5380_GPIO_IRQ    0
#define NCR5380_GPIO_DRQ    1

/* sysbus MMIO regions */
#define NCR5380_MMIO_REGS   0   /* 8 registers, register n at n << it_shift */
#define NCR5380_MMIO_PDMA   1   /* pseudo-DMA window, pdma-size bytes */

#define NCR5380_NUM_REGS    8
#define NCR5380_CDB_MAX     16

struct NCR5380State {
    SysBusDevice parent_obj;

    MemoryRegion regs;
    MemoryRegion pdma;
    qemu_irq irq;
    qemu_irq drq;
    SCSIBus bus;

    /* properties */
    uint8_t it_shift;
    uint32_t pdma_size;
    uint8_t initiator_id;
    bool sync_io;
    bool defer_req;

    /* chip registers and internal state */
    uint8_t odr;
    uint8_t idr;
    uint8_t icr;
    uint8_t mr;
    uint8_t tcr;
    uint8_t selen;
    bool aip;
    bool irq_state;
    bool drq_state;
    bool parity_err;
    bool busy_err;
    bool end_dma;
    int dma_dir;
    bool dma_latched;

    /* emulated SCSI bus / target side */
    int tstate;
    bool req;
    bool req_hidden;
    bool acked;
    uint8_t target;
    uint8_t lun;
    bool msg_abort;
    uint8_t cdb[NCR5380_CDB_MAX];
    int cdb_len;
    int cdb_pos;
    uint8_t status;
    uint32_t tag;
    SCSIDevice *current_dev;
    SCSIRequest *current_req;
    uint8_t *buf;
    uint32_t buf_len;
    uint32_t buf_pos;
    bool waiting;
};

/*
 * Register access for machines that need their own address decoding
 * (e.g. the Mac II's A9 = /DACK alias inside the register window).
 * @reg is 0..7, the value is the 8-bit register contents.
 */
uint8_t ncr5380_reg_read(NCR5380State *s, unsigned reg);
void ncr5380_reg_write(NCR5380State *s, unsigned reg, uint8_t val);

/*
 * Pseudo-DMA (/DACK) data access: one byte per call, with an automatic
 * REQ/ACK handshake.  ncr5380_drq() returns the current DRQ state.
 */
uint8_t ncr5380_dma_read(NCR5380State *s);
void ncr5380_dma_write(NCR5380State *s, uint8_t val);
bool ncr5380_drq(NCR5380State *s);

#endif

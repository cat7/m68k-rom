/*
 * NCR 5380 / 53C80 SCSI bus controller
 *
 * Only initiator mode is implemented.  The controller is attached to a
 * QEMU SCSIBus, so the SCSI bus seen by the guest is synthesised here:
 * whenever the guest selects a target that exists on the QEMU bus, this
 * model plays the target's part of the bus protocol (BSY, phase lines,
 * REQ) and turns the bytes exchanged through the REQ/ACK handshake into
 * QEMU SCSI requests.
 *
 * Data can be moved either with the manual REQ/ACK handshake (ICR assert
 * ACK) or in "pseudo-DMA" mode, where every access to the /DACK data path
 * (ncr5380_dma_read/ncr5380_dma_write or the pdma MMIO region) performs
 * one complete handshake.
 *
 * Asynchronous I/O: QEMU's scsi-disk completes READ/WRITE asynchronously,
 * but guests such as the Macintosh SCSI Manager transfer blocks "blind",
 * i.e. they only check DRQ before the first byte and rely on the hardware
 * to insert wait states afterwards.  With the "sync-io" property (default
 * on) the model therefore waits, inside the MMIO access that started an
 * I/O, until QEMU has delivered the next data chunk or completed the
 * request (blk_drain() on the target's block backend, the same approach
 * hw/ide/macio.c uses).  From the guest's point of view the target then
 * answers immediately.  Without sync-io REQ/DRQ simply stay low until the
 * data arrives, and a DMA access without DRQ returns the input data
 * register unchanged (writes are latched until the target asks for them).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/bitops.h"
#include "qapi/error.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/scsi/ncr5380.h"
#include "migration/vmstate.h"
#include "scsi/constants.h"
#include "scsi/utils.h"
#include "system/block-backend.h"
#include "trace.h"

/* Register numbers */
#define REG_CDR         0   /* r: current SCSI data, w: output data */
#define REG_ICR         1   /* initiator command */
#define REG_MR          2   /* mode */
#define REG_TCR         3   /* target command */
#define REG_CSR         4   /* r: current bus status, w: select enable */
#define REG_BSR         5   /* r: bus and status, w: start DMA send */
#define REG_IDR         6   /* r: input data, w: start DMA target receive */
#define REG_RPI         7   /* r: reset parity/IRQ, w: start DMA ini. recv */

/* Initiator command register */
#define ICR_DBUS        0x01
#define ICR_ATN         0x02
#define ICR_SEL         0x04
#define ICR_BSY         0x08
#define ICR_ACK         0x10
#define ICR_LA          0x20    /* read: lost arbitration */
#define ICR_AIP         0x40    /* read: arbitration in progress */
#define ICR_RST         0x80
#define ICR_WMASK       (ICR_RST | ICR_ACK | ICR_BSY | ICR_SEL | ICR_ATN | \
                         ICR_DBUS)

/* Mode register */
#define MR_ARB          0x01
#define MR_DMA          0x02
#define MR_MONBSY       0x04
#define MR_EOPIRQ       0x08
#define MR_PARIRQ       0x10
#define MR_PARCHK       0x20
#define MR_TARGET       0x40
#define MR_BLOCK        0x80

/* Target command register */
#define TCR_IO          0x01
#define TCR_CD          0x02
#define TCR_MSG         0x04
#define TCR_REQ         0x08
#define TCR_PHASE       (TCR_MSG | TCR_CD | TCR_IO)
#define TCR_WMASK       0x0f

/* Current SCSI bus status register */
#define CSR_DBP         0x01
#define CSR_SEL         0x02
#define CSR_IO          0x04
#define CSR_CD          0x08
#define CSR_MSG         0x10
#define CSR_REQ         0x20
#define CSR_BSY         0x40
#define CSR_RST         0x80

/* Bus and status register */
#define BSR_ACK         0x01
#define BSR_ATN         0x02
#define BSR_BUSYERR     0x04
#define BSR_PHASEMATCH  0x08
#define BSR_IRQ         0x10
#define BSR_PARERR      0x20
#define BSR_DRQ         0x40
#define BSR_ENDDMA      0x80

/* SCSI bus phases, encoded as MSG/CD/IO like the TCR */
#define PHASE_DATA_OUT  0
#define PHASE_DATA_IN   TCR_IO
#define PHASE_COMMAND   TCR_CD
#define PHASE_STATUS    (TCR_CD | TCR_IO)
#define PHASE_MSG_OUT   (TCR_MSG | TCR_CD)
#define PHASE_MSG_IN    (TCR_MSG | TCR_CD | TCR_IO)

/* Pseudo-DMA direction armed by the start DMA registers */
enum {
    DMA_NONE,
    DMA_SEND,
    DMA_RECV,
};

/* State of the emulated target */
enum {
    TGT_FREE,           /* bus free, no target connected */
    TGT_SELECTED,       /* target asserts BSY, waiting for SEL to drop */
    TGT_MSG_OUT,
    TGT_COMMAND,
    TGT_DATA_OUT,
    TGT_DATA_IN,
    TGT_STATUS,
    TGT_MSG_IN,
};

static const char *const tstate_names[] = {
    [TGT_FREE] = "BUS FREE",
    [TGT_SELECTED] = "SELECTED",
    [TGT_MSG_OUT] = "MESSAGE OUT",
    [TGT_COMMAND] = "COMMAND",
    [TGT_DATA_OUT] = "DATA OUT",
    [TGT_DATA_IN] = "DATA IN",
    [TGT_STATUS] = "STATUS",
    [TGT_MSG_IN] = "MESSAGE IN",
};

static const uint8_t tstate_phase[] = {
    [TGT_FREE] = PHASE_DATA_OUT,
    [TGT_SELECTED] = PHASE_DATA_OUT,
    [TGT_MSG_OUT] = PHASE_MSG_OUT,
    [TGT_COMMAND] = PHASE_COMMAND,
    [TGT_DATA_OUT] = PHASE_DATA_OUT,
    [TGT_DATA_IN] = PHASE_DATA_IN,
    [TGT_STATUS] = PHASE_STATUS,
    [TGT_MSG_IN] = PHASE_MSG_IN,
};

static const char *const rreg_names[NCR5380_NUM_REGS] = {
    "CDR", "ICR", "MR", "TCR", "CSR", "BSR", "IDR", "RPI",
};

static const char *const wreg_names[NCR5380_NUM_REGS] = {
    "ODR", "ICR", "MR", "TCR", "SER", "SDS", "SDTR", "SDIR",
};

#define SYNC_IO_MAX_ROUNDS  16

static void ncr5380_target_req(NCR5380State *s);
static void ncr5380_target_set_state(NCR5380State *s, int tstate);

/* SCSI phase currently driven on the bus by the target */
static uint8_t ncr5380_bus_phase(NCR5380State *s)
{
    return tstate_phase[s->tstate];
}

static bool ncr5380_phase_match(NCR5380State *s)
{
    return ncr5380_bus_phase(s) == (s->tcr & TCR_PHASE);
}

static bool ncr5380_bus_busy(NCR5380State *s)
{
    return s->tstate != TGT_FREE || (s->icr & ICR_BSY) ||
           ((s->mr & MR_ARB) && s->aip);
}

/* Byte the target presents on the data bus in an input phase */
static uint8_t ncr5380_target_byte(NCR5380State *s)
{
    switch (s->tstate) {
    case TGT_DATA_IN:
        return s->buf_pos < s->buf_len ? s->buf[s->buf_pos] : 0;
    case TGT_STATUS:
        return s->status;
    case TGT_MSG_IN:
        return 0x00;    /* COMMAND COMPLETE */
    default:
        return 0;
    }
}

/* Value of the SCSI data bus (wired-OR of all drivers) */
static uint8_t ncr5380_bus_data(NCR5380State *s)
{
    uint8_t phase = ncr5380_bus_phase(s);
    uint8_t v = 0;

    if ((s->mr & MR_ARB) && s->aip) {
        v |= s->odr;
    }
    if (s->icr & ICR_DBUS) {
        /*
         * The initiator's drivers are only enabled while the bus is free
         * or in a matching output phase.
         */
        if (s->tstate <= TGT_SELECTED ||
            (!(phase & TCR_IO) && ncr5380_phase_match(s))) {
            v |= s->odr;
        }
    }
    if (s->tstate > TGT_SELECTED && (phase & TCR_IO)) {
        v |= ncr5380_target_byte(s);
    }
    return v;
}

static void ncr5380_set_irq(NCR5380State *s, bool level)
{
    if (level != s->irq_state) {
        trace_ncr5380_irq(level);
        s->irq_state = level;
        qemu_set_irq(s->irq, level);
    }
}

static void ncr5380_update_drq(NCR5380State *s)
{
    bool drq = false;

    if (s->mr & MR_DMA) {
        switch (s->dma_dir) {
        case DMA_SEND:
            /* The chip asks for a byte whenever its output latch is free */
            drq = !s->dma_latched;
            break;
        case DMA_RECV:
            drq = s->req && (ncr5380_bus_phase(s) & TCR_IO) &&
                  ncr5380_phase_match(s);
            break;
        }
    }

    if (drq != s->drq_state) {
        trace_ncr5380_drq(drq);
        s->drq_state = drq;
        qemu_set_irq(s->drq, drq);
    }
}

/*
 * Wait for QEMU to deliver the data or status the target is waiting for,
 * see the comment at the top of the file.
 */
static void ncr5380_sync_io(NCR5380State *s)
{
    BlockBackend *blk;
    int i;

    if (!s->sync_io || !s->waiting || !s->current_req ||
        !s->current_dev) {
        return;
    }
    blk = s->current_dev->conf.blk;
    if (!blk) {
        return;
    }
    /*
     * A request that is (re)issued while the backend is drained is queued
     * until the drained section ends, so allow a few rounds.  If the
     * request still has not progressed (e.g. it is parked because of
     * werror=stop) give up and let it complete asynchronously.
     */
    for (i = 0; i < SYNC_IO_MAX_ROUNDS && s->waiting && s->current_req; i++) {
        blk_drain(blk);
    }
    trace_ncr5380_sync_io(i, s->waiting);
}

/* Drop the current request without completing it */
static void ncr5380_cancel_request(NCR5380State *s)
{
    SCSIRequest *req = s->current_req;

    s->waiting = false;
    s->buf = NULL;
    s->buf_len = 0;
    s->buf_pos = 0;
    if (req) {
        s->current_req = NULL;
        scsi_req_cancel(req);
        scsi_req_unref(req);
    }
}

/* Target releases the bus */
static void ncr5380_target_free(NCR5380State *s)
{
    s->tstate = TGT_FREE;
    s->req = false;
    s->req_hidden = false;
    s->acked = false;
    s->current_dev = NULL;
    ncr5380_cancel_request(s);
}

/* Normal or unexpected disconnect: the 5380 sees BSY go away */
static void ncr5380_target_disconnect(NCR5380State *s)
{
    trace_ncr5380_disconnect(s->target);
    ncr5380_target_free(s);

    /*
     * Like the chip, stop driving the bus and leave DMA mode on loss of
     * BSY; with "monitor busy" set this is also an error interrupt.
     */
    s->icr &= ICR_RST;
    if (s->mr & MR_DMA) {
        s->mr &= ~MR_DMA;
        s->dma_dir = DMA_NONE;
        s->dma_latched = false;
        s->end_dma = false;
    }
    if (s->mr & MR_MONBSY) {
        s->busy_err = true;
        ncr5380_set_irq(s, true);
    }
    ncr5380_update_drq(s);
}

/* Pending pseudo-DMA output byte: hand it to the target if it wants one */
static void ncr5380_dma_try_send(NCR5380State *s);

static void ncr5380_target_req(NCR5380State *s)
{
    s->req = true;
    /*
     * The Mac II ROM can get stuck when REQ for the next phase is already
     * set when it looks for the end of the previous handshake, so only
     * show a new REQ in the CSR from the second CSR read on.
     */
    s->req_hidden = s->defer_req;

    if ((s->mr & MR_DMA) && !ncr5380_phase_match(s)) {
        /* Phase mismatch interrupt, ends a DMA transfer */
        trace_ncr5380_phase_mismatch(ncr5380_bus_phase(s),
                                     s->tcr & TCR_PHASE);
        if (s->dma_dir != DMA_NONE) {
            s->end_dma = true;
        }
        ncr5380_set_irq(s, true);
    }

    ncr5380_dma_try_send(s);
    ncr5380_update_drq(s);
}

static void ncr5380_target_set_state(NCR5380State *s, int tstate)
{
    trace_ncr5380_phase(tstate_names[s->tstate], tstate_names[tstate]);
    s->tstate = tstate;
    s->req = false;
    s->req_hidden = false;

    switch (tstate) {
    case TGT_COMMAND:
        s->cdb_pos = 0;
        s->cdb_len = 0;
        ncr5380_target_req(s);
        break;
    case TGT_MSG_OUT:
    case TGT_STATUS:
    case TGT_MSG_IN:
        ncr5380_target_req(s);
        break;
    case TGT_DATA_IN:
    case TGT_DATA_OUT:
        if (s->buf_pos < s->buf_len) {
            ncr5380_target_req(s);
        }
        break;
    default:
        break;
    }
    ncr5380_update_drq(s);
}

/* The CDB is complete: hand it to the QEMU SCSI device */
static void ncr5380_command(NCR5380State *s)
{
    SCSIDevice *dev;
    SCSIRequest *req;
    int32_t len;

    trace_ncr5380_command(s->target, s->lun, s->cdb[0], s->cdb_len);

    dev = scsi_device_find(&s->bus, 0, s->target, s->lun);
    if (!dev) {
        /* The target disappeared (hot unplug) */
        s->status = CHECK_CONDITION;
        ncr5380_target_set_state(s, TGT_STATUS);
        return;
    }

    if (s->current_req) {
        ncr5380_cancel_request(s);
    }

    /* The target stays in COMMAND phase, without REQ, until it is ready */
    req = scsi_req_new(dev, s->tag++, s->lun, s->cdb, s->cdb_len, s);
    s->current_req = req;
    s->buf = NULL;
    s->buf_len = 0;
    s->buf_pos = 0;
    s->waiting = true;

    len = scsi_req_enqueue(req);
    /* A request without data may already have completed here */
    if (len != 0 && s->current_req == req) {
        scsi_req_continue(req);
    }
}

static void ncr5380_message_out(NCR5380State *s, uint8_t msg)
{
    trace_ncr5380_msg_out(msg);

    if (msg & 0x80) {
        /* IDENTIFY; disconnect privilege is ignored, we never disconnect */
        s->lun = msg & 0x07;
    } else if (msg == 0x06 || msg == 0x0c) {
        /* ABORT, BUS DEVICE RESET: the target goes bus free */
        s->msg_abort = true;
    } else {
        /* Other messages (including extended ones) are ignored */
        qemu_log_mask(LOG_UNIMP, "ncr5380: ignoring message out 0x%02x\n",
                      msg);
    }
}

/* Initiator asserts ACK while the target asserts REQ: byte transferred */
static void ncr5380_target_ack(NCR5380State *s, uint8_t out)
{
    int len;

    s->req = false;
    s->req_hidden = false;
    s->acked = true;

    if (ncr5380_bus_phase(s) & TCR_IO) {
        /* The input data register latches the bus on ACK */
        s->idr = ncr5380_target_byte(s);
    }

    switch (s->tstate) {
    case TGT_MSG_OUT:
        ncr5380_message_out(s, out);
        break;
    case TGT_COMMAND:
        if (s->cdb_pos == 0) {
            s->cdb[0] = out;
            len = scsi_cdb_length(s->cdb);
            if (len <= 0 || len > NCR5380_CDB_MAX) {
                /* Reserved/vendor group: assume a 6 byte CDB */
                len = 6;
            }
            s->cdb_len = len;
        }
        if (s->cdb_pos < s->cdb_len) {
            s->cdb[s->cdb_pos++] = out;
        }
        break;
    case TGT_DATA_IN:
        if (s->buf_pos < s->buf_len) {
            s->buf_pos++;
        }
        break;
    case TGT_DATA_OUT:
        if (s->buf_pos < s->buf_len) {
            s->buf[s->buf_pos++] = out;
        }
        break;
    default:
        break;
    }
    ncr5380_update_drq(s);
}

/* Initiator releases ACK: the target moves on */
static void ncr5380_target_ack_release(NCR5380State *s)
{
    s->acked = false;

    switch (s->tstate) {
    case TGT_MSG_OUT:
        if (s->msg_abort) {
            s->msg_abort = false;
            ncr5380_target_disconnect(s);
        } else if (s->icr & ICR_ATN) {
            ncr5380_target_req(s);
        } else {
            ncr5380_target_set_state(s, TGT_COMMAND);
        }
        break;
    case TGT_COMMAND:
        if (s->cdb_pos < s->cdb_len) {
            ncr5380_target_req(s);
        } else {
            ncr5380_command(s);
        }
        break;
    case TGT_DATA_IN:
    case TGT_DATA_OUT:
        if (s->buf_pos < s->buf_len) {
            ncr5380_target_req(s);
        } else if (s->current_req) {
            /* Chunk done; ask QEMU for the next one or for completion */
            s->buf_len = 0;
            s->buf_pos = 0;
            s->waiting = true;
            scsi_req_continue(s->current_req);
        } else {
            ncr5380_target_set_state(s, TGT_STATUS);
        }
        break;
    case TGT_STATUS:
        ncr5380_target_set_state(s, TGT_MSG_IN);
        break;
    case TGT_MSG_IN:
        ncr5380_target_disconnect(s);
        break;
    default:
        break;
    }
}

/* One complete REQ/ACK cycle on behalf of the pseudo-DMA logic */
static void ncr5380_handshake(NCR5380State *s, uint8_t out)
{
    ncr5380_target_ack(s, out);
    ncr5380_target_ack_release(s);
}

static void ncr5380_dma_try_send(NCR5380State *s)
{
    uint8_t phase = ncr5380_bus_phase(s);

    if (s->dma_latched && (s->mr & MR_DMA) && s->req &&
        s->tstate > TGT_SELECTED && !(phase & TCR_IO) &&
        ncr5380_phase_match(s)) {
        s->dma_latched = false;
        ncr5380_handshake(s, s->odr);
    }
}

/* Selection of a QEMU SCSI device and the start of the info phases */
static void ncr5380_check_selection(NCR5380State *s)
{
    if (s->tstate == TGT_FREE && (s->icr & ICR_SEL) &&
        !(s->icr & ICR_RST) && !(s->mr & MR_ARB)) {
        /*
         * A target responds as soon as SEL is asserted with its ID on the
         * data bus.  Drivers differ in the order in which they set up
         * ODR, ICR and MR, so this is re-evaluated after each of those
         * writes, and the ID is taken from ODR whether or not the
         * initiator already drives the bus.
         */
        uint8_t ids = s->odr & ~BIT(s->initiator_id);
        SCSIDevice *dev;

        if (ids && is_power_of_2(ids)) {
            int id = ctz32(ids);

            dev = scsi_device_find(&s->bus, 0, id, 0);
            if (dev && dev->id == id) {
                trace_ncr5380_select(id, !!(s->icr & ICR_ATN));
                s->target = id;
                s->lun = 0;
                s->msg_abort = false;
                s->current_dev = dev;
                s->tstate = TGT_SELECTED;
            }
        }
    }

    if (s->tstate == TGT_SELECTED && !(s->icr & ICR_SEL)) {
        /* Selection done; with ATN the initiator has a message for us */
        ncr5380_target_set_state(s, (s->icr & ICR_ATN) ? TGT_MSG_OUT :
                                                         TGT_COMMAND);
    }
}

static void ncr5380_update(NCR5380State *s)
{
    /* Arbitration: we are the only initiator, so we win once bus is free */
    if ((s->mr & MR_ARB) && !s->aip && s->tstate == TGT_FREE &&
        !(s->icr & (ICR_SEL | ICR_RST))) {
        s->aip = true;
    }
    ncr5380_check_selection(s);
    ncr5380_update_drq(s);
}

static void ncr5380_chip_reset(NCR5380State *s)
{
    s->odr = 0;
    s->idr = 0;
    s->icr = 0;
    s->mr = 0;
    s->tcr = 0;
    s->selen = 0;
    s->aip = false;
    s->parity_err = false;
    s->busy_err = false;
    s->end_dma = false;
    s->dma_dir = DMA_NONE;
    s->dma_latched = false;
}

static void ncr5380_scsi_bus_reset(NCR5380State *s)
{
    trace_ncr5380_bus_reset();
    ncr5380_target_free(s);
    ncr5380_chip_reset(s);
    bus_cold_reset(BUS(&s->bus));
    ncr5380_set_irq(s, true);
    ncr5380_update_drq(s);
}

static uint8_t ncr5380_read_csr(NCR5380State *s)
{
    uint8_t v = 0;

    if (s->icr & ICR_RST) {
        v |= CSR_RST;
    }
    if (ncr5380_bus_busy(s)) {
        v |= CSR_BSY;
    }
    if (s->req) {
        if (s->req_hidden) {
            s->req_hidden = false;
        } else {
            v |= CSR_REQ;
        }
    }
    if (s->tstate > TGT_SELECTED) {
        v |= ncr5380_bus_phase(s) << 2;
    }
    if (s->icr & ICR_SEL) {
        v |= CSR_SEL;
    }
    return v;
}

static uint8_t ncr5380_read_bsr(NCR5380State *s)
{
    uint8_t v = 0;

    if (s->end_dma) {
        v |= BSR_ENDDMA;
    }
    if (s->drq_state) {
        v |= BSR_DRQ;
    }
    if (s->parity_err) {
        v |= BSR_PARERR;
    }
    if (s->irq_state) {
        v |= BSR_IRQ;
    }
    if (ncr5380_phase_match(s)) {
        v |= BSR_PHASEMATCH;
    }
    if (s->busy_err) {
        v |= BSR_BUSYERR;
    }
    if (s->icr & ICR_ATN) {
        v |= BSR_ATN;
    }
    if (s->icr & ICR_ACK) {
        v |= BSR_ACK;
    }
    return v;
}

uint8_t ncr5380_reg_read(NCR5380State *s, unsigned reg)
{
    uint8_t v = 0;

    reg &= NCR5380_NUM_REGS - 1;
    switch (reg) {
    case REG_CDR:
        v = ncr5380_bus_data(s);
        break;
    case REG_ICR:
        v = s->icr;
        if (s->aip) {
            v |= ICR_AIP;
        }
        /* There is no other initiator, so arbitration is never lost */
        break;
    case REG_MR:
        v = s->mr;
        break;
    case REG_TCR:
        v = s->tcr;
        break;
    case REG_CSR:
        v = ncr5380_read_csr(s);
        break;
    case REG_BSR:
        v = ncr5380_read_bsr(s);
        break;
    case REG_IDR:
        v = s->idr;
        break;
    case REG_RPI:
        s->parity_err = false;
        s->busy_err = false;
        ncr5380_set_irq(s, false);
        break;
    }
    trace_ncr5380_reg_read(rreg_names[reg], v);
    return v;
}

static void ncr5380_write_icr(NCR5380State *s, uint8_t val)
{
    uint8_t old = s->icr;

    if ((val & ICR_RST) && !(old & ICR_RST)) {
        ncr5380_scsi_bus_reset(s);
        s->icr = val & ICR_WMASK;
        return;
    }

    s->icr = val & ICR_WMASK;

    if ((val & ICR_ACK) && !(old & ICR_ACK)) {
        if (s->req && s->tstate > TGT_SELECTED) {
            ncr5380_target_ack(s, s->odr);
        }
    } else if (!(val & ICR_ACK) && (old & ICR_ACK)) {
        if (s->acked) {
            ncr5380_target_ack_release(s);
        }
    }
}

static void ncr5380_write_mr(NCR5380State *s, uint8_t val)
{
    uint8_t old = s->mr;

    if (val & MR_TARGET) {
        qemu_log_mask(LOG_UNIMP, "ncr5380: target mode not supported\n");
    }

    s->mr = val;

    if (!(val & MR_MONBSY)) {
        s->busy_err = false;
    }
    if ((old & MR_DMA) && !(val & MR_DMA)) {
        s->dma_dir = DMA_NONE;
        s->dma_latched = false;
        s->end_dma = false;
    }
    if ((old ^ val) & MR_ARB) {
        /* Started or stopped arbitration; ncr5380_update() re-evaluates */
        s->aip = false;
    }
}

static void ncr5380_start_dma(NCR5380State *s, int dir)
{
    if (!(s->mr & MR_DMA)) {
        return;
    }
    s->dma_dir = dir;
    s->dma_latched = false;
    s->end_dma = false;
}

void ncr5380_reg_write(NCR5380State *s, unsigned reg, uint8_t val)
{
    reg &= NCR5380_NUM_REGS - 1;
    trace_ncr5380_reg_write(wreg_names[reg], val);

    switch (reg) {
    case REG_CDR:
        s->odr = val;
        break;
    case REG_ICR:
        ncr5380_write_icr(s, val);
        break;
    case REG_MR:
        ncr5380_write_mr(s, val);
        break;
    case REG_TCR:
        s->tcr = val & TCR_WMASK;
        break;
    case REG_CSR:
        /* Select enable; we never get (re)selected, so just store it */
        s->selen = val;
        break;
    case REG_BSR:
        ncr5380_start_dma(s, DMA_SEND);
        break;
    case REG_IDR:
        qemu_log_mask(LOG_UNIMP,
                      "ncr5380: start DMA target receive not supported\n");
        break;
    case REG_RPI:
        ncr5380_start_dma(s, DMA_RECV);
        break;
    }

    ncr5380_update(s);
    ncr5380_sync_io(s);
}

uint8_t ncr5380_dma_read(NCR5380State *s)
{
    bool ok = false;
    uint8_t v;

    ncr5380_sync_io(s);

    /*
     * A/UX reads the DMA window with DMA mode enabled but without arming
     * a receive, so only DMA mode and the phase are checked here.
     */
    if ((s->mr & MR_DMA) && s->req && s->tstate > TGT_SELECTED &&
        (ncr5380_bus_phase(s) & TCR_IO) && ncr5380_phase_match(s)) {
        ncr5380_handshake(s, 0);
        ok = true;
    }
    /* Without a handshake this returns the previously latched byte */
    v = s->idr;
    trace_ncr5380_dma_read(v, ok);

    ncr5380_sync_io(s);
    return v;
}

void ncr5380_dma_write(NCR5380State *s, uint8_t val)
{
    trace_ncr5380_dma_write(val, s->dma_latched);

    if (!(s->mr & MR_DMA)) {
        return;
    }
    /* A byte still in the output latch is overwritten (lost) */
    s->odr = val;
    s->dma_latched = true;
    ncr5380_dma_try_send(s);
    ncr5380_update_drq(s);
    ncr5380_sync_io(s);
}

bool ncr5380_drq(NCR5380State *s)
{
    return s->drq_state;
}

/* SCSIBusInfo callbacks */

static void ncr5380_transfer_data(SCSIRequest *req, uint32_t len)
{
    NCR5380State *s = req->hba_private;

    trace_ncr5380_transfer_data(len);
    if (req != s->current_req) {
        return;
    }

    s->buf = scsi_req_get_buf(req);
    s->buf_len = len;
    s->buf_pos = 0;
    s->waiting = false;

    if (len == 0) {
        s->waiting = true;
        scsi_req_continue(req);
        return;
    }

    ncr5380_target_set_state(s, req->cmd.mode == SCSI_XFER_TO_DEV ?
                                TGT_DATA_OUT : TGT_DATA_IN);
}

static void ncr5380_command_complete(SCSIRequest *req, size_t resid)
{
    NCR5380State *s = req->hba_private;

    trace_ncr5380_command_complete(req->status, resid);
    if (req != s->current_req) {
        return;
    }

    s->status = req->status < 0 ? CHECK_CONDITION : req->status;
    s->current_req = NULL;
    s->waiting = false;
    s->buf = NULL;
    s->buf_len = 0;
    s->buf_pos = 0;
    scsi_req_unref(req);

    if (s->tstate != TGT_FREE) {
        ncr5380_target_set_state(s, TGT_STATUS);
    }
}

static void ncr5380_request_cancelled(SCSIRequest *req)
{
    NCR5380State *s = req->hba_private;

    if (req == s->current_req) {
        s->current_req = NULL;
        s->waiting = false;
        s->buf = NULL;
        s->buf_len = 0;
        s->buf_pos = 0;
        scsi_req_unref(req);
    }
}

static const struct SCSIBusInfo ncr5380_scsi_info = {
    .tcq = false,
    .max_target = 6,   /* the initiator is ID 7 */
    .max_lun = 7,

    .transfer_data = ncr5380_transfer_data,
    .complete = ncr5380_command_complete,
    .cancel = ncr5380_request_cancelled,
};

/* MMIO */

static uint64_t ncr5380_regs_read(void *opaque, hwaddr addr, unsigned size)
{
    NCR5380State *s = opaque;
    uint8_t v = ncr5380_reg_read(s, addr >> s->it_shift);

    /* Present the register on every byte lane */
    return v * (MAKE_64BIT_MASK(0, size * 8) / 0xff);
}

static void ncr5380_regs_write(void *opaque, hwaddr addr, uint64_t val,
                               unsigned size)
{
    NCR5380State *s = opaque;

    /* Wider accesses: take the most significant (even address) byte */
    ncr5380_reg_write(s, addr >> s->it_shift, val >> ((size - 1) * 8));
}

static const MemoryRegionOps ncr5380_regs_ops = {
    .read = ncr5380_regs_read,
    .write = ncr5380_regs_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

static uint64_t ncr5380_pdma_read(void *opaque, hwaddr addr, unsigned size)
{
    NCR5380State *s = opaque;
    uint64_t v = 0;
    unsigned i;

    for (i = 0; i < size; i++) {
        v = (v << 8) | ncr5380_dma_read(s);
    }
    return v;
}

static void ncr5380_pdma_write(void *opaque, hwaddr addr, uint64_t val,
                               unsigned size)
{
    NCR5380State *s = opaque;
    int i;

    for (i = size - 1; i >= 0; i--) {
        ncr5380_dma_write(s, val >> (i * 8));
    }
}

static const MemoryRegionOps ncr5380_pdma_ops = {
    .read = ncr5380_pdma_read,
    .write = ncr5380_pdma_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

static void ncr5380_reset(DeviceState *dev)
{
    NCR5380State *s = NCR5380(dev);

    ncr5380_target_free(s);
    ncr5380_chip_reset(s);
    s->msg_abort = false;
    s->status = 0;
    ncr5380_set_irq(s, false);
    ncr5380_update_drq(s);
}

static void ncr5380_realize(DeviceState *dev, Error **errp)
{
    NCR5380State *s = NCR5380(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    if (s->initiator_id > 7) {
        error_setg(errp, "initiator-id must be between 0 and 7");
        return;
    }
    if (s->it_shift > 16) {
        error_setg(errp, "it_shift too large");
        return;
    }
    if (!s->pdma_size) {
        error_setg(errp, "pdma-size must not be 0");
        return;
    }

    memory_region_init_io(&s->regs, OBJECT(s), &ncr5380_regs_ops, s,
                          "ncr5380-regs", NCR5380_NUM_REGS << s->it_shift);
    sysbus_init_mmio(sbd, &s->regs);
    memory_region_init_io(&s->pdma, OBJECT(s), &ncr5380_pdma_ops, s,
                          "ncr5380-pdma", s->pdma_size);
    sysbus_init_mmio(sbd, &s->pdma);

    qdev_init_gpio_out(dev, &s->irq, 1);    /* NCR5380_GPIO_IRQ */
    qdev_init_gpio_out(dev, &s->drq, 1);    /* NCR5380_GPIO_DRQ */

    scsi_bus_init(&s->bus, sizeof(s->bus), dev, &ncr5380_scsi_info);
}

static const Property ncr5380_properties[] = {
    DEFINE_PROP_UINT8("it_shift", NCR5380State, it_shift, 0),
    DEFINE_PROP_UINT32("pdma-size", NCR5380State, pdma_size, 0x2000),
    DEFINE_PROP_UINT8("initiator-id", NCR5380State, initiator_id, 7),
    DEFINE_PROP_BOOL("sync-io", NCR5380State, sync_io, true),
    DEFINE_PROP_BOOL("defer-req", NCR5380State, defer_req, true),
};

/* The bus state and in-flight requests are not migrated */
static const VMStateDescription vmstate_ncr5380 = {
    .name = "ncr5380",
    .unmigratable = 1,
};

static void ncr5380_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->desc = "NCR 5380 SCSI controller";
    dc->realize = ncr5380_realize;
    device_class_set_legacy_reset(dc, ncr5380_reset);
    device_class_set_props(dc, ncr5380_properties);
    dc->vmsd = &vmstate_ncr5380;
    set_bit(DEVICE_CATEGORY_STORAGE, dc->categories);
}

static const TypeInfo ncr5380_info = {
    .name = TYPE_NCR5380,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NCR5380State),
    .class_init = ncr5380_class_init,
};

static void ncr5380_register_types(void)
{
    type_register_static(&ncr5380_info);
}

type_init(ncr5380_register_types)

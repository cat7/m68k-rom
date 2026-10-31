/*
 * Apple PSC ("Peripheral Subsystem Controller") of the Quadra 660AV and
 * 840AV: the pseudo-VIA2, the interrupt levels 3-6, the DMA engine and the
 * sound frame engine. VIA1 is a separate device.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_MAC_PSC_H
#define HW_MISC_MAC_PSC_H

#include "hw/core/sysbus.h"
#include "qom/object.h"
#include "qemu/audio.h"
#include "hw/net/mac_mace.h"

#define TYPE_MAC_PSC "mac-psc"
OBJECT_DECLARE_SIMPLE_TYPE(MacPSCState, MAC_PSC)

/* sysbus MMIO regions */
enum {
    MAC_PSC_MMIO_VIA2,          /* $50F02000, 8 KB */
    MAC_PSC_MMIO_REGS,          /* $50F31000, 8 KB */
};

/* unnamed GPIO inputs */
enum {
    MAC_PSC_IN_VIA1,            /* VIA1's interrupt: level 1 */
    MAC_PSC_IN_SCSI_IRQ,        /* pseudo-VIA2 IFR bit 3 */
    MAC_PSC_IN_SCSI_DRQ,        /* IFR bit 0, and the SCSI DMA channel */
    MAC_PSC_IN_FDC,             /* IFR bit 5 */
    MAC_PSC_IN_ENET,            /* level 3 bit 0 */
    MAC_PSC_IN_SCC_A,           /* level 4 bit 1 */
    MAC_PSC_IN_SCC_B,           /* level 4 bit 2 */
    MAC_PSC_IN_NB,
};

/*
 * Named GPIO input "slot", lines 0-6: the slot interrupts (active high
 * here), shown active low in the pseudo-VIA2's slot register. Lines 3-5
 * are NuBus slots $C-$E, line 6 is the built-in video's VBL.
 */
#define MAC_PSC_SLOT_LINES 7

#define MAC_PSC_DMA_CHANNELS 9

typedef struct MacPSCDMASet {
    uint32_t addr;
    uint32_t count;
    uint16_t cmd;
} MacPSCDMASet;

struct MacPSCState {
    SysBusDevice parent_obj;

    MemoryRegion via2_mmio;
    MemoryRegion regs_mmio;
    MemoryRegion *scsi_pdma;    /* the 53C96's pseudo-DMA data window */
    MacMACEState *mace;         /* the Ethernet controller */
    AddressSpace *dma_as;
    qemu_irq ipl;               /* the CPU's interrupt level, 0-6 */
    QEMUTimer *frame_timer;
    QEMUBH *scsi_bh;
    AudioBackend *audio_be;
    SWVoiceOut *voice;

    /* pseudo-VIA2 */
    uint8_t ifr;
    uint8_t ier;
    uint8_t slot_lines;         /* asserted slot interrupts */

    /* levels 3-6: level inputs, latched events and enables */
    uint8_t lvl_src[4];
    uint8_t lvl_latch[4];
    uint8_t lvl_ier[4];

    bool via1_irq;
    bool scsi_drq;
    bool pumping;               /* inside the SCSI DMA loop */
    int cur_ipl;

    /* DMA */
    uint16_t dma_ctrl[MAC_PSC_DMA_CHANNELS];
    MacPSCDMASet dma_set[MAC_PSC_DMA_CHANNELS][2];

    /* sound */
    uint16_t snd_com;
    uint32_t singer_ctl;
    uint32_t snd_in_base;
    uint32_t snd_out_base;
    uint16_t snd_size;
    uint8_t dsp_overrun;
    int64_t snd_epoch_ns;       /* when the sample counter was 0 */
    int64_t frame_next;         /* index of the next frame boundary */
    int16_t *out_ring;          /* stereo samples for the audio backend */
    uint32_t out_head, out_tail;
};

/* MACE receive callback: store a frame through DMA channel 1 */
bool mac_psc_enet_receive(void *opaque, const uint8_t *buf, size_t len,
                          const uint8_t status[4]);

#endif

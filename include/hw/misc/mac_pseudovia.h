/*
 * Apple "pseudo-VIA": the interrupt controller and GPIO block in the
 * RBV, V8 and Sonora chips of the Mac IIci, LC family and LC III
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_MAC_PSEUDOVIA_H
#define HW_MISC_MAC_PSEUDOVIA_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_MAC_PSEUDOVIA "mac-pseudovia"
OBJECT_DECLARE_SIMPLE_TYPE(MacPseudoVIAState, MAC_PSEUDOVIA)

/* the chip it is part of */
enum {
    MAC_PSEUDOVIA_RBV,
    MAC_PSEUDOVIA_V8,
    MAC_PSEUDOVIA_SONORA,
};

/* unnamed GPIO inputs, active high */
enum {
    MAC_PSEUDOVIA_SCSI_DRQ,     /* IFR bit 0 (CA2) */
    MAC_PSEUDOVIA_SCSI_IRQ,     /* IFR bit 3 (CB2) */
    MAC_PSEUDOVIA_ASC,          /* IFR bit 4 (CB1) */
    MAC_PSEUDOVIA_IN_NB,
};

/*
 * Named GPIO input "slot", lines 0-6: the slot interrupts, shown active
 * low in register 2 and summed into IFR bit 1. On Sonora line 6 is the
 * built-in video's VBL and lines 3-5 are PDS slots $C-$E.
 */
#define MAC_PSEUDOVIA_SLOT_LINES 7

/* named GPIO output "port-a": V8 port A writes (bits 0-7) */
/* named GPIO output "config": register 1 as written (bits 0-7) */
/* named GPIO output "video": register $10 as written (bits 0-7) */
/* named GPIO output "port-b": port B as written (bits 0-7) */

struct MacPseudoVIAState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    qemu_irq irq;
    qemu_irq port_a[8];
    qemu_irq config_out[8];
    qemu_irq video_out[8];
    qemu_irq portb_out[8];

    uint8_t variant;
    uint8_t regs[0x20];
    uint8_t slot_lines;         /* asserted slot interrupt inputs */
    uint8_t ifr_lines;          /* asserted level inputs (IFR bits) */
    uint8_t portb_in;           /* what port B reads */
    uint8_t config_in;          /* what register 1 reads */
    uint8_t video_in;           /* register $10 bits 3-5 (RBV monitor) */
};

#endif

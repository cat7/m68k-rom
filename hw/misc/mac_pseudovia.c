/*
 * Apple "pseudo-VIA"
 *
 * The RBV (Mac IIci/IIsi), V8 (LC, LC II) and Sonora (LC III) chips stand
 * in for the second VIA with a block that keeps a VIA's register layout
 * for the few registers it has, but has no timers, shift register or
 * data direction registers:
 *
 *   $00  port B
 *   $01  configuration (RBV, Sonora: RAM; written out on "config")
 *   $02  slot interrupts, active low: bit 6 built-in video VBL,
 *        bits 0-5 slots $9-$E; a write of bit 6 acknowledges the VBL
 *   $03  IFR: bit 0 SCSI DRQ, 1 any slot, 3 SCSI IRQ, 4 sound;
 *        bit 7 = any enabled one; writing 1s clears edge sources
 *   $04-$05  Sonora only: more configuration, read back as written
 *   $10  video configuration (RBV: bits 3-5 monitor sense)
 *   $12  slot interrupt enable \  writes with bit 7 set set the 1 bits,
 *   $13  IER                   /  with bit 7 clear clear them
 *
 * The RBV decodes only A0, A1 and A4; V8 and Sonora also A2-A3. A V8
 * write with A9 set goes to port A.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "hw/misc/mac_pseudovia.h"
#include "trace.h"

#define PV_PORTB        0x00
#define PV_CONFIG       0x01
#define PV_SLOT_IFR     0x02
#define PV_IFR          0x03
#define PV_VIDEO        0x10
#define PV_SLOT_IER     0x12
#define PV_IER          0x13

#define PV_IFR_SCSI_DRQ 0x01
#define PV_IFR_SLOT     0x02
#define PV_IFR_SCSI_IRQ 0x08
#define PV_IFR_ASC      0x10
#define PV_IFR_ANY      0x80

#define PV_SLOT_VBL     0x40

static const uint8_t pv_input_bit[MAC_PSEUDOVIA_IN_NB] = {
    [MAC_PSEUDOVIA_SCSI_DRQ] = PV_IFR_SCSI_DRQ,
    [MAC_PSEUDOVIA_SCSI_IRQ] = PV_IFR_SCSI_IRQ,
    [MAC_PSEUDOVIA_ASC] = PV_IFR_ASC,
};

/* pending slot interrupts: the lines, but the VBL only until it is acked */
static uint8_t pv_slots(MacPseudoVIAState *s)
{
    return ((s->slot_lines & ~PV_SLOT_VBL) | ~s->regs[PV_SLOT_IFR]) & 0x7f;
}

static void pv_update_irq(MacPseudoVIAState *s)
{
    uint8_t ifr = s->regs[PV_IFR] & 0x7f;
    uint8_t slots = pv_slots(s);

    /* level sources follow their lines */
    ifr &= ~(PV_IFR_SCSI_DRQ | PV_IFR_SCSI_IRQ | PV_IFR_SLOT);
    ifr |= s->ifr_lines & (PV_IFR_SCSI_DRQ | PV_IFR_SCSI_IRQ);
    if (s->variant != MAC_PSEUDOVIA_RBV) {
        /* V8 and Sonora: the sound interrupt is a level too */
        ifr = (ifr & ~PV_IFR_ASC) | (s->ifr_lines & PV_IFR_ASC);
    }
    if (slots & s->regs[PV_SLOT_IER] & 0x7f) {
        ifr |= PV_IFR_SLOT;
    }
    if (ifr & s->regs[PV_IER] & 0x7f) {
        ifr |= PV_IFR_ANY;
    }
    s->regs[PV_IFR] = ifr;
    qemu_set_irq(s->irq, !!(ifr & PV_IFR_ANY));
}

static void pv_set_input(void *opaque, int n, int level)
{
    MacPseudoVIAState *s = opaque;
    uint8_t bit = pv_input_bit[n];
    bool rise = level && !(s->ifr_lines & bit);

    s->ifr_lines = level ? (s->ifr_lines | bit) : (s->ifr_lines & ~bit);
    if (rise && bit == PV_IFR_ASC) {
        /* RBV: the sound interrupt is latched on its edge */
        s->regs[PV_IFR] |= PV_IFR_ASC;
    }
    pv_update_irq(s);
}

static void pv_set_slot(void *opaque, int n, int level)
{
    MacPseudoVIAState *s = opaque;
    uint8_t bit = BIT(n);

    if (level && !(s->slot_lines & bit) && bit == PV_SLOT_VBL) {
        /* the VBL stays flagged until the handler acknowledges it */
        s->regs[PV_SLOT_IFR] &= ~PV_SLOT_VBL;
    }
    s->slot_lines = level ? (s->slot_lines | bit) : (s->slot_lines & ~bit);
    pv_update_irq(s);
}

static hwaddr pv_reg(MacPseudoVIAState *s, hwaddr addr)
{
    return addr & (s->variant == MAC_PSEUDOVIA_RBV ? 0x13 : 0x1f);
}

static uint64_t pv_read(void *opaque, hwaddr addr, unsigned size)
{
    MacPseudoVIAState *s = opaque;
    hwaddr reg = pv_reg(s, addr);
    uint8_t val = s->regs[reg];

    switch (reg) {
    case PV_PORTB:
        val = s->portb_in;
        break;
    case PV_CONFIG:
        /*
         * Sonora and V8 read back their RAM configuration; V8's bits 1-2
         * come from config-in (VRAM: bit 2 present, bit 1 only 256 KB)
         */
        if (s->variant == MAC_PSEUDOVIA_SONORA) {
            break;
        }
        val = s->variant == MAC_PSEUDOVIA_V8 ?
              (val & ~0x06) | (s->config_in & 0x06) : s->config_in;
        break;
    case PV_SLOT_IFR:
        val = ~pv_slots(s) & 0x7f;
        break;
    case PV_VIDEO:
        val = (val & ~0x38) | (s->video_in & 0x38);
        break;
    case PV_SLOT_IER:
    case PV_IER:
        val &= 0x7f;
        break;
    }
    trace_mac_pseudovia_read(reg, val);
    return val;
}

static void pv_write(void *opaque, hwaddr addr, uint64_t val64,
                     unsigned size)
{
    MacPseudoVIAState *s = opaque;
    hwaddr reg = pv_reg(s, addr);
    uint8_t val = val64;
    int i;

    trace_mac_pseudovia_write(reg, val);
    if (s->variant == MAC_PSEUDOVIA_V8 && (addr & 0x200)) {
        for (i = 0; i < 8; i++) {
            qemu_set_irq(s->port_a[i], (val >> i) & 1);
        }
        return;
    }
    switch (reg) {
    case PV_PORTB:
        s->regs[PV_PORTB] = val;
        for (i = 0; i < 8; i++) {
            qemu_set_irq(s->portb_out[i], (val >> i) & 1);
        }
        break;
    case PV_CONFIG:
        s->regs[PV_CONFIG] = val;
        for (i = 0; i < 8; i++) {
            qemu_set_irq(s->config_out[i], (val >> i) & 1);
        }
        break;
    case PV_VIDEO:
        s->regs[PV_VIDEO] = val;
        for (i = 0; i < 8; i++) {
            qemu_set_irq(s->video_out[i], (val >> i) & 1);
        }
        break;
    case PV_SLOT_IFR:
        /* writing bit 6 acknowledges the VBL */
        s->regs[PV_SLOT_IFR] |= val & PV_SLOT_VBL;
        break;
    case PV_IFR:
        s->regs[PV_IFR] &= ~(val & 0x7f);
        break;
    case PV_SLOT_IER:
    case PV_IER:
        if (val & 0x80) {
            s->regs[reg] |= val & 0x7f;
        } else {
            s->regs[reg] &= ~(val & 0x7f);
        }
        break;
    default:
        s->regs[reg] = val;
        break;
    }
    pv_update_irq(s);
}

static const MemoryRegionOps pv_ops = {
    .read = pv_read,
    .write = pv_write,
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

static void pv_reset_hold(Object *obj, ResetType type)
{
    MacPseudoVIAState *s = MAC_PSEUDOVIA(obj);

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[PV_SLOT_IFR] = 0x7f;
    pv_update_irq(s);
}

static void pv_init(Object *obj)
{
    MacPseudoVIAState *s = MAC_PSEUDOVIA(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->mmio, obj, &pv_ops, s, "mac-pseudovia",
                          0x2000);
    sysbus_init_mmio(sbd, &s->mmio);
    sysbus_init_irq(sbd, &s->irq);
    qdev_init_gpio_in(DEVICE(obj), pv_set_input, MAC_PSEUDOVIA_IN_NB);
    qdev_init_gpio_in_named(DEVICE(obj), pv_set_slot, "slot",
                            MAC_PSEUDOVIA_SLOT_LINES);
    qdev_init_gpio_out_named(DEVICE(obj), s->port_a, "port-a", 8);
    qdev_init_gpio_out_named(DEVICE(obj), s->config_out, "config", 8);
    qdev_init_gpio_out_named(DEVICE(obj), s->video_out, "video", 8);
    qdev_init_gpio_out_named(DEVICE(obj), s->portb_out, "port-b", 8);
}

static const VMStateDescription vmstate_mac_pseudovia = {
    .name = "mac-pseudovia",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8_ARRAY(regs, MacPseudoVIAState, 0x20),
        VMSTATE_UINT8(slot_lines, MacPseudoVIAState),
        VMSTATE_UINT8(ifr_lines, MacPseudoVIAState),
        VMSTATE_END_OF_LIST()
    }
};

static const Property pv_properties[] = {
    DEFINE_PROP_UINT8("variant", MacPseudoVIAState, variant,
                      MAC_PSEUDOVIA_SONORA),
    DEFINE_PROP_UINT8("portb-in", MacPseudoVIAState, portb_in, 0),
    DEFINE_PROP_UINT8("config-in", MacPseudoVIAState, config_in, 0),
    DEFINE_PROP_UINT8("video-in", MacPseudoVIAState, video_in, 0),
};

static void pv_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->desc = "Apple pseudo-VIA (RBV, V8, Sonora)";
    dc->vmsd = &vmstate_mac_pseudovia;
    rc->phases.hold = pv_reset_hold;
    device_class_set_props(dc, pv_properties);
}

static const TypeInfo pv_info = {
    .name = TYPE_MAC_PSEUDOVIA,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MacPseudoVIAState),
    .instance_init = pv_init,
    .class_init = pv_class_init,
};

static void pv_register_types(void)
{
    type_register_static(&pv_info);
}

type_init(pv_register_types)

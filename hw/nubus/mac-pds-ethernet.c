/*
 * SONIC Ethernet cards for the Macintosh: the LC processor-direct slot
 * card (mac-pds-ethernet) and the NuBus card (nubus-ethernet)
 *
 * Both cards carry a DP83932 SONIC, an address PROM and a declaration ROM
 * at the top of the slot space.  Mac OS picks its driver by the board ID
 * in the declaration ROM; the 'enet' driver and its 'ecf2' config resource
 * already name all the addresses, so the ROM needs no driver code.
 *
 * mac-pds-ethernet, board ID 93 ("Sonic 16"): the LC-family PDS shows up
 * as NuBus slot $E.  Registers (one 16-bit register per longword) at the
 * start of the slot space, PROM at +$40000, and the SONIC does its DMA
 * straight into host memory.  The
 * driver asks Mac OS for physical addresses (GetPhysical), so it needs a
 * System where that call exists; the Mac II family has it only with
 * virtual memory on.
 *
 * nubus-ethernet, board ID 87 or 38 ("Sonic 16 NB"): the card has 128 KB
 * of its own buffer RAM at the start of the slot space, and the driver
 * keeps every SONIC buffer and descriptor there.  It hands the SONIC
 * NuBus addresses ($Fs0xxxxx), so its DMA goes over the bus into that RAM.
 *
 *                  board 87        board 38
 *   buffer RAM     +$000000        +$000000    128 KB
 *   SONIC          +$0C0000        +$900000    (38: 32-bit mode only)
 *   address PROM   +$400001        +$700001    one byte every 2 bytes
 *   card control   +$C2000x        +$CA000x    written once at open
 *
 * Its SONIC registers are one 16-bit word each, 2 bytes apart.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/bitops.h"
#include "qemu/log.h"
#include "exec/target_page.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/nubus/nubus.h"
#include "hw/nubus/nubus-decl-rom.h"
#include "hw/net/dp8393x.h"
#include "net/net.h"
#include "system/address-spaces.h"
#include "qom/object.h"

#define TYPE_MAC_ENET_CARD    "mac-enet-card"
#define TYPE_MAC_PDS_ETHERNET "mac-pds-ethernet"
#define TYPE_NUBUS_ETHERNET   "nubus-ethernet"
OBJECT_DECLARE_SIMPLE_TYPE(MacEnetState, MAC_ENET_CARD)

#define PDS_ENET_REGS_OFFSET    0x00000
#define PDS_ENET_PROM_OFFSET    0x40000
#define ENET_PROM_SIZE          0x1000

#define PDS_ENET_BOARD_ID       93      /* Apple "Sonic 16" LC card */
#define ENET_DRHW               0x0119

#define NB_ENET_RAM_SIZE        0x20000
#define NB_ENET_CTRL_OFFSET     0xc00000
#define NB_ENET_CTRL_SIZE       0x100000

struct MacEnetState {
    NubusDevice parent_obj;

    dp8393xState sonic;
    MemoryRegion prom;
    MemoryRegion decl_rom;

    /* nubus-ethernet only */
    MemoryRegion ram;
    MemoryRegion ctrl;
    uint32_t board_id;
};

static void enet_build_decl_rom(uint8_t *buf, int size, int board_id,
                                const char *board_name)
{
    NubusDeclROM r;
    int dir, dir_board, dir_net;
    int b_type, b_name, b_vendor, n_type, n_name, v_id;

    nubus_decl_rom_init(&r, buf, size);

    /* sResource directory */
    dir = r.pos;
    dir_board = nubus_decl_rom_entry_off(&r, 0x01);
    dir_net = nubus_decl_rom_entry_off(&r, 0x80);
    nubus_decl_rom_end(&r);

    /* Board sResource */
    nubus_decl_rom_here(&r, dir_board);
    b_type = nubus_decl_rom_entry_off(&r, NUBUS_sRsrcType);
    b_name = nubus_decl_rom_entry_off(&r, NUBUS_sRsrcName);
    nubus_decl_rom_entry_data(&r, NUBUS_BoardId, board_id);
    b_vendor = nubus_decl_rom_entry_off(&r, NUBUS_VendorInfo);
    nubus_decl_rom_end(&r);

    /* Ethernet sResource */
    nubus_decl_rom_here(&r, dir_net);
    n_type = nubus_decl_rom_entry_off(&r, NUBUS_sRsrcType);
    n_name = nubus_decl_rom_entry_off(&r, NUBUS_sRsrcName);
    nubus_decl_rom_entry_data(&r, NUBUS_sRsrcHWDevId, 1);
    nubus_decl_rom_end(&r);

    nubus_decl_rom_here(&r, b_vendor);
    v_id = nubus_decl_rom_entry_off(&r, NUBUS_VendorId);
    nubus_decl_rom_end(&r);

    nubus_decl_rom_here(&r, b_type);
    nubus_decl_rom_word(&r, NUBUS_catBoard);
    nubus_decl_rom_word(&r, 0x0000);
    nubus_decl_rom_word(&r, 0x0000);
    nubus_decl_rom_word(&r, 0x0000);

    nubus_decl_rom_here(&r, n_type);
    nubus_decl_rom_word(&r, NUBUS_catNetwork);
    nubus_decl_rom_word(&r, NUBUS_typeEthernet);
    nubus_decl_rom_word(&r, NUBUS_drSwApple);
    nubus_decl_rom_word(&r, ENET_DRHW);

    nubus_decl_rom_here(&r, b_name);
    nubus_decl_rom_cstring(&r, board_name);
    nubus_decl_rom_here(&r, n_name);
    nubus_decl_rom_cstring(&r, "Network_Ethernet_SONIC");
    nubus_decl_rom_here(&r, v_id);
    nubus_decl_rom_cstring(&r, "QEMU");

    nubus_decl_rom_finish(&r, dir);
}

static void mac_enet_irq(void *opaque, int n, int level)
{
    nubus_set_irq(NUBUS_DEVICE(opaque), level);
}

/*
 * The SONIC, the address PROM and the declaration ROM, common to both
 * cards.  The PROM holds the address bit-reversed, then a check byte that
 * makes all 8 bytes XOR to $FF; byte n is at @first + n * @stride.
 */
static bool mac_enet_realize_common(MacEnetState *s, DeviceState *dev,
                                    hwaddr regs, int reg_shift,
                                    MemoryRegion *dma_mr,
                                    hwaddr prom_at, int first, int stride,
                                    int board_id,
                                    const char *board_name, Error **errp)
{
    NubusDevice *nd = NUBUS_DEVICE(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(&s->sonic);
    uint8_t *mac, *prom;
    int i, checksum;

    /* The Apple drivers insist on an 08:00:07 address */
    qemu_macaddr_default_if_unset(&s->sonic.conf.macaddr);
    mac = s->sonic.conf.macaddr.a;
    mac[0] = 0x08;
    mac[1] = 0x00;
    mac[2] = 0x07;

    qdev_prop_set_uint8(DEVICE(sbd), "it_shift", reg_shift);
    qdev_prop_set_bit(DEVICE(sbd), "big_endian", true);
    object_property_set_link(OBJECT(sbd), "dma_mr", OBJECT(dma_mr),
                             &error_abort);
    if (!sysbus_realize(sbd, errp)) {
        return false;
    }
    memory_region_add_subregion(&nd->slot_mem, regs,
                                sysbus_mmio_get_region(sbd, 0));
    sysbus_connect_irq(sbd, 0, qdev_get_gpio_in_named(dev, "sonic-irq", 0));

    memory_region_init_rom(&s->prom, OBJECT(dev), "mac-enet.prom",
                           ENET_PROM_SIZE, &error_fatal);
    prom = memory_region_get_ram_ptr(&s->prom);
    memset(prom, 0, ENET_PROM_SIZE);
    checksum = 0;
    for (i = 0; i < 6; i++) {
        prom[first + i * stride] = revbit8(mac[i]);
        checksum ^= revbit8(mac[i]);
    }
    prom[first + 7 * stride] = 0xff - checksum;
    memory_region_add_subregion(&nd->slot_mem, prom_at, &s->prom);

    memory_region_init_rom(&s->decl_rom, OBJECT(dev), "mac-enet.decl-rom",
                           qemu_target_page_size(), &error_fatal);
    enet_build_decl_rom(memory_region_get_ram_ptr(&s->decl_rom),
                        qemu_target_page_size(), board_id, board_name);
    memory_region_add_subregion(&nd->slot_mem,
                                NUBUS_SLOT_SIZE - qemu_target_page_size(),
                                &s->decl_rom);
    return true;
}

static void mac_pds_ethernet_realize(DeviceState *dev, Error **errp)
{
    ERRP_GUARD();
    MacEnetState *s = MAC_ENET_CARD(dev);
    DeviceClass *pdc = DEVICE_CLASS(object_class_get_parent(
                                        object_get_class(OBJECT(dev))));

    pdc->realize(dev, errp);
    if (*errp) {
        return;
    }
    mac_enet_realize_common(s, dev, PDS_ENET_REGS_OFFSET, 2,
                            get_system_memory(),
                            PDS_ENET_PROM_OFFSET, 0, 1, PDS_ENET_BOARD_ID,
                            "QEMU PDS Ethernet", errp);
}

/*
 * The card control registers: the driver clears a few of them and sets
 * one ($07 or $70) at open, and never reads them back.
 */
static uint64_t nb_enet_ctrl_read(void *opaque, hwaddr addr, unsigned size)
{
    qemu_log_mask(LOG_UNIMP, "nubus-ethernet: control read at 0x%"
                  HWADDR_PRIx "\n", addr);
    return 0;
}

static void nb_enet_ctrl_write(void *opaque, hwaddr addr, uint64_t val,
                               unsigned size)
{
}

static const MemoryRegionOps nb_enet_ctrl_ops = {
    .read = nb_enet_ctrl_read,
    .write = nb_enet_ctrl_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static void nubus_ethernet_realize(DeviceState *dev, Error **errp)
{
    ERRP_GUARD();
    MacEnetState *s = MAC_ENET_CARD(dev);
    NubusDevice *nd = NUBUS_DEVICE(dev);
    DeviceClass *pdc = DEVICE_CLASS(object_class_get_parent(
                                        object_get_class(OBJECT(dev))));
    hwaddr regs, prom;

    switch (s->board_id) {
    case 87:
        regs = 0x0c0000;
        prom = 0x400000;
        break;
    case 38:
        regs = 0x900000;
        prom = 0x700000;
        break;
    default:
        error_setg(errp, "nubus-ethernet: board-id must be 87 or 38");
        return;
    }

    pdc->realize(dev, errp);
    if (*errp) {
        return;
    }

    memory_region_init_ram(&s->ram, OBJECT(dev), "nubus-ethernet.ram",
                           NB_ENET_RAM_SIZE, errp);
    if (*errp) {
        return;
    }
    memory_region_add_subregion(&nd->slot_mem, 0, &s->ram);

    memory_region_init_io(&s->ctrl, OBJECT(dev), &nb_enet_ctrl_ops, s,
                          "nubus-ethernet.ctrl", NB_ENET_CTRL_SIZE);
    memory_region_add_subregion(&nd->slot_mem, NB_ENET_CTRL_OFFSET,
                                &s->ctrl);

    /* PROM bytes on the odd addresses */
    mac_enet_realize_common(s, dev, regs, 1, get_system_memory(), prom, 1, 2,
                            s->board_id, "QEMU NuBus Ethernet", errp);
}

static void mac_enet_init(Object *obj)
{
    MacEnetState *s = MAC_ENET_CARD(obj);

    object_initialize_child(obj, "sonic", &s->sonic, TYPE_DP8393X);
    object_property_add_alias(obj, "mac", OBJECT(&s->sonic), "mac");
    object_property_add_alias(obj, "netdev", OBJECT(&s->sonic), "netdev");
    qdev_init_gpio_in_named(DEVICE(obj), mac_enet_irq, "sonic-irq", 1);
}

static void mac_pds_ethernet_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = mac_pds_ethernet_realize;
    dc->desc = "SONIC Ethernet card for the Macintosh LC PDS slot";
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
}

static const Property nubus_ethernet_properties[] = {
    DEFINE_PROP_UINT32("board-id", MacEnetState, board_id, 87),
};

static void nubus_ethernet_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = nubus_ethernet_realize;
    dc->desc = "SONIC Ethernet NuBus card with on-board buffer RAM";
    device_class_set_props(dc, nubus_ethernet_properties);
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
}

static const TypeInfo mac_enet_types[] = {
    {
        .name = TYPE_MAC_ENET_CARD,
        .parent = TYPE_NUBUS_DEVICE,
        .instance_size = sizeof(MacEnetState),
        .instance_init = mac_enet_init,
        .abstract = true,
    }, {
        .name = TYPE_MAC_PDS_ETHERNET,
        .parent = TYPE_MAC_ENET_CARD,
        .class_init = mac_pds_ethernet_class_init,
    }, {
        .name = TYPE_NUBUS_ETHERNET,
        .parent = TYPE_MAC_ENET_CARD,
        .class_init = nubus_ethernet_class_init,
    },
};

DEFINE_TYPES(mac_enet_types)

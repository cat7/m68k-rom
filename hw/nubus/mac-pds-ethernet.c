/*
 * SONIC Ethernet card for the Macintosh LC processor-direct slot
 *
 * The LC-family PDS shows up to Mac OS as NuBus slot $E.  The card has a
 * DP83932 SONIC with its registers at the start of the slot space (one
 * 16-bit register per longword, 16-bit DMA into host memory), an address
 * PROM at +$40000 and a declaration ROM at the top of the slot space.
 *
 * Mac OS picks its driver by the board ID in the declaration ROM: board
 * ID 93 loads the "Sonic 16" 'enet' driver and the "0:93" 'ecf2' config,
 * which already name these addresses, so the ROM needs no driver code.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/bitops.h"
#include "exec/target_page.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/nubus/nubus.h"
#include "hw/nubus/nubus-decl-rom.h"
#include "hw/net/dp8393x.h"
#include "net/net.h"
#include "system/address-spaces.h"
#include "qom/object.h"

#define TYPE_MAC_PDS_ETHERNET "mac-pds-ethernet"
OBJECT_DECLARE_SIMPLE_TYPE(MacPDSEthernetState, MAC_PDS_ETHERNET)

#define PDS_ENET_REGS_OFFSET    0x00000
#define PDS_ENET_PROM_OFFSET    0x40000
#define PDS_ENET_PROM_SIZE      0x1000

#define PDS_ENET_BOARD_ID       93      /* Apple "Sonic 16" LC card */
#define PDS_ENET_DRHW           0x0119

struct MacPDSEthernetState {
    NubusDevice parent_obj;

    dp8393xState sonic;
    MemoryRegion prom;
    MemoryRegion decl_rom;
};

static void mac_pds_ethernet_build_rom(uint8_t *buf, int size)
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
    nubus_decl_rom_entry_data(&r, NUBUS_BoardId, PDS_ENET_BOARD_ID);
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
    nubus_decl_rom_word(&r, PDS_ENET_DRHW);

    nubus_decl_rom_here(&r, b_name);
    nubus_decl_rom_cstring(&r, "QEMU PDS Ethernet");
    nubus_decl_rom_here(&r, n_name);
    nubus_decl_rom_cstring(&r, "Network_Ethernet_SONIC_PDS");
    nubus_decl_rom_here(&r, v_id);
    nubus_decl_rom_cstring(&r, "QEMU");

    nubus_decl_rom_finish(&r, dir);
}

static void mac_pds_ethernet_irq(void *opaque, int n, int level)
{
    nubus_set_irq(NUBUS_DEVICE(opaque), level);
}

static void mac_pds_ethernet_realize(DeviceState *dev, Error **errp)
{
    ERRP_GUARD();
    MacPDSEthernetState *s = MAC_PDS_ETHERNET(dev);
    NubusDevice *nd = NUBUS_DEVICE(dev);
    DeviceClass *pdc = DEVICE_CLASS(object_class_get_parent(
                                        object_get_class(OBJECT(dev))));
    SysBusDevice *sbd = SYS_BUS_DEVICE(&s->sonic);
    uint8_t *mac, *prom;
    int i, checksum;

    pdc->realize(dev, errp);
    if (*errp) {
        return;
    }

    /* The Apple driver insists on an 08:00:07 address */
    qemu_macaddr_default_if_unset(&s->sonic.conf.macaddr);
    mac = s->sonic.conf.macaddr.a;
    mac[0] = 0x08;
    mac[1] = 0x00;
    mac[2] = 0x07;

    qdev_prop_set_uint8(DEVICE(sbd), "it_shift", 2);
    qdev_prop_set_bit(DEVICE(sbd), "big_endian", true);
    object_property_set_link(OBJECT(sbd), "dma_mr",
                             OBJECT(get_system_memory()), &error_abort);
    if (!sysbus_realize(sbd, errp)) {
        return;
    }
    memory_region_add_subregion(&nd->slot_mem, PDS_ENET_REGS_OFFSET,
                                sysbus_mmio_get_region(sbd, 0));
    sysbus_connect_irq(sbd, 0, qdev_get_gpio_in_named(dev, "sonic-irq", 0));

    /* Address PROM: bit-reversed bytes, as on the built-in Ethernet */
    memory_region_init_rom(&s->prom, OBJECT(dev), "mac-pds-ethernet.prom",
                           PDS_ENET_PROM_SIZE, &error_fatal);
    prom = memory_region_get_ram_ptr(&s->prom);
    memset(prom, 0, PDS_ENET_PROM_SIZE);
    checksum = 0;
    for (i = 0; i < 6; i++) {
        prom[i] = revbit8(mac[i]);
        checksum ^= prom[i];
    }
    prom[7] = 0xff - checksum;
    memory_region_add_subregion(&nd->slot_mem, PDS_ENET_PROM_OFFSET,
                                &s->prom);

    /* Declaration ROM */
    memory_region_init_rom(&s->decl_rom, OBJECT(dev),
                           "mac-pds-ethernet.decl-rom",
                           qemu_target_page_size(), &error_fatal);
    mac_pds_ethernet_build_rom(memory_region_get_ram_ptr(&s->decl_rom),
                               qemu_target_page_size());
    memory_region_add_subregion(&nd->slot_mem,
                                NUBUS_SLOT_SIZE - qemu_target_page_size(),
                                &s->decl_rom);
}

static void mac_pds_ethernet_init(Object *obj)
{
    MacPDSEthernetState *s = MAC_PDS_ETHERNET(obj);

    object_initialize_child(obj, "sonic", &s->sonic, TYPE_DP8393X);
    object_property_add_alias(obj, "mac", OBJECT(&s->sonic), "mac");
    object_property_add_alias(obj, "netdev", OBJECT(&s->sonic), "netdev");
    qdev_init_gpio_in_named(DEVICE(obj), mac_pds_ethernet_irq,
                            "sonic-irq", 1);
}

static void mac_pds_ethernet_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = mac_pds_ethernet_realize;
    dc->desc = "SONIC Ethernet card for the Macintosh LC PDS slot";
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
}

static const TypeInfo mac_pds_ethernet_info = {
    .name = TYPE_MAC_PDS_ETHERNET,
    .parent = TYPE_NUBUS_DEVICE,
    .instance_size = sizeof(MacPDSEthernetState),
    .instance_init = mac_pds_ethernet_init,
    .class_init = mac_pds_ethernet_class_init,
};

static void mac_pds_ethernet_register_types(void)
{
    type_register_static(&mac_pds_ethernet_info);
}

type_init(mac_pds_ethernet_register_types)

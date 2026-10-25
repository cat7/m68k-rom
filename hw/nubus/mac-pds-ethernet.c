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

#define PDS_ENET_DECL_ROM_SIZE  0x100

struct MacPDSEthernetState {
    NubusDevice parent_obj;

    dp8393xState sonic;
    MemoryRegion prom;
    MemoryRegion decl_rom;
};

/*
 * Declaration ROM builder: all four byte lanes, so ROM offsets are plain
 * byte offsets.
 */
typedef struct DeclROM {
    uint8_t *buf;
    int pos;
} DeclROM;

static void rom_byte(DeclROM *r, uint8_t v)
{
    r->buf[r->pos++] = v;
}

static void rom_word(DeclROM *r, uint16_t v)
{
    rom_byte(r, v >> 8);
    rom_byte(r, v);
}

static void rom_long(DeclROM *r, uint32_t v)
{
    rom_word(r, v >> 16);
    rom_word(r, v);
}

/* An entry with a 24-bit offset to @target (filled in later if 0) */
static int rom_entry_off(DeclROM *r, uint8_t id, int target)
{
    int at = r->pos;

    rom_long(r, ((uint32_t)id << 24) | ((target - at) & 0xffffff));
    return at;
}

static void rom_patch_off(DeclROM *r, int at, int target)
{
    uint32_t v = ((uint32_t)r->buf[at] << 24) | ((target - at) & 0xffffff);

    stl_be_p(r->buf + at, v);
}

static void rom_entry_data(DeclROM *r, uint8_t id, uint32_t data)
{
    rom_long(r, ((uint32_t)id << 24) | (data & 0xffffff));
}

static void rom_end(DeclROM *r)
{
    rom_long(r, 0xff000000);
}

static void rom_cstring(DeclROM *r, const char *s)
{
    do {
        rom_byte(r, *s);
    } while (*s++);
    while (r->pos & 1) {
        rom_byte(r, 0);
    }
}

#define sRsrcType       1
#define sRsrcName       2
#define sRsrcHWDevId    8
#define BoardId         32
#define VendorInfo      36
#define VendorId        1

static void mac_pds_ethernet_build_rom(uint8_t *buf, int size)
{
    DeclROM r = { .buf = buf, .pos = 0 };
    int dir_board, dir_net;
    int b_type, b_name, b_vendor, n_type, n_name, v_id;
    int board, net, vendor, p;
    uint32_t crc;
    int i, len, start;

    /* sResource directory */
    dir_board = rom_entry_off(&r, 0x01, 0);
    dir_net = rom_entry_off(&r, 0x80, 0);
    rom_end(&r);

    /* Board sResource */
    board = r.pos;
    b_type = rom_entry_off(&r, sRsrcType, 0);
    b_name = rom_entry_off(&r, sRsrcName, 0);
    rom_entry_data(&r, BoardId, PDS_ENET_BOARD_ID);
    b_vendor = rom_entry_off(&r, VendorInfo, 0);
    rom_end(&r);

    /* Ethernet sResource */
    net = r.pos;
    n_type = rom_entry_off(&r, sRsrcType, 0);
    n_name = rom_entry_off(&r, sRsrcName, 0);
    rom_entry_data(&r, sRsrcHWDevId, 1);
    rom_end(&r);

    vendor = r.pos;
    v_id = rom_entry_off(&r, VendorId, 0);
    rom_end(&r);

    p = r.pos;
    rom_patch_off(&r, b_type, p);
    rom_word(&r, 0x0001);               /* catBoard */
    rom_word(&r, 0x0000);
    rom_word(&r, 0x0000);
    rom_word(&r, 0x0000);

    p = r.pos;
    rom_patch_off(&r, n_type, p);
    rom_word(&r, 0x0004);               /* catNetwork */
    rom_word(&r, 0x0001);               /* typeEthernet */
    rom_word(&r, 0x0001);               /* drSwApple */
    rom_word(&r, PDS_ENET_DRHW);

    p = r.pos;
    rom_patch_off(&r, b_name, p);
    rom_cstring(&r, "QEMU PDS Ethernet");
    p = r.pos;
    rom_patch_off(&r, n_name, p);
    rom_cstring(&r, "Network_Ethernet_SONIC_PDS");
    p = r.pos;
    rom_patch_off(&r, v_id, p);
    rom_cstring(&r, "QEMU");

    rom_patch_off(&r, dir_board, board);
    rom_patch_off(&r, dir_net, net);
    rom_patch_off(&r, b_vendor, vendor);

    len = r.pos + 20;
    g_assert(len <= size);

    /* The ROM sits at the top of the slot space: move it there */
    start = size - len;
    memmove(buf + start, buf, r.pos);
    memset(buf, 0, start);

    /* Format block, ending at the last byte */
    r.pos = start + r.pos;
    rom_long(&r, (uint32_t)(start - r.pos) & 0xffffff);  /* fhDirOffset */
    rom_long(&r, len);                                    /* fhLength */
    rom_long(&r, 0);                                      /* fhCRC */
    rom_byte(&r, 1);                                      /* fhROMRev */
    rom_byte(&r, 1);                                      /* appleFormat */
    rom_long(&r, 0x5a932bc7);                             /* fhTstPat */
    rom_byte(&r, 0);                                      /* fhReserved */
    rom_byte(&r, 0x0f);                                   /* fhByteLanes */
    g_assert(r.pos == size);

    /* CRC: rotate left and add each byte, the CRC field counting as 0 */
    crc = 0;
    for (i = start; i < size; i++) {
        crc = rol32(crc, 1);
        if (i < size - 12 || i >= size - 8) {
            crc += buf[i];
        }
    }
    stl_be_p(buf + size - 12, crc);
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

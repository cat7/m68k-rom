/*
 * Macintosh IIfx
 *
 * 68030 + 68882 at 40 MHz. The OSS chip replaces VIA2 as the interrupt
 * controller (and controls the reset overlay), two IOPs run the serial
 * port (SCC IOP) and the floppy and ADB (SWIM IOP), the FMC memory
 * controller places RAM bank B, and the SCSI chip is a 53C80 inside a DMA
 * ASIC. Six NuBus slots ($9-$E), no built-in video. 24-bit mode is done by
 * the ROM with the 68030 MMU.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/datadir.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "system/system.h"
#include "system/qtest.h"
#include "system/reset.h"
#include "system/address-spaces.h"
#include "system/block-backend.h"
#include "hw/core/boards.h"
#include "hw/core/loader.h"
#include "hw/core/irq.h"
#include "hw/core/or-irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/sysbus.h"
#include "hw/misc/unimp.h"
#include "hw/char/escc.h"
#include "hw/misc/mac_via.h"
#include "hw/misc/mac_iop.h"
#include "hw/misc/mac_oss.h"
#include "hw/audio/asc.h"
#include "hw/nubus/mac-nubus-bridge.h"
#include "hw/input/adb.h"
#include "target/m68k/cpu.h"
#include "qom/object.h"

#define IIFX_ROM_ADDR       0x40000000
#define IIFX_ROM_SIZE       (512 * KiB)
#define IIFX_ROM_WINDOW     (16 * MiB)

#define IIFX_IO_BASE        0x50000000
#define IIFX_IO_SLICE       0x00040000      /* the I/O block repeats */
#define IIFX_IO_SIZE        0x01000000

#define VIA1_OFFSET         0x00000
#define SCC_IOP_OFFSET      0x04000
#define SCC_OFFSET          0x04020         /* bypass mode */
#define SCSI_OFFSET         0x08000
#define ASC_OFFSET          0x10000
#define SWIM_IOP_OFFSET     0x12000
#define OSS_OFFSET          0x1a000
#define FMC_OFFSET          0x1c000

#define IIFX_RAM_SPACE      0x40000000
#define IIFX_BANK_WINDOW    (64 * MiB)

#define MAC_CLOCK           3686418
#define MAC_SCC_PCLK        7833600

#define TYPE_IIFX_MACHINE MACHINE_TYPE_NAME("maciifx")
OBJECT_DECLARE_SIMPLE_TYPE(MacIIfxMachineState, IIFX_MACHINE)

struct MacIIfxMachineState {
    MachineState parent_obj;

    M68kCPU cpu;
    MemoryRegion rom;
    MemoryRegion rom_mirror[IIFX_ROM_WINDOW / IIFX_ROM_SIZE];
    MemoryRegion overlay;
    MemoryRegion ram_space;
    MemoryRegion bank_a[IIFX_BANK_WINDOW / (4 * MiB)];
    MemoryRegion bank_b[IIFX_BANK_WINDOW / (4 * MiB)];
    MemoryRegion io;
    MemoryRegion io_alias[IIFX_IO_SIZE / IIFX_IO_SLICE];
    MemoryRegion fmc;

    MOS6522Q800VIA1State via1;
    MacOSSState oss;
    MacIOPState scc_iop;
    MacIOPState swim_iop;
    ESCCState escc;
    OrIRQState scc_orgate;
    ASCState asc;
    MacNubusBridge nubus_bridge;

    uint64_t bank_a_size;
    uint64_t bank_b_size;
    uint16_t fmc_shift;         /* FMC configuration being shifted in/out */
    uint16_t fmc_config;        /* latched: bits 7:6 place bank B */
};

static void iifx_set_ipl(void *opaque, int n, int level)
{
    MacIIfxMachineState *m = opaque;

    m68k_set_irq_level(&m->cpu, level, level ? 24 + level : 0);
}

static void iifx_set_overlay(void *opaque, int n, int level)
{
    MacIIfxMachineState *m = opaque;

    memory_region_set_enabled(&m->overlay, level);
}

/*
 * RAM: bank A at 0 and bank B at 4, 8, 16 or 64 MB (FMC bits 7:6); each
 * repeats through its 64 MB window, which the ROM's sizing relies on.
 */
static void iifx_map_bank(MacIIfxMachineState *m, MemoryRegion *alias,
                          uint64_t offset, uint64_t size, hwaddr base,
                          int priority)
{
    MachineState *machine = MACHINE(m);
    int i;

    for (i = 0; size && i < IIFX_BANK_WINDOW / size; i++) {
        if (memory_region_is_mapped(&alias[i])) {
            memory_region_del_subregion(&m->ram_space, &alias[i]);
            object_unparent(OBJECT(&alias[i]));
        }
        memory_region_init_alias(&alias[i], OBJECT(m), "maciifx.ram",
                                 machine->ram, offset, size);
        memory_region_add_subregion_overlap(&m->ram_space, base + i * size,
                                            &alias[i], priority);
    }
}

static void iifx_update_ram(MacIIfxMachineState *m)
{
    static const uint64_t bank_b_base[4] = {
        4 * MiB, 8 * MiB, 16 * MiB, 64 * MiB
    };

    memory_region_transaction_begin();
    iifx_map_bank(m, m->bank_a, 0, m->bank_a_size, 0, 0);
    iifx_map_bank(m, m->bank_b, m->bank_a_size, m->bank_b_size,
                  bank_b_base[(m->fmc_config >> 6) & 3], 1);
    memory_region_transaction_commit();
}

/*
 * FMC: a 16-bit configuration register loaded one bit at a time, LSB
 * first, through bit 0 of offset 0; a write to offset $10 latches it.
 * Reading offset 0 shifts it out the same way.
 */
static uint64_t iifx_fmc_read(void *opaque, hwaddr addr, unsigned size)
{
    MacIIfxMachineState *m = opaque;
    uint8_t bit = m->fmc_shift & 1;

    if (addr == 0) {
        m->fmc_shift = (m->fmc_shift >> 1) | (bit << 15);
        return bit;
    }
    return 0;
}

static void iifx_fmc_write(void *opaque, hwaddr addr, uint64_t val,
                           unsigned size)
{
    MacIIfxMachineState *m = opaque;

    if (addr == 0) {
        m->fmc_shift = (m->fmc_shift >> 1) | ((val & 1) << 15);
    } else if (addr == 0x10) {
        m->fmc_config = m->fmc_shift;
        iifx_update_ram(m);
    }
}

static const MemoryRegionOps iifx_fmc_ops = {
    .read = iifx_fmc_read,
    .write = iifx_fmc_write,
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

/* reads past the installed RAM return 0 */
static uint64_t open_bus_read(void *opaque, hwaddr addr, unsigned size)
{
    return 0;
}

static void open_bus_write(void *opaque, hwaddr addr, uint64_t val,
                           unsigned size)
{
}

static const MemoryRegionOps open_bus_ops = {
    .read = open_bus_read,
    .write = open_bus_write,
    .endianness = DEVICE_BIG_ENDIAN,
};

static void iifx_unimp(MacIIfxMachineState *m, const char *name,
                       hwaddr offset, uint64_t size)
{
    DeviceState *dev = qdev_new(TYPE_UNIMPLEMENTED_DEVICE);

    qdev_prop_set_string(dev, "name", name);
    qdev_prop_set_uint64(dev, "size", size);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    memory_region_add_subregion(&m->io, offset,
                                sysbus_mmio_get_region(SYS_BUS_DEVICE(dev),
                                                       0));
}

static void iifx_machine_reset(MachineState *machine, ResetType type)
{
    MacIIfxMachineState *m = IIFX_MACHINE(machine);
    CPUState *cs = CPU(&m->cpu);

    qemu_devices_reset(type);
    m->fmc_shift = m->fmc_config = 0xffff;
    iifx_update_ram(m);
    cpu_reset(cs);
    /* the reset vectors come through the overlay at 0 */
    m->cpu.env.aregs[7] = ldl_phys(cs->as, 0);
    m->cpu.env.pc = ldl_phys(cs->as, 4);
}

static void iifx_machine_init(MachineState *machine)
{
    MacIIfxMachineState *m = IIFX_MACHINE(machine);
    const char *bios_name = machine->firmware ?: "MacIIfx.rom";
    MemoryRegion *sysmem = get_system_memory();
    uint64_t ram_size = machine->ram_size;
    SysBusDevice *sysbus;
    DeviceState *dev, *oss;
    BusState *adb_bus;
    DriveInfo *dinfo;
    char *filename;
    int i, size;

    /* bank A: the biggest power of two up to 64 MB, bank B the rest */
    m->bank_a_size = MIN(pow2floor(ram_size), 64 * MiB);
    m->bank_b_size = ram_size - m->bank_a_size;
    if (m->bank_a_size < 4 * MiB ||
        (m->bank_b_size && (m->bank_b_size < 4 * MiB ||
                            !is_power_of_2(m->bank_b_size)))) {
        error_report("The Macintosh IIfx takes 4-128 MB in two banks of "
                     "4, 8, 16, 32 or 64 MB");
        exit(1);
    }

    object_initialize_child(OBJECT(machine), "cpu", &m->cpu,
                            machine->cpu_type);
    qdev_realize(DEVICE(&m->cpu), NULL, &error_fatal);

    memory_region_init_io(&m->ram_space, OBJECT(m), &open_bus_ops, m,
                          "maciifx.ram-space", IIFX_RAM_SPACE);
    memory_region_add_subregion(sysmem, 0, &m->ram_space);

    /* ROM, repeated through its window; also at 0 while the overlay is on */
    memory_region_init_rom(&m->rom, NULL, "maciifx.rom", IIFX_ROM_SIZE,
                           &error_fatal);
    for (i = 0; i < ARRAY_SIZE(m->rom_mirror); i++) {
        memory_region_init_alias(&m->rom_mirror[i], OBJECT(m),
                                 "maciifx.rom-mirror", &m->rom, 0,
                                 IIFX_ROM_SIZE);
        memory_region_add_subregion(sysmem, IIFX_ROM_ADDR + i * IIFX_ROM_SIZE,
                                    &m->rom_mirror[i]);
    }
    memory_region_init_alias(&m->overlay, OBJECT(m), "maciifx.overlay",
                             &m->rom, 0, IIFX_ROM_SIZE);
    memory_region_add_subregion_overlap(sysmem, 0, &m->overlay, 2);

    filename = qemu_find_file(QEMU_FILE_TYPE_BIOS, bios_name);
    size = filename ? load_image_mr(filename, &m->rom) : -1;
    g_free(filename);
    if (size != IIFX_ROM_SIZE && !qtest_enabled()) {
        error_report("could not load the Mac IIfx ROM '%s'", bios_name);
        exit(1);
    }

    /* I/O block, repeating every 256 KB */
    memory_region_init(&m->io, OBJECT(m), "maciifx.io", IIFX_IO_SLICE);
    for (i = 0; i < ARRAY_SIZE(m->io_alias); i++) {
        memory_region_init_alias(&m->io_alias[i], OBJECT(m),
                                 "maciifx.io-alias", &m->io, 0, IIFX_IO_SLICE);
        memory_region_add_subregion(sysmem, IIFX_IO_BASE + i * IIFX_IO_SLICE,
                                    &m->io_alias[i]);
    }

    /* OSS: interrupts, the 60 Hz tick, the reset overlay */
    object_initialize_child(OBJECT(machine), "oss", &m->oss, TYPE_MAC_OSS);
    oss = DEVICE(&m->oss);
    sysbus = SYS_BUS_DEVICE(oss);
    sysbus_realize(sysbus, &error_fatal);
    memory_region_add_subregion(&m->io, OSS_OFFSET,
                                sysbus_mmio_get_region(sysbus, 0));
    qdev_connect_gpio_out_named(oss, "ipl", 0,
                                qemu_allocate_irq(iifx_set_ipl, m, 0));
    qdev_connect_gpio_out_named(oss, "overlay", 0,
                                qemu_allocate_irq(iifx_set_overlay, m, 0));

    /* FMC memory controller */
    memory_region_init_io(&m->fmc, OBJECT(m), &iifx_fmc_ops, m,
                          "maciifx.fmc", 0x2000);
    memory_region_add_subregion(&m->io, FMC_OFFSET, &m->fmc);

    /* VIA 1: RTC/PRAM and the one-second interrupt */
    object_initialize_child(OBJECT(machine), "via1", &m->via1,
                            TYPE_MOS6522_Q800_VIA1);
    dev = DEVICE(&m->via1);
    qdev_prop_set_uint8(dev, "cpuid", 0x52);        /* PA6,4,2,1 = 1,1,0,1 */
    qdev_prop_set_uint8(dev, "cpuid-mask", 0x56);
    qdev_prop_set_uint8(dev, "porta-pullup", 0x81);
    qdev_prop_set_bit(dev, "adb", false);
    dinfo = drive_get(IF_MTD, 0, 0);
    if (dinfo) {
        qdev_prop_set_drive(dev, "drive", blk_by_legacy_dinfo(dinfo));
    }
    sysbus = SYS_BUS_DEVICE(dev);
    sysbus_realize(sysbus, &error_fatal);
    memory_region_add_subregion(&m->io, VIA1_OFFSET,
                                sysbus_mmio_get_region(sysbus, 1));
    sysbus_connect_irq(sysbus, 0, qdev_get_gpio_in(oss, OSS_SRC_VIA1));

    /* SCC IOP with the SCC behind it, used in bypass mode */
    object_initialize_child(OBJECT(machine), "scc-iop", &m->scc_iop,
                            TYPE_MAC_IOP);
    qdev_prop_set_uint8(DEVICE(&m->scc_iop), "kind", MAC_IOP_SCC);
    sysbus = SYS_BUS_DEVICE(&m->scc_iop);
    sysbus_realize(sysbus, &error_fatal);
    memory_region_add_subregion(&m->io, SCC_IOP_OFFSET,
                                sysbus_mmio_get_region(sysbus, 0));

    object_initialize_child(OBJECT(machine), "escc", &m->escc, TYPE_ESCC);
    dev = DEVICE(&m->escc);
    qdev_prop_set_uint32(dev, "disabled", 0);
    qdev_prop_set_uint32(dev, "frequency", MAC_CLOCK);
    qdev_prop_set_uint32(dev, "pclk", MAC_SCC_PCLK);
    qdev_prop_set_uint32(dev, "it_shift", 1);
    qdev_prop_set_bit(dev, "bit_swap", true);
    qdev_prop_set_chr(dev, "chrA", serial_hd(0));
    qdev_prop_set_chr(dev, "chrB", serial_hd(1));
    qdev_prop_set_uint32(dev, "chnBtype", 0);
    qdev_prop_set_uint32(dev, "chnAtype", 0);
    sysbus = SYS_BUS_DEVICE(dev);
    sysbus_realize(sysbus, &error_fatal);
    memory_region_add_subregion(&m->io, SCC_OFFSET,
                                sysbus_mmio_get_region(sysbus, 0));

    object_initialize_child(OBJECT(machine), "scc-orgate", &m->scc_orgate,
                            TYPE_OR_IRQ);
    object_property_set_int(OBJECT(&m->scc_orgate), "num-lines", 3,
                            &error_fatal);
    qdev_realize(DEVICE(&m->scc_orgate), NULL, &error_fatal);
    sysbus_connect_irq(sysbus, 0, qdev_get_gpio_in(DEVICE(&m->scc_orgate),
                                                   0));
    sysbus_connect_irq(sysbus, 1, qdev_get_gpio_in(DEVICE(&m->scc_orgate),
                                                   1));
    sysbus_connect_irq(SYS_BUS_DEVICE(&m->scc_iop), 0,
                       qdev_get_gpio_in(DEVICE(&m->scc_orgate), 2));
    qdev_connect_gpio_out(DEVICE(&m->scc_orgate), 0,
                          qdev_get_gpio_in(oss, OSS_SRC_SCC_IOP));

    /* SWIM IOP: ADB (and the floppy, not done yet) */
    object_initialize_child(OBJECT(machine), "swim-iop", &m->swim_iop,
                            TYPE_MAC_IOP);
    qdev_prop_set_uint8(DEVICE(&m->swim_iop), "kind", MAC_IOP_ISM);
    sysbus = SYS_BUS_DEVICE(&m->swim_iop);
    sysbus_realize(sysbus, &error_fatal);
    memory_region_add_subregion(&m->io, SWIM_IOP_OFFSET,
                                sysbus_mmio_get_region(sysbus, 0));
    sysbus_connect_irq(sysbus, 0, qdev_get_gpio_in(oss, OSS_SRC_SWIM_IOP));
    adb_bus = qdev_get_child_bus(DEVICE(&m->swim_iop), "adb.0");
    qdev_realize_and_unref(qdev_new(TYPE_ADB_KEYBOARD), adb_bus,
                           &error_fatal);
    qdev_realize_and_unref(qdev_new(TYPE_ADB_MOUSE), adb_bus, &error_fatal);

    /* Apple Sound Chip */
    object_initialize_child(OBJECT(machine), "asc", &m->asc, TYPE_ASC);
    qdev_prop_set_uint8(DEVICE(&m->asc), "asctype", ASC_TYPE_ASC);
    if (machine->audiodev) {
        qdev_prop_set_string(DEVICE(&m->asc), "audiodev", machine->audiodev);
    }
    sysbus = SYS_BUS_DEVICE(&m->asc);
    sysbus_realize(sysbus, &error_fatal);
    memory_region_add_subregion(&m->io, ASC_OFFSET,
                                sysbus_mmio_get_region(sysbus, 0));
    sysbus_connect_irq(sysbus, 0, qdev_get_gpio_in(oss, OSS_SRC_ASC));

    /* not emulated yet */
    iifx_unimp(m, "maciifx.scsi-dma", SCSI_OFFSET, 0x2000);

    /* NuBus slots $9-$E */
    object_initialize_child(OBJECT(machine), "mac-nubus-bridge",
                            &m->nubus_bridge, TYPE_MAC_NUBUS_BRIDGE);
    dev = DEVICE(&m->nubus_bridge);
    qdev_prop_set_uint32(dev, "slot-available-mask", 0x7e00);
    sysbus = SYS_BUS_DEVICE(dev);
    sysbus_realize(sysbus, &error_fatal);
    memory_region_add_subregion(sysmem,
                                MAC_NUBUS_FIRST_SLOT * NUBUS_SUPER_SLOT_SIZE,
                                sysbus_mmio_get_region(sysbus, 0));
    memory_region_add_subregion(sysmem,
                                NUBUS_SLOT_BASE +
                                MAC_NUBUS_FIRST_SLOT * NUBUS_SLOT_SIZE,
                                sysbus_mmio_get_region(sysbus, 1));
    for (i = 0; i < 6; i++) {
        qdev_connect_gpio_out(dev, 9 + i,
                              qdev_get_gpio_in(oss, OSS_SRC_SLOT9 + i));
    }
}

static void iifx_machine_class_init(ObjectClass *oc, const void *data)
{
    static const char * const valid_cpu_types[] = {
        M68K_CPU_TYPE_NAME("m68030"),
        NULL
    };
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "Macintosh IIfx";
    mc->init = iifx_machine_init;
    mc->reset = iifx_machine_reset;
    mc->default_cpu_type = M68K_CPU_TYPE_NAME("m68030");
    mc->valid_cpu_types = valid_cpu_types;
    mc->max_cpus = 1;
    mc->block_default_type = IF_SCSI;
    mc->default_ram_id = "maciifx.ram";
    mc->default_ram_size = 32 * MiB;
    machine_add_audiodev_property(mc);
}

static const TypeInfo iifx_machine_types[] = {
    {
        .name           = TYPE_IIFX_MACHINE,
        .parent         = TYPE_MACHINE,
        .instance_size  = sizeof(MacIIfxMachineState),
        .class_init     = iifx_machine_class_init,
    },
};

DEFINE_TYPES(iifx_machine_types)

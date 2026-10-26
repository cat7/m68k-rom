/*
 * Macintosh II
 *
 * 68020 + 68881, two VIAs, SCC, NCR 5380 SCSI, Apple Sound Chip, IWM floppy
 * and six NuBus slots ($9-$E); no built-in video. Without a 68851 the
 * "AMU" in the PMMU socket provides 24-bit addressing, switched by VIA2
 * PB3: in 24-bit mode only A0-A23 count and
 *
 *   $000000-$7FFFFF  RAM
 *   $800000-$8FFFFF  ROM ($40800000)
 *   $s00000-$sFFFFF  NuBus slot s = 9..E, the first MB of $Fs000000
 *   $F00000-$FFFFFF  I/O ($50F00000)
 *
 * At reset the ROM also appears at 0 (overlay) until VIA1 PA4 goes low.
 *
 * The Mac IIx (maciix) and IIcx (maciicx) are the same board with a
 * 68030, whose own PMMU does the 24-bit mapping (no AMU), and SIMMs of up
 * to 16 MB. The ROM tells them apart by VIA1 PA6 and by VIA2 PB3 read as
 * an input: II = 0/1, IIx = 0/0, IIcx = 1/1. The IIcx has three NuBus
 * slots ($9-$B).
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
#include "hw/core/qdev-properties.h"
#include "hw/core/sysbus.h"
#include "hw/char/escc.h"
#include "hw/misc/mac_via.h"
#include "hw/audio/asc.h"
#include "hw/block/swim2.h"
#include "hw/scsi/ncr5380.h"
#include "hw/nubus/mac-nubus-bridge.h"
#include "hw/input/adb.h"
#include "hw/core/or-irq.h"
#include "hw/nubus/nubus-video.h"
#include "target/m68k/cpu.h"
#include "qom/object.h"

#define MACII_ROM_ADDR      0x40000000
#define MACII_ROM_SIZE      (256 * KiB)
#define MACII_ROM_WINDOW    (16 * MiB)      /* ROM repeats through it */

#define MACII_IO_BASE       0x50000000
#define MACII_IO_SLICE      0x00020000      /* the I/O block repeats */
#define MACII_IO_SIZE       0x01000000

#define VIA1_OFFSET         0x00000
#define VIA2_OFFSET         0x02000
#define SCC_OFFSET          0x04000
#define SCSI_PDMA_OFFSET    0x06000
#define SCSI_OFFSET         0x10000
#define SCSI_HS_OFFSET      0x12000
#define ASC_OFFSET          0x14000
#define IWM_OFFSET          0x16000

#define MACII_RAM_SPACE     0x40000000      /* RAM decode: $0-$3FFFFFFF */

#define MAC_CLOCK           3686418
#define MAC_SCC_PCLK        7833600

/* VIA2 port A: SIMM size code; port B: 24/32-bit mode */
#define VIA2A_RAM_SHIFT     6
#define VIA2B_32BIT         3

enum {
    MACII_IRQ_VIA1,
    MACII_IRQ_VIA2,
    MACII_IRQ_SCC,
    MACII_IRQ_NB,
};

typedef struct MacIIModel {
    const char *desc;
    const char *cpu;
    uint8_t via1_pa6;           /* ID bit */
    uint8_t via2_pb3;           /* pulled up: reads 1 as an input */
    bool amu;                   /* 24-bit mapping by the AMU (68020) */
    uint16_t slots;             /* NuBus slot mask */
    uint32_t max_ram_mb;
} MacIIModel;

#define TYPE_MACII_MACHINE MACHINE_TYPE_NAME("macii")
OBJECT_DECLARE_TYPE(MacIIMachineState, MacIIMachineClass, MACII_MACHINE)

struct MacIIMachineClass {
    MachineClass parent_class;
    const MacIIModel *model;
};

struct MacIIMachineState {
    MachineState parent_obj;

    M68kCPU cpu;
    MemoryRegion rom;
    MemoryRegion rom_mirror[MACII_ROM_WINDOW / MACII_ROM_SIZE];
    MemoryRegion overlay;
    MemoryRegion ram_space;
    MemoryRegion open_bus;
    MemoryRegion ram_alias[8];
    const MacIIModel *model;
    MemoryRegion io;
    MemoryRegion io_alias[MACII_IO_SIZE / MACII_IO_SLICE];

    MOS6522Q800VIA1State via1;
    MOS6522Q800VIA2State via2;
    ESCCState escc;
    OrIRQState escc_orgate;
    ASCState asc;
    SWIM2State swim;
    NCR5380State *scsi;
    MemoryRegion scsi_regs;
    MemoryRegion scsi_pdma_alias;
    MacNubusBridge nubus_bridge;

    uint8_t ram_code;           /* VIA2 PA6-7 */
    bool mode32;                /* VIA2 PB3 */
    uint32_t irq_state;
};

/*
 * 24-bit mode: map the page at @addr (only A0-A23 decoded) to its 32-bit
 * physical address.
 */
static hwaddr macii_map24(void *opaque, vaddr addr)
{
    uint32_t a = addr & 0xffffff;

    if (a < 0x800000) {
        return a;
    }
    if (a < 0x900000) {
        return 0x40800000 | (a & 0xfffff);
    }
    if (a < 0xf00000) {
        return 0xf0000000 | ((a >> 20) << 24) | (a & 0xfffff);
    }
    return 0x50f00000 | (a & 0xfffff);
}

static void macii_set_mode32(void *opaque, int n, int level)
{
    MacIIMachineState *m = opaque;

    if (!m->model->amu) {
        return;
    }

    m->mode32 = level;
    m68k_cpu_set_addr_map(&m->cpu, level ? NULL : macii_map24, m);
}

static void macii_set_overlay(void *opaque, int n, int level)
{
    MacIIMachineState *m = opaque;

    memory_region_set_enabled(&m->overlay, level);
}

/* Bank size for the SIMM size code: 256 KB, 1 MB, 4 MB, 16 MB SIMMs */
static uint64_t macii_bank_size(uint8_t code)
{
    return (1 * MiB) << (2 * code);
}

/* The SIMM size code @size needs, with one or two full banks */
static int macii_ram_code_for(uint64_t size, bool *two_banks)
{
    int code;

    for (code = 0; code < 4; code++) {
        if (size == macii_bank_size(code) || size == 2 * macii_bank_size(code)) {
            *two_banks = size == 2 * macii_bank_size(code);
            return code;
        }
    }
    return -1;
}

/*
 * RAM decode. Bank A (4 SIMMs) starts at 0 and bank B follows it; VIA2
 * PA6-7 tell the GLUE the SIMM size so it can place bank B. The ROM sizes
 * memory by trying the codes and looking for mirroring, so with the wrong
 * code only the first megabyte is seen (repeated), and with the right one
 * all of it; past the end, two populated banks repeat and one bank reads
 * as open bus.
 */
static void macii_update_ram(MacIIMachineState *m)
{
    MachineState *machine = MACHINE(m);
    uint64_t size = machine->ram_size;
    bool two_banks;
    uint8_t want = macii_ram_code_for(size, &two_banks);
    uint64_t seg = (m->ram_code == want) ? size : 1 * MiB;
    int i;

    memory_region_transaction_begin();
    for (i = 0; i < ARRAY_SIZE(m->ram_alias); i++) {
        hwaddr base = i * seg;
        bool on = base < MAX(8 * MiB, 2 * size) &&
                  (i == 0 || m->ram_code != want || two_banks);

        if (memory_region_is_mapped(&m->ram_alias[i])) {
            memory_region_del_subregion(&m->ram_space, &m->ram_alias[i]);
            object_unparent(OBJECT(&m->ram_alias[i]));
        }
        if (!on) {
            continue;
        }
        memory_region_init_alias(&m->ram_alias[i], OBJECT(m), "macii.ram",
                                 machine->ram, 0, seg);
        memory_region_add_subregion(&m->ram_space, base, &m->ram_alias[i]);
    }
    memory_region_transaction_commit();
}

static void macii_set_ram_code(void *opaque, int n, int level)
{
    MacIIMachineState *m = opaque;
    uint8_t code = deposit32(m->ram_code, n, 1, level);

    if (code != m->ram_code) {
        m->ram_code = code;
        macii_update_ram(m);
    }
}

/* VIA1 level 1, VIA2 level 2, SCC level 4: the highest one wins */
static void macii_set_irq(void *opaque, int n, int level)
{
    static const uint8_t ipl[MACII_IRQ_NB] = { 1, 2, 4 };
    MacIIMachineState *m = opaque;
    int i, best = 0;

    m->irq_state = deposit32(m->irq_state, n, 1, level);
    for (i = 0; i < MACII_IRQ_NB; i++) {
        if ((m->irq_state & BIT(i)) && ipl[i] > best) {
            best = ipl[i];
        }
    }
    m68k_set_irq_level(&m->cpu, best, best ? 24 + best : 0);
}

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

static void macii_headsel(void *opaque, int n, int level)
{
    swim2_set_sel(opaque, level);
}

/* 5380 register window: register n at n * $10; A9 set = pseudo-DMA data */
static uint64_t macii_scsi_read(void *opaque, hwaddr addr, unsigned size)
{
    NCR5380State *s = opaque;
    uint64_t val = 0;
    unsigned i;

    if (addr & 0x200) {
        for (i = 0; i < size; i++) {
            val = (val << 8) | ncr5380_dma_read(s);
        }
        return val;
    }
    val = ncr5380_reg_read(s, (addr >> 4) & 7);
    for (i = 1; i < size; i++) {
        val |= val << 8;
    }
    return val;
}

static void macii_scsi_write(void *opaque, hwaddr addr, uint64_t val,
                             unsigned size)
{
    NCR5380State *s = opaque;
    int i;

    if (addr & 0x200) {
        for (i = size - 1; i >= 0; i--) {
            ncr5380_dma_write(s, val >> (8 * i));
        }
        return;
    }
    ncr5380_reg_write(s, (addr >> 4) & 7, val >> (8 * (size - 1)));
}

static const MemoryRegionOps macii_scsi_ops = {
    .read = macii_scsi_read,
    .write = macii_scsi_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static void macii_cpu_reset(void *opaque)
{
    MacIIMachineState *m = opaque;

    cpu_reset(CPU(&m->cpu));
    /* the reset vectors come through the overlay at 0 */
    m->cpu.env.aregs[7] = ldl_phys(CPU(&m->cpu)->as, 0);
    m->cpu.env.pc = ldl_phys(CPU(&m->cpu)->as, 4);
}

static void macii_machine_reset(MachineState *machine, ResetType type)
{
    MacIIMachineState *m = MACII_MACHINE(machine);

    qemu_devices_reset(type);
    m->ram_code = 0;
    macii_update_ram(m);
    macii_cpu_reset(m);
}


/* The RESET instruction cold-resets the NuBus cards, as on a real restart */
static void macii_reset_out(void *opaque, int n, int level)
{
    BusState *bus = qdev_get_child_bus(DEVICE(opaque), "nubus-bus.0");
    BusChild *kid;

    if (!level) {
        return;
    }
    QTAILQ_FOREACH(kid, &bus->children, sibling) {
        device_cold_reset(kid->child);
    }
}

static void macii_machine_init(MachineState *machine)
{
    MacIIMachineState *m = MACII_MACHINE(machine);
    const MacIIModel *model = MACII_MACHINE_GET_CLASS(m)->model;
    const char *bios_name = machine->firmware ?: "MacII.rom";
    bool two_banks;
    MemoryRegion *sysmem = get_system_memory();
    SysBusDevice *sysbus;
    DeviceState *dev;
    BusState *adb_bus;
    DriveInfo *dinfo;
    char *filename;
    int i, size;

    m->model = model;
    if (macii_ram_code_for(machine->ram_size, &two_banks) < 0 ||
        machine->ram_size > model->max_ram_mb * MiB) {
        error_report("The %s takes one or two banks of 1, 4, 16 or 64 MB "
                     "(1, 2, 4, 8, 16, 32, 64 or 128 MB), up to %u MB",
                     model->desc, model->max_ram_mb);
        exit(1);
    }

    object_initialize_child(OBJECT(machine), "cpu", &m->cpu,
                            machine->cpu_type);
    qdev_realize(DEVICE(&m->cpu), NULL, &error_fatal);

    /* RAM, inside its decode area */
    memory_region_init_io(&m->ram_space, OBJECT(m), &open_bus_ops, m,
                          "macii.ram-space", MACII_RAM_SPACE);
    memory_region_add_subregion(sysmem, 0, &m->ram_space);

    /* ROM, repeated through its window; also at 0 while the overlay is on */
    memory_region_init_rom(&m->rom, NULL, "macii.rom", MACII_ROM_SIZE,
                           &error_fatal);
    for (i = 0; i < ARRAY_SIZE(m->rom_mirror); i++) {
        memory_region_init_alias(&m->rom_mirror[i], OBJECT(m),
                                 "macii.rom-mirror", &m->rom, 0,
                                 MACII_ROM_SIZE);
        memory_region_add_subregion(sysmem,
                                    MACII_ROM_ADDR + i * MACII_ROM_SIZE,
                                    &m->rom_mirror[i]);
    }
    memory_region_init_alias(&m->overlay, OBJECT(m), "macii.overlay",
                             &m->rom, 0, MACII_ROM_SIZE);
    memory_region_add_subregion_overlap(sysmem, 0, &m->overlay, 1);

    filename = qemu_find_file(QEMU_FILE_TYPE_BIOS, bios_name);
    size = filename ? load_image_mr(filename, &m->rom) : -1;
    g_free(filename);
    if (size != MACII_ROM_SIZE && !qtest_enabled()) {
        error_report("could not load the Mac II ROM '%s'", bios_name);
        exit(1);
    }

    /* I/O block, repeating every 128 KB */
    memory_region_init(&m->io, OBJECT(m), "macii.io", MACII_IO_SLICE);
    for (i = 0; i < ARRAY_SIZE(m->io_alias); i++) {
        memory_region_init_alias(&m->io_alias[i], OBJECT(m), "macii.io-alias",
                                 &m->io, 0, MACII_IO_SLICE);
        memory_region_add_subregion(sysmem,
                                    MACII_IO_BASE + i * MACII_IO_SLICE,
                                    &m->io_alias[i]);
    }

    /* VIA 1: RTC/PRAM, ADB, 60 Hz and one-second interrupts */
    object_initialize_child(OBJECT(machine), "via1", &m->via1,
                            TYPE_MOS6522_Q800_VIA1);
    dev = DEVICE(&m->via1);
    qdev_prop_set_uint8(dev, "cpuid", model->via1_pa6 << 6);
    qdev_prop_set_uint8(dev, "cpuid-mask", 0x40);
    qdev_prop_set_uint8(dev, "porta-pullup", 0xbf);
    qdev_prop_set_bit(dev, "adb", true);
    dinfo = drive_get(IF_MTD, 0, 0);
    if (dinfo) {
        qdev_prop_set_drive(dev, "drive", blk_by_legacy_dinfo(dinfo));
    }
    sysbus = SYS_BUS_DEVICE(dev);
    sysbus_realize(sysbus, &error_fatal);
    memory_region_add_subregion(&m->io, VIA1_OFFSET,
                                sysbus_mmio_get_region(sysbus, 1));
    sysbus_connect_irq(sysbus, 0,
                       qemu_allocate_irq(macii_set_irq, m, MACII_IRQ_VIA1));
    qdev_connect_gpio_out_named(dev, "overlay", 0,
                                qemu_allocate_irq(macii_set_overlay, m, 0));

    adb_bus = qdev_get_child_bus(dev, "adb.0");
    qdev_realize_and_unref(qdev_new(TYPE_ADB_KEYBOARD), adb_bus,
                           &error_fatal);
    qdev_realize_and_unref(qdev_new(TYPE_ADB_MOUSE), adb_bus, &error_fatal);

    /* VIA 2: NuBus interrupts, RAM size, 24/32-bit mode, power */
    object_initialize_child(OBJECT(machine), "via2", &m->via2,
                            TYPE_MOS6522_Q800_VIA2);
    dev = DEVICE(&m->via2);
    qdev_prop_set_uint8(dev, "portb-pullup",
                        model->via2_pb3 ? 0xff : 0xf7);
    sysbus = SYS_BUS_DEVICE(dev);
    sysbus_realize(sysbus, &error_fatal);
    memory_region_add_subregion(&m->io, VIA2_OFFSET,
                                sysbus_mmio_get_region(sysbus, 1));
    sysbus_connect_irq(sysbus, 0,
                       qemu_allocate_irq(macii_set_irq, m, MACII_IRQ_VIA2));
    for (i = 0; i < 2; i++) {
        qdev_connect_gpio_out_named(dev, "port-a", VIA2A_RAM_SHIFT + i,
                                    qemu_allocate_irq(macii_set_ram_code,
                                                      m, i));
    }
    qdev_connect_gpio_out_named(dev, "port-b", VIA2B_32BIT,
                                qemu_allocate_irq(macii_set_mode32, m, 0));

    /* SCC */
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

    object_initialize_child(OBJECT(machine), "escc-orgate", &m->escc_orgate,
                            TYPE_OR_IRQ);
    object_property_set_int(OBJECT(&m->escc_orgate), "num-lines", 2,
                            &error_fatal);
    qdev_realize(DEVICE(&m->escc_orgate), NULL, &error_fatal);
    sysbus_connect_irq(sysbus, 0, qdev_get_gpio_in(DEVICE(&m->escc_orgate),
                                                   0));
    sysbus_connect_irq(sysbus, 1, qdev_get_gpio_in(DEVICE(&m->escc_orgate),
                                                   1));
    qdev_connect_gpio_out(DEVICE(&m->escc_orgate), 0,
                          qemu_allocate_irq(macii_set_irq, m, MACII_IRQ_SCC));

    /* Apple Sound Chip, interrupting on VIA2 CB1 */
    object_initialize_child(OBJECT(machine), "asc", &m->asc, TYPE_ASC);
    qdev_prop_set_uint8(DEVICE(&m->asc), "asctype", ASC_TYPE_ASC);
    if (machine->audiodev) {
        qdev_prop_set_string(DEVICE(&m->asc), "audiodev", machine->audiodev);
    }
    sysbus = SYS_BUS_DEVICE(&m->asc);
    sysbus_realize(sysbus, &error_fatal);
    memory_region_add_subregion(&m->io, ASC_OFFSET,
                                sysbus_mmio_get_region(sysbus, 0));
    sysbus_connect_irq(sysbus, 0, qdev_get_gpio_in(DEVICE(&m->via2),
                                                   VIA2_IRQ_ASC_BIT));

    /*
     * SCSI: NCR 5380 registers every 16 bytes, A9 selecting pseudo-DMA
     * data; the hardware-handshake window for blind transfers, also seen
     * at +$6000. IRQ to VIA2 CB2, DRQ to VIA2 CA2.
     */
    dev = qdev_new(TYPE_NCR5380);
    m->scsi = NCR5380(dev);
    qdev_prop_set_uint8(dev, "it_shift", 4);
    sysbus = SYS_BUS_DEVICE(dev);
    sysbus_realize_and_unref(sysbus, &error_fatal);
    memory_region_init_io(&m->scsi_regs, OBJECT(m), &macii_scsi_ops, m->scsi,
                          "macii.scsi", 0x2000);
    memory_region_add_subregion(&m->io, SCSI_OFFSET, &m->scsi_regs);
    memory_region_add_subregion(&m->io, SCSI_HS_OFFSET,
                                sysbus_mmio_get_region(sysbus,
                                                       NCR5380_MMIO_PDMA));
    memory_region_init_alias(&m->scsi_pdma_alias, OBJECT(m),
                             "macii.scsi-pdma",
                             sysbus_mmio_get_region(sysbus, NCR5380_MMIO_PDMA),
                             0, 0x2000);
    memory_region_add_subregion(&m->io, SCSI_PDMA_OFFSET,
                                &m->scsi_pdma_alias);
    qdev_connect_gpio_out(dev, NCR5380_GPIO_IRQ,
                       qemu_irq_invert(qdev_get_gpio_in(DEVICE(&m->via2),
                                                        VIA2_IRQ_SCSI_BIT)));
    qdev_connect_gpio_out(dev, NCR5380_GPIO_DRQ,
                       qemu_irq_invert(qdev_get_gpio_in(DEVICE(&m->via2),
                                                        VIA2_IRQ_SCSI_DATA_BIT)));
    scsi_bus_legacy_handle_cmdline(&m->scsi->bus);

    /*
     * Floppy: the Mac II has an IWM, which a SWIM in its power-on IWM mode
     * stands in for; head select comes from VIA1 PA5
     */
    object_initialize_child(OBJECT(machine), "swim", &m->swim, TYPE_SWIM2);
    qdev_prop_set_bit(DEVICE(&m->swim), "swim1", true);
    qdev_connect_gpio_out_named(DEVICE(&m->via1), "headsel", 0,
                                qemu_allocate_irq(macii_headsel, &m->swim, 0));
    sysbus = SYS_BUS_DEVICE(&m->swim);
    sysbus_realize(sysbus, &error_fatal);
    memory_region_add_subregion(&m->io, IWM_OFFSET,
                                sysbus_mmio_get_region(sysbus, 0));

    /* NuBus slots $9-$E */
    object_initialize_child(OBJECT(machine), "mac-nubus-bridge",
                            &m->nubus_bridge, TYPE_MAC_NUBUS_BRIDGE);
    dev = DEVICE(&m->nubus_bridge);
    qdev_prop_set_uint32(dev, "slot-available-mask", model->slots);
    sysbus = SYS_BUS_DEVICE(dev);
    sysbus_realize(sysbus, &error_fatal);
    memory_region_add_subregion(sysmem,
                                MAC_NUBUS_FIRST_SLOT * NUBUS_SUPER_SLOT_SIZE,
                                sysbus_mmio_get_region(sysbus, 0));
    memory_region_add_subregion(sysmem,
                                NUBUS_SLOT_BASE +
                                MAC_NUBUS_FIRST_SLOT * NUBUS_SLOT_SIZE,
                                sysbus_mmio_get_region(sysbus, 1));
    for (i = 0; i < VIA2_NUBUS_IRQ_NB; i++) {
        if (!(model->slots & BIT(9 + i))) {
            continue;
        }
        qdev_connect_gpio_out(dev, 9 + i,
                              qdev_get_gpio_in_named(DEVICE(&m->via2),
                                                     "nubus-irq",
                                                     VIA2_NUBUS_IRQ_9 + i));
    }


    qdev_connect_gpio_out_named(DEVICE(&m->cpu), "reset-out", 0,
                                qemu_allocate_irq(macii_reset_out,
                                                  &m->nubus_bridge, 0));

    /* our own video card in slot 9, unless -nodefaults */
    if (defaults_enabled()) {
        dev = qdev_new(TYPE_NUBUS_VIDEO);
        qdev_prop_set_uint32(dev, "slot", 9);
        if (graphic_width && graphic_height) {
            qdev_prop_set_uint32(dev, "width", graphic_width);
            qdev_prop_set_uint32(dev, "height", graphic_height);
        }
        qdev_realize_and_unref(dev, BUS(&m->nubus_bridge.parent_obj.bus),
                               &error_fatal);
    }

    m->mode32 = true;
}

static const MacIIModel macii_model = {
    .desc = "Macintosh II", .cpu = M68K_CPU_TYPE_NAME("m68020"),
    .via1_pa6 = 0, .via2_pb3 = 1, .amu = true, .slots = 0x7e00,
    .max_ram_mb = 8,
};

static const MacIIModel maciix_model = {
    .desc = "Macintosh IIx", .cpu = M68K_CPU_TYPE_NAME("m68030"),
    .via1_pa6 = 0, .via2_pb3 = 0, .amu = false, .slots = 0x7e00,
    .max_ram_mb = 128,
};

static const MacIIModel maciicx_model = {
    .desc = "Macintosh IIcx", .cpu = M68K_CPU_TYPE_NAME("m68030"),
    .via1_pa6 = 1, .via2_pb3 = 1, .amu = false, .slots = 0x0e00,
    .max_ram_mb = 128,
};

static void macii_machine_class_init(ObjectClass *oc, const void *data)
{
    static const char * const valid_cpu_types[] = {
        M68K_CPU_TYPE_NAME("m68020"),
        M68K_CPU_TYPE_NAME("m68030"),
        NULL
    };
    MachineClass *mc = MACHINE_CLASS(oc);
    MacIIMachineClass *mmc = MACII_MACHINE_CLASS(oc);
    const MacIIModel *model = data ? data : &macii_model;

    mmc->model = model;
    mc->desc = model->desc;
    mc->default_cpu_type = model->cpu;
    if (data) {
        return;                 /* a variant: the rest comes from macii */
    }
    mc->init = macii_machine_init;
    mc->reset = macii_machine_reset;
    mc->valid_cpu_types = valid_cpu_types;
    mc->max_cpus = 1;
    mc->block_default_type = IF_SCSI;
    mc->default_ram_id = "macii.ram";
    mc->default_ram_size = 8 * MiB;
    machine_add_audiodev_property(mc);
}

static const TypeInfo macii_machine_types[] = {
    {
        .name           = TYPE_MACII_MACHINE,
        .parent         = TYPE_MACHINE,
        .instance_size  = sizeof(MacIIMachineState),
        .class_size     = sizeof(MacIIMachineClass),
        .class_init     = macii_machine_class_init,
    }, {
        .name           = MACHINE_TYPE_NAME("maciix"),
        .parent         = TYPE_MACII_MACHINE,
        .class_init     = macii_machine_class_init,
        .class_data     = &maciix_model,
    }, {
        .name           = MACHINE_TYPE_NAME("maciicx"),
        .parent         = TYPE_MACII_MACHINE,
        .class_init     = macii_machine_class_init,
        .class_data     = &maciicx_model,
    },
};

DEFINE_TYPES(macii_machine_types)

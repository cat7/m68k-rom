/*
 * Macintosh Quadra 840AV
 *
 * A 40 MHz 68040 with Apple's "Cyclone" chip set:
 *
 *   YMCA   memory controller: RAM in eight 16 MB banks (four 72-pin
 *          SIMMs, 128 MB at most), the ROM, and the model's ID pins
 *   PSC    I/O controller: VIA1, a pseudo-VIA2, interrupt levels 3-6,
 *          DMA for SCSI, Ethernet, floppy and SCC, the sound engine and
 *          the DSP's interface
 *   Civic  built-in video, with the Sebastian CLUT/DAC
 *
 * plus an 85C30 SCC, a 53C96 SCSI controller, a MACE Ethernet controller,
 * the AT&T DSP3210, Cuda (ADB, clock, PRAM, power) and three NuBus slots
 * ($C-$E).
 *
 *   $00000000  RAM
 *   $40800000  ROM (2 MB, repeating from $40000000 to $4FFFFFFF)
 *   $50036000  Civic registers
 *   $50100000  VRAM (1 MB; 2 MB with the VRAM upgrade)
 *   $50F00000  VIA1                 $50F02000  pseudo-VIA2
 *   $50F04000  SCC                  $50F08000  Ethernet address PROM
 *   $50F18000  53C96 SCSI           $50F1C000  MACE
 *   $50F2A000  floppy controller    $50F2E000  pixel clock
 *   $50F30000  YMCA                 $50F30800  Sebastian
 *   $50F31000  PSC registers
 *   $5FFFFFFC  machine ID, $A55A2830 on both AV models
 *
 * The ROM (5BF10FD1) tells the 840AV from the 660AV by the YMCA's four ID
 * pins: $F is the 840AV.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/datadir.h"
#include "qemu/error-report.h"
#include "qemu/log.h"
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
#include "hw/misc/mac_psc.h"
#include "hw/display/mac_civic.h"
#include "hw/scsi/esp.h"
#include "hw/net/mac_mace.h"
#include "net/net.h"
#include "hw/nubus/mac-nubus-bridge.h"
#include "hw/input/adb.h"
#include "hw/core/or-irq.h"
#include "target/m68k/cpu.h"
#include "qom/object.h"
#include "system/calib-governor.h"
#include "mac-scsi-compat.h"

#define ROM_SIZE        (2 * MiB)
#define ROM_BASE        0x40800000
#define ROM_SPACE       0x40000000
#define ROM_WINDOW      (256 * MiB)

#define IO_BASE         0x50000000
#define IO_SIZE         (256 * MiB)

#define CIVIC_BASE      0x50036000
#define VRAM_BASE       0x50100000
#define VIA1_BASE       0x50f00000
#define VIA2_BASE       0x50f02000
#define SCC_BASE        0x50f04000
#define ENET_PROM_BASE  0x50f08000
#define SCSI_BASE       0x50f18000
#define MACE_BASE       0x50f1c000
#define FDC_BASE        0x50f2a000
#define CLOCK_BASE      0x50f2e000
#define YMCA_BASE       0x50f30000
#define DAC_BASE        0x50f30800
#define PSC_BASE        0x50f31000
#define MACHINE_ID      0x5ffffffc

#define MACHINE_ID_VAL  0xa55a2830
#define YMCA_ID_840AV   0xf

#define MAC_CLOCK       3686418
#define MAC_SCC_PCLK    7833600

#define MAX_RAM         (128 * MiB)
#define RAM_SPACE       0x40000000

#define TYPE_Q840AV_MACHINE MACHINE_TYPE_NAME("q840av")
OBJECT_DECLARE_SIMPLE_TYPE(Q840AVMachineState, Q840AV_MACHINE)

struct Q840AVMachineState {
    MachineState parent_obj;

    M68kCPU cpu;
    MemoryRegion rom;
    MemoryRegion rom_mirror[ROM_WINDOW / ROM_SIZE];
    MemoryRegion ram_space;
    MemoryRegion io_open_bus;
    MemoryRegion machine_id;
    MemoryRegion ymca;
    MemoryRegion fdc;
    MemoryRegion enet_prom;

    MOS6522Q800VIA1State via1;
    MacPSCState psc;
    MacCivicState civic;
    MacMACEState mace;
    ESCCState escc;
    SysBusESPState esp;
    MacNubusBridge nubus_bridge;

    uint32_t ymca_regs[0x800 / 4];
    uint8_t ymca_id;
};

static void q840av_set_ipl(void *opaque, int n, int level)
{
    Q840AVMachineState *m = opaque;

    m68k_set_irq_level(&m->cpu, level, level ? 24 + level : 0);
}

/* unused I/O reads 0 */
static uint64_t open_bus_read(void *opaque, hwaddr addr, unsigned size)
{
    qemu_log_mask(LOG_UNIMP, "q840av: read of unused I/O 0x%08" HWADDR_PRIx
                  "\n", IO_BASE + addr);
    return 0;
}

static void open_bus_write(void *opaque, hwaddr addr, uint64_t val,
                           unsigned size)
{
    qemu_log_mask(LOG_UNIMP, "q840av: write 0x%" PRIx64 " to unused I/O 0x%08"
                  HWADDR_PRIx "\n", val, IO_BASE + addr);
}

static const MemoryRegionOps open_bus_ops = {
    .read = open_bus_read,
    .write = open_bus_write,
    .endianness = DEVICE_BIG_ENDIAN,
};

static uint64_t ram_open_bus_read(void *opaque, hwaddr addr, unsigned size)
{
    return 0;
}

static void ram_open_bus_write(void *opaque, hwaddr addr, uint64_t val,
                               unsigned size)
{
}

static const MemoryRegionOps ram_open_bus_ops = {
    .read = ram_open_bus_read,
    .write = ram_open_bus_write,
    .endianness = DEVICE_BIG_ENDIAN,
};

static uint64_t machine_id_read(void *opaque, hwaddr addr, unsigned size)
{
    return extract32(MACHINE_ID_VAL, (4 - addr - size) * 8, size * 8);
}

static const MemoryRegionOps machine_id_ops = {
    .read = machine_id_read,
    .write = open_bus_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

/*
 * YMCA: RAM configuration registers (remembered), and the model ID pins
 * at +$438, +$43C, +$440, +$444 (bit 31 of each)
 */
static uint64_t ymca_read(void *opaque, hwaddr addr, unsigned size)
{
    Q840AVMachineState *m = opaque;
    uint32_t val;

    if (addr >= 0x438 && addr < 0x448) {
        val = (uint32_t)((m->ymca_id >> ((addr - 0x438) / 4)) & 1) << 31;
    } else {
        val = m->ymca_regs[addr / 4];
    }
    return extract32(val, (4 - (addr & 3) - size) * 8, size * 8);
}

static void ymca_write(void *opaque, hwaddr addr, uint64_t val,
                       unsigned size)
{
    Q840AVMachineState *m = opaque;
    unsigned shift = (4 - (addr & 3) - size) * 8;

    qemu_log_mask(LOG_UNIMP, "q840av: YMCA write 0x%" PRIx64 " to +0x%03"
                  HWADDR_PRIx "\n", val, addr);
    m->ymca_regs[addr / 4] = deposit32(m->ymca_regs[addr / 4], shift,
                                       size * 8, val);
}

static const MemoryRegionOps ymca_ops = {
    .read = ymca_read,
    .write = ymca_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

/* the floppy controller: not there yet; reads its idle status */
static uint64_t fdc_read(void *opaque, hwaddr addr, unsigned size)
{
    qemu_log_mask(LOG_UNIMP, "q840av: floppy read +0x%03" HWADDR_PRIx "\n",
                  addr);
    if ((addr & ~3) == 0x100) {
        return extract32(0x00800000, (4 - (addr & 3) - size) * 8, size * 8);
    }
    return 0;
}

static void fdc_write(void *opaque, hwaddr addr, uint64_t val,
                      unsigned size)
{
    qemu_log_mask(LOG_UNIMP, "q840av: floppy write 0x%" PRIx64 " to +0x%03"
                  HWADDR_PRIx "\n", val, addr);
}

static const MemoryRegionOps fdc_ops = {
    .read = fdc_read,
    .write = fdc_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static void q840av_cpu_reset(void *opaque)
{
    Q840AVMachineState *m = opaque;
    uint8_t *rom = memory_region_get_ram_ptr(&m->rom);

    cpu_reset(CPU(&m->cpu));
    m->cpu.env.aregs[7] = ldl_be_p(rom);
    m->cpu.env.pc = ROM_BASE + ldl_be_p(rom + 4);
}

/* The RESET instruction resets the NuBus cards, and recalibrates VIA1 */
static void q840av_reset_out(void *opaque, int n, int level)
{
    Q840AVMachineState *m = opaque;
    BusState *bus = qdev_get_child_bus(DEVICE(&m->nubus_bridge),
                                       "nubus-bus.0");
    BusChild *kid;

    if (!level) {
        return;
    }
    mac_via1_cpu_reset(&m->via1);
    QTAILQ_FOREACH(kid, &bus->children, sibling) {
        device_cold_reset(kid->child);
    }
}

static void q840av_init(MachineState *machine)
{
    Q840AVMachineState *m = Q840AV_MACHINE(machine);
    MemoryRegion *sysmem = get_system_memory();
    const char *bios_name = machine->firmware ?: "Quadra840AV.rom";
    SysBusDevice *sysbus;
    DeviceState *dev;
    BusState *adb_bus;
    DriveInfo *dinfo;
    ESPState *esp;
    char *filename;
    NICInfo *nd;
    MACAddr mac;
    uint8_t *prom, check;
    int i, size;

    calib_governor_default_on(12);

    /*
     * The ROM sizes RAM per 16 MB bank, and a bank holds 4, 8 or 16 MB:
     * full banks, then at most one bank of 4 or 8 MB
     */
    if (machine->ram_size < 8 * MiB || machine->ram_size > MAX_RAM ||
        machine->ram_size % (4 * MiB) ||
        machine->ram_size % (16 * MiB) == 12 * MiB) {
        error_report("The Quadra 840AV takes 8 to 128 MB of RAM: a multiple "
                     "of 16 MB, plus 4 or 8 MB (for example 8, 16, 20, 24, "
                     "32, 36, 40, 64 or 128 MB)");
        exit(1);
    }

    object_initialize_child(OBJECT(machine), "cpu", &m->cpu,
                            machine->cpu_type);
    qdev_realize(DEVICE(&m->cpu), NULL, &error_fatal);

    /* RAM; the rest of the RAM space reads 0 (empty banks) */
    memory_region_init_io(&m->ram_space, OBJECT(m), &ram_open_bus_ops, m,
                          "q840av.ram-space", RAM_SPACE);
    memory_region_add_subregion(sysmem, 0, &m->ram_space);
    memory_region_add_subregion(&m->ram_space, 0, machine->ram);

    /* ROM */
    memory_region_init_rom(&m->rom, NULL, "q840av.rom", ROM_SIZE,
                           &error_fatal);
    filename = qemu_find_file(QEMU_FILE_TYPE_BIOS, bios_name);
    size = filename ? load_image_mr(filename, &m->rom) : -1;
    g_free(filename);
    if (size != ROM_SIZE && !qtest_enabled()) {
        error_report("could not load the Quadra 840AV ROM '%s'", bios_name);
        exit(1);
    }
    for (i = 0; i < ARRAY_SIZE(m->rom_mirror); i++) {
        memory_region_init_alias(&m->rom_mirror[i], NULL, "q840av.rom-mirror",
                                 &m->rom, 0, ROM_SIZE);
        memory_region_add_subregion(sysmem, ROM_SPACE + i * ROM_SIZE,
                                    &m->rom_mirror[i]);
    }

    /* I/O: unused addresses read 0 */
    memory_region_init_io(&m->io_open_bus, OBJECT(m), &open_bus_ops, m,
                          "q840av.io-open-bus", IO_SIZE);
    memory_region_add_subregion_overlap(sysmem, IO_BASE, &m->io_open_bus, -1);

    memory_region_init_io(&m->machine_id, OBJECT(m), &machine_id_ops, m,
                          "q840av.machine-id", 4);
    memory_region_add_subregion(sysmem, MACHINE_ID, &m->machine_id);

    m->ymca_id = YMCA_ID_840AV;
    memory_region_init_io(&m->ymca, OBJECT(m), &ymca_ops, m, "q840av.ymca",
                          0x800);
    memory_region_add_subregion(sysmem, YMCA_BASE, &m->ymca);

    memory_region_init_io(&m->fdc, OBJECT(m), &fdc_ops, m, "q840av.fdc",
                          0x2000);
    memory_region_add_subregion(sysmem, FDC_BASE, &m->fdc);

    /* PSC */
    object_initialize_child(OBJECT(machine), "psc", &m->psc, TYPE_MAC_PSC);
    dev = DEVICE(&m->psc);
    if (machine->audiodev) {
        qdev_prop_set_string(dev, "audiodev", machine->audiodev);
    }
    sysbus = SYS_BUS_DEVICE(dev);
    sysbus_realize(sysbus, &error_fatal);
    memory_region_add_subregion(sysmem, VIA2_BASE,
                                sysbus_mmio_get_region(sysbus,
                                                       MAC_PSC_MMIO_VIA2));
    memory_region_add_subregion(sysmem, PSC_BASE,
                                sysbus_mmio_get_region(sysbus,
                                                       MAC_PSC_MMIO_REGS));
    qdev_connect_gpio_out_named(dev, "ipl", 0,
                                qemu_allocate_irq(q840av_set_ipl, m, 0));

    /* VIA1: Cuda (ADB, clock, PRAM, power), 60 Hz and one-second ticks */
    object_initialize_child(OBJECT(machine), "via1", &m->via1,
                            TYPE_MOS6522_Q800_VIA1);
    dev = DEVICE(&m->via1);
    qdev_prop_set_uint8(dev, "cpuid", 0);
    qdev_prop_set_uint8(dev, "cpuid-mask", 0);
    qdev_prop_set_uint8(dev, "porta-pullup", 0x80);
    qdev_prop_set_bit(dev, "cuda", true);
    qdev_prop_set_bit(dev, "adb", true);
    dinfo = drive_get(IF_MTD, 0, 0);
    if (dinfo) {
        qdev_prop_set_drive(dev, "drive", blk_by_legacy_dinfo(dinfo));
    }
    sysbus = SYS_BUS_DEVICE(dev);
    sysbus_realize(sysbus, &error_fatal);
    memory_region_add_subregion(sysmem, VIA1_BASE,
                                sysbus_mmio_get_region(sysbus, 1));
    sysbus_connect_irq(sysbus, 0, qdev_get_gpio_in(DEVICE(&m->psc),
                                                   MAC_PSC_IN_VIA1));
    adb_bus = qdev_get_child_bus(dev, "adb.0");
    qdev_realize_and_unref(qdev_new(TYPE_ADB_KEYBOARD), adb_bus,
                           &error_fatal);
    qdev_realize_and_unref(qdev_new(TYPE_ADB_MOUSE), adb_bus, &error_fatal);

    /* Civic video; VBL on the PSC's slot line 6 */
    object_initialize_child(OBJECT(machine), "civic", &m->civic,
                            TYPE_MAC_CIVIC);
    dev = DEVICE(&m->civic);
    if (graphic_width && graphic_height) {
        qdev_prop_set_uint32(dev, "width", graphic_width);
        qdev_prop_set_uint32(dev, "height", graphic_height);
    }
    sysbus = SYS_BUS_DEVICE(dev);
    sysbus_realize(sysbus, &error_fatal);
    memory_region_add_subregion(sysmem, CIVIC_BASE,
                                sysbus_mmio_get_region(sysbus,
                                                       MAC_CIVIC_MMIO_REGS));
    memory_region_add_subregion(sysmem, VRAM_BASE,
                                sysbus_mmio_get_region(sysbus,
                                                       MAC_CIVIC_MMIO_VRAM));
    memory_region_add_subregion(sysmem, CLOCK_BASE,
                                sysbus_mmio_get_region(sysbus,
                                                       MAC_CIVIC_MMIO_CLOCK));
    memory_region_add_subregion(sysmem, DAC_BASE,
                                sysbus_mmio_get_region(sysbus,
                                                       MAC_CIVIC_MMIO_DAC));
    qdev_connect_gpio_out_named(dev, "vbl", 0,
                                qdev_get_gpio_in_named(DEVICE(&m->psc),
                                                       "slot", 6));

    /* SCC, on PSC levels 4 (and 6) */
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
    memory_region_add_subregion(sysmem, SCC_BASE,
                                sysbus_mmio_get_region(sysbus, 0));
    sysbus_connect_irq(sysbus, 0, qdev_get_gpio_in(DEVICE(&m->psc),
                                                   MAC_PSC_IN_SCC_A));
    sysbus_connect_irq(sysbus, 1, qdev_get_gpio_in(DEVICE(&m->psc),
                                                   MAC_PSC_IN_SCC_B));

    /* SCSI: a 53C96, its data moved by the PSC's DMA */
    object_initialize_child(OBJECT(machine), "esp", &m->esp, TYPE_SYSBUS_ESP);
    esp = &m->esp.esp;
    esp->dma_memory_read = NULL;
    esp->dma_memory_write = NULL;
    esp->dma_opaque = NULL;
    m->esp.it_shift = 4;
    esp->dma_enabled = 1;
    sysbus = SYS_BUS_DEVICE(&m->esp);
    sysbus_realize(sysbus, &error_fatal);
    memory_region_add_subregion(sysmem, SCSI_BASE,
                                sysbus_mmio_get_region(sysbus, 0));
    m->psc.scsi_pdma = sysbus_mmio_get_region(sysbus, 1);
    sysbus_connect_irq(sysbus, 0, qdev_get_gpio_in(DEVICE(&m->psc),
                                                   MAC_PSC_IN_SCSI_IRQ));
    sysbus_connect_irq(sysbus, 1, qdev_get_gpio_in(DEVICE(&m->psc),
                                                   MAC_PSC_IN_SCSI_DRQ));
    scsi_bus_legacy_handle_cmdline(&esp->bus);

    /*
     * MACE Ethernet on PSC level 3, its frames moved by the PSC; the
     * address PROM holds the address bit-reversed, one byte every 16 (in
     * the odd byte lane), then a check byte: all eight XOR to $FF
     */
    object_initialize_child(OBJECT(machine), "mace", &m->mace, TYPE_MAC_MACE);
    dev = DEVICE(&m->mace);
    nd = qemu_find_nic_info(TYPE_MAC_MACE, true, "mace");
    if (nd) {
        qdev_set_nic_properties(dev, nd);
        memcpy(mac.a, nd->macaddr.a, sizeof(mac.a));
    } else {
        qemu_macaddr_default_if_unset(&mac);
    }
    mac.a[0] = 0x08;
    mac.a[1] = 0x00;
    mac.a[2] = 0x07;
    qdev_prop_set_macaddr(dev, "mac", mac.a);
    sysbus = SYS_BUS_DEVICE(dev);
    sysbus_realize(sysbus, &error_fatal);
    memory_region_add_subregion(sysmem, MACE_BASE,
                                sysbus_mmio_get_region(sysbus, 0));
    sysbus_connect_irq(sysbus, 0, qdev_get_gpio_in(DEVICE(&m->psc),
                                                   MAC_PSC_IN_ENET));
    m->mace.rx = mac_psc_enet_receive;
    m->mace.rx_opaque = &m->psc;
    m->psc.mace = &m->mace;

    memory_region_init_rom(&m->enet_prom, NULL, "q840av.enet-prom",
                           0x1000, &error_fatal);
    memory_region_add_subregion(sysmem, ENET_PROM_BASE, &m->enet_prom);
    prom = memory_region_get_ram_ptr(&m->enet_prom);
    memset(prom, 0, 0x1000);
    check = 0;
    for (i = 0; i < 6; i++) {
        prom[i * 16 + 1] = revbit8(mac.a[i]);
        check ^= prom[i * 16 + 1];
    }
    prom[7 * 16 + 1] = 0xff ^ check;

    /* NuBus slots $C-$E, on the PSC's slot lines 3-5 */
    object_initialize_child(OBJECT(machine), "mac-nubus-bridge",
                            &m->nubus_bridge, TYPE_MAC_NUBUS_BRIDGE);
    dev = DEVICE(&m->nubus_bridge);
    qdev_prop_set_uint32(dev, "slot-available-mask",
                         BIT(0xc) | BIT(0xd) | BIT(0xe));
    sysbus = SYS_BUS_DEVICE(dev);
    sysbus_realize(sysbus, &error_fatal);
    memory_region_add_subregion(sysmem,
                                MAC_NUBUS_FIRST_SLOT * NUBUS_SUPER_SLOT_SIZE,
                                sysbus_mmio_get_region(sysbus, 0));
    memory_region_add_subregion(sysmem,
                                NUBUS_SLOT_BASE +
                                MAC_NUBUS_FIRST_SLOT * NUBUS_SLOT_SIZE,
                                sysbus_mmio_get_region(sysbus, 1));
    for (i = 0; i < 3; i++) {
        qdev_connect_gpio_out(dev, 0xc + i,
                              qdev_get_gpio_in_named(DEVICE(&m->psc), "slot",
                                                     3 + i));
    }
    qdev_connect_gpio_out_named(DEVICE(&m->cpu), "reset-out", 0,
                                qemu_allocate_irq(q840av_reset_out, m, 0));
}

static void q840av_machine_reset(MachineState *machine, ResetType type)
{
    qemu_devices_reset(type);
    q840av_cpu_reset(Q840AV_MACHINE(machine));
}

static void q840av_class_init(ObjectClass *oc, const void *data)
{
    static const char * const valid_cpu_types[] = {
        M68K_CPU_TYPE_NAME("m68040"),
        NULL
    };
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "Macintosh Quadra 840AV";
    mc->init = q840av_init;
    mc->reset = q840av_machine_reset;
    mc->default_cpu_type = M68K_CPU_TYPE_NAME("m68040");
    mc->valid_cpu_types = valid_cpu_types;
    mc->max_cpus = 1;
    mc->block_default_type = IF_SCSI;
    mc->default_ram_id = "q840av.ram";
    mc->default_ram_size = 16 * MiB;
    mac_scsi_compat_add(mc);
    machine_add_audiodev_property(mc);
    calib_governor_add_machine_property(oc);
}

static const TypeInfo q840av_machine_types[] = {
    {
        .name           = TYPE_Q840AV_MACHINE,
        .parent         = TYPE_MACHINE,
        .instance_size  = sizeof(Q840AVMachineState),
        .class_init     = q840av_class_init,
    },
};

DEFINE_TYPES(q840av_machine_types)

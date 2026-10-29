/*
 * Macintosh LC, LC II and LC III
 *
 * Low-cost 68020/68030 Macs built around one Apple chip each, which holds
 * the memory controller, the built-in video, VIA1 and a "pseudo-VIA"
 * standing in for VIA2: V8 in the LC (16 MHz 68020) and LC II (16 MHz
 * 68030), Sonora in the LC III (25 MHz 68030). Next to it: an Egret for
 * ADB, the clock, PRAM and power, an SCC, an NCR 53C80 SCSI controller, a
 * SWIM, an Apple Sound Chip and one processor-direct slot ($E).
 *
 * Sonora (LC III):
 *
 *   $00000000  RAM: 4 MB on the board, then one 72-pin SIMM of 1, 2, 4,
 *              8, 16 or 32 MB (36 MB in all at most); see sonora_map_ram
 *   $40000000  ROM (1 MB, repeating to $4FFFFFFF)
 *   $50F00000  I/O, repeating every MB from $50000000 (offsets below)
 *   $5FFFFFFC  machine ID, $A55A0001 for the LC III
 *   $60000000  VRAM (512 KB; 768 KB with the VRAM SIMM), repeating every
 *              MB to $6FFFFFFF
 *   $FE000000  PDS slot $E
 *
 * V8 (LC, LC II) decodes only A31 and A0-A23, so its 16 MB repeat through
 * the lower 2 GB; the PDS slot answers at $FE000000 (in 24-bit mode at
 * $E00000, through the LC's HMMU or the LC II's PMMU tables):
 *
 *   $000000  RAM: a pair of 30-pin SIMMs (2, 4 or 8 MB) and the board's
 *            RAM (LC 2 MB, LC II 4 MB): 10 MB at most; see v8_map_ram
 *   $A00000  ROM (512 KB)
 *   $F00000  I/O (offsets below); VRAM at +$40000 (256 KB; 512 KB with
 *            the VRAM SIMM)
 *
 * The LC's 68020 has no MMU: Apple's HMMU between it and V8 drops A31 in
 * 24-bit mode, which pseudo-VIA port B bit 3 turns off.
 *
 * I/O:  +$00000 VIA1        +$04000 SCC
 *       +$06000 SCSI pseudo-DMA (handshake)
 *       +$10000 SCSI        +$12000 SCSI pseudo-DMA
 *       +$14000 ASC         +$16000 SWIM
 *       +$24000 video DAC   +$26000 pseudo-VIA
 *       +$28000 video control (Sonora)
 *
 * The LC III ROM (ECBBC41C) finds the model from the machine ID register;
 * the LC and LC II ROMs (350EACF0, 35C28F5F) from VIA1 port A bits 6, 4,
 * 2, 1 = 1, 1, 1, 0. At reset the CPU takes its vectors from the ROM.
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
#include "mac-scsi-compat.h"
#include "hw/core/loader.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/sysbus.h"
#include "hw/char/escc.h"
#include "hw/misc/mac_via.h"
#include "hw/misc/mac_pseudovia.h"
#include "hw/display/mac_lc_video.h"
#include "hw/audio/asc.h"
#include "hw/block/swim2.h"
#include "hw/scsi/ncr5380.h"
#include "hw/nubus/mac-nubus-bridge.h"
#include "hw/input/adb.h"
#include "hw/core/or-irq.h"
#include "target/m68k/cpu.h"
#include "qom/object.h"
#include "system/calib-governor.h"

/* Sonora */
#define SONORA_ROM_ADDR     0x40000000
#define SONORA_ROM_WINDOW   (256 * MiB)
#define SONORA_ROM_BASE     0x40800000      /* where the ROM runs */
#define SONORA_IO_BASE      0x50000000
#define SONORA_IO_SIZE      (16 * MiB)
#define SONORA_MACHINE_ID   0x5ffffffc
#define SONORA_VRAM_ADDR    0x60000000
#define SONORA_VRAM_WINDOW  (1 * MiB)       /* VRAM, then nothing */
#define SONORA_VRAM_SPACE   (256 * MiB)     /* repeating to $6FFFFFFF */
#define SONORA_RAM_SPACE    0x40000000
#define SONORA_RAM_BANK     (16 * MiB)      /* the ROM sizes RAM per bank */

/* V8, inside its 16 MB */
#define V8_SPACE            (16 * MiB)
#define V8_RAM_SPACE        0xa00000
#define V8_ROM_ADDR         0xa00000
#define V8_ROM_COPIES       2               /* 512 KB, to $AFFFFF */
#define V8_ROM_BASE         0x40a00000      /* where the ROM runs */
#define V8_IO_ADDR          0xf00000
#define V8_VRAM_OFFSET      0x40000
#define V8_VRAM_WINDOW      (512 * KiB)
#define V8_BOARD_LOW        0x800000        /* first 2 MB of the board RAM */

/* I/O, within its MB */
#define LC_IO_SLICE         (1 * MiB)
#define VIA1_OFFSET         0x00000
#define SCC_OFFSET          0x04000
#define SCSI_HS_ALIAS       0x06000
#define SCSI_OFFSET         0x10000
#define SCSI_HS_OFFSET      0x12000
#define ASC_OFFSET          0x14000
#define SWIM_OFFSET         0x16000
#define DAC_OFFSET          0x24000
#define PVIA_OFFSET         0x26000
#define VCTRL_OFFSET        0x28000

#define MAC_CLOCK           3686418
#define MAC_SCC_PCLK        7833600

/* pseudo-VIA port B: 1 turns the LC's HMMU off (32-bit mode) */
#define V8_PB_HMMU_OFF      3

enum {
    LC_IRQ_VIA1,
    LC_IRQ_VIA2,
    LC_IRQ_SCC,
    LC_IRQ_NB,
};

typedef struct MacLCModel {
    const char *desc;
    const char *cpu;
    const char *rom_filename;
    uint32_t rom_size;
    bool v8;                    /* V8, else Sonora */
    bool hmmu;                  /* the LC's 24-bit HMMU */
    uint32_t machine_id;        /* Sonora: at $5FFFFFFC */
    uint32_t onboard_mb;        /* RAM soldered to the board */
    uint32_t default_ram_mb;
    uint32_t calib_mips;        /* calibration-governor pacing rate */
} MacLCModel;

#define TYPE_MACLC_MACHINE MACHINE_TYPE_NAME("mac-lc")
OBJECT_DECLARE_TYPE(MacLCMachineState, MacLCMachineClass, MACLC_MACHINE)

struct MacLCMachineClass {
    MachineClass parent_class;
    const MacLCModel *model;
};

struct MacLCMachineState {
    MachineState parent_obj;

    M68kCPU cpu;
    const MacLCModel *model;
    MemoryRegion rom;
    MemoryRegion rom_mirror[SONORA_ROM_WINDOW / MiB];
    MemoryRegion ram_space;
    MemoryRegion ram_alias[3];          /* board, SIMM, V8 board low */
    MemoryRegion board;                 /* the board's RAM */
    MemoryRegion board_used;            /* the part -m pays for */
    MemoryRegion board_spare;           /* LC II at 10 MB: the rest */
    MemoryRegion io;
    MemoryRegion io_open_bus;
    MemoryRegion io_alias[SONORA_IO_SIZE / LC_IO_SLICE];
    MemoryRegion machine_id;
    MemoryRegion vram_window;
    MemoryRegion vram_open_bus;
    MemoryRegion vram_alias[SONORA_VRAM_SPACE / SONORA_VRAM_WINDOW];
    MemoryRegion v8_space;
    MemoryRegion v8_alias[0x80000000ULL / V8_SPACE];

    MOS6522Q800VIA1State via1;
    MacPseudoVIAState pvia;
    MacLCVideoState video;
    ESCCState escc;
    OrIRQState escc_orgate;
    ASCState asc;
    SWIM2State swim;
    NCR5380State *scsi;
    MemoryRegion scsi_regs;
    MemoryRegion scsi_hs_alias;
    MacNubusBridge nubus_bridge;

    uint32_t irq_state;
    uint8_t ram_config;         /* pseudo-VIA register 1 */
    uint64_t board_size;        /* board RAM counted in -m */
};

static void maclc_map_ram(MacLCMachineState *m);

/* VIA1 level 1, pseudo-VIA level 2, SCC level 4: the highest one wins */
static void maclc_set_irq(void *opaque, int n, int level)
{
    static const uint8_t ipl[LC_IRQ_NB] = { 1, 2, 4 };
    MacLCMachineState *m = opaque;
    int i, best = 0;

    m->irq_state = deposit32(m->irq_state, n, 1, level);
    for (i = 0; i < LC_IRQ_NB; i++) {
        if ((m->irq_state & BIT(i)) && ipl[i] > best) {
            best = ipl[i];
        }
    }
    m68k_set_irq_level(&m->cpu, best, best ? 24 + best : 0);
}

/* reads past the installed RAM, and of unused I/O, return 0 */
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

static uint64_t machine_id_read(void *opaque, hwaddr addr, unsigned size)
{
    MacLCMachineState *m = opaque;

    return extract32(m->model->machine_id, (4 - addr - size) * 8, size * 8);
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

static void maclc_headsel(void *opaque, int n, int level)
{
    swim2_set_sel(opaque, level);
}

/* 5380 register window: register n at n * $10; A9 set = pseudo-DMA data */
static uint64_t maclc_scsi_read(void *opaque, hwaddr addr, unsigned size)
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

static void maclc_scsi_write(void *opaque, hwaddr addr, uint64_t val,
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

static const MemoryRegionOps maclc_scsi_ops = {
    .read = maclc_scsi_read,
    .write = maclc_scsi_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

/*
 * The LC's HMMU in 24-bit mode: only A0-A23 count; $E00000-$EFFFFF is the
 * PDS slot (the first MB of $FE000000), the rest goes to V8
 */
static hwaddr v8_hmmu_map24(void *opaque, vaddr addr)
{
    uint32_t a = addr & 0xffffff;

    if ((a >> 20) == 0xe) {
        return 0xfe000000 | (a & 0xfffff);
    }
    return a;
}

static void v8_set_hmmu_off(void *opaque, int n, int level)
{
    MacLCMachineState *m = opaque;

    m68k_cpu_set_addr_map(&m->cpu, level ? NULL : v8_hmmu_map24, m);
}

static void maclc_cpu_reset(void *opaque)
{
    MacLCMachineState *m = opaque;
    /*
     * The ROM image is copied in at reset; its reset PC is an offset into
     * the ROM, which also sits at 0
     */
    uint8_t *rom = memory_region_get_ram_ptr(&m->rom);

    cpu_reset(CPU(&m->cpu));
    m->cpu.env.aregs[7] = ldl_be_p(rom);
    m->cpu.env.pc = (m->model->v8 ? V8_ROM_BASE : SONORA_ROM_BASE) +
                    ldl_be_p(rom + 4);
}

static void maclc_machine_reset(MachineState *machine, ResetType type)
{
    MacLCMachineState *m = MACLC_MACHINE(machine);

    qemu_devices_reset(type);
    /* V8 comes up with an 8 MB SIMM's layout until the ROM sizes RAM */
    m->ram_config = m->model->v8 ? 0xc0 : 0;
    maclc_map_ram(m);
    if (m->model->hmmu) {
        v8_set_hmmu_off(m, 0, 0);
    }
    maclc_cpu_reset(m);
}

/* The RESET instruction cold-resets the PDS card, as on a real restart */
static void maclc_reset_out(void *opaque, int n, int level)
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

/* RAM */

static uint64_t maclc_simm_size(MacLCMachineState *m)
{
    return MACHINE(m)->ram_size - m->board_size;
}

/* (Re)place RAM piece @i; where pieces overlap, the board's RAM wins */
static void maclc_place(MacLCMachineState *m, int i, bool on, hwaddr base)
{
    static const int priority[3] = { 2, 1, 0 };

    if (memory_region_is_mapped(&m->ram_alias[i])) {
        memory_region_del_subregion(&m->ram_space, &m->ram_alias[i]);
    }
    if (on) {
        memory_region_add_subregion_overlap(&m->ram_space, base,
                                            &m->ram_alias[i], priority[i]);
    }
}

/*
 * Sonora: the board's 4 MB at 0, and one SIMM. The ROM sizes RAM in 16 MB
 * banks, each from where it stops; for that it sets the configuration
 * register (pseudo-VIA register 1, bits 0-2) to 7 and the SIMM sits in its
 * own bank at $01000000 (two banks for 32 MB). It then writes the SIMM's
 * size code (1 MB = 1 ... 32 MB = 6), and the SIMM moves to just after
 * the board's RAM, so memory is one stretch.
 */
static void sonora_map_ram(MacLCMachineState *m)
{
    uint8_t code = m->ram_config & 7;
    bool joined = code >= 1 && code <= 6;

    maclc_place(m, 0, true, 0);
    maclc_place(m, 1, maclc_simm_size(m),
                joined ? m->board_size : SONORA_RAM_BANK);
}

/*
 * V8: the SIMMs, if the configuration (pseudo-VIA register 1) enables
 * them, at 0; the board's RAM after the size it gives for them (bits 6-7:
 * none, 2, 4 or 8 MB; with 8 MB the board's RAM is not placed), cut to
 * 2 MB if bit 5 says so. The first 2 MB of the board's RAM always also
 * appear at $800000. An LC II with 8 MB of SIMMs uses 2 MB of its board.
 */
static void v8_map_ram(MacLCMachineState *m)
{
    static const uint32_t simm_mb[4] = { 0, 2, 4, 8 };
    uint8_t code = m->ram_config >> 6;
    bool board_on = code != 3;

    maclc_place(m, 1, maclc_simm_size(m) && code, 0);
    maclc_place(m, 2, true, V8_BOARD_LOW);
    maclc_place(m, 0, false, 0);
    if (board_on) {
        memory_region_set_size(&m->ram_alias[0], (m->ram_config & 0x20) ?
                                                 2 * MiB :
                                                 m->model->onboard_mb * MiB);
        maclc_place(m, 0, true, simm_mb[code] * MiB);
    }
}

static void maclc_map_ram(MacLCMachineState *m)
{
    memory_region_transaction_begin();
    if (m->model->v8) {
        v8_map_ram(m);
    } else {
        sonora_map_ram(m);
    }
    memory_region_transaction_commit();
}

static void maclc_set_ram_config(void *opaque, int n, int level)
{
    MacLCMachineState *m = opaque;
    uint8_t config = deposit32(m->ram_config, n, 1, level);

    if (config != m->ram_config) {
        m->ram_config = config;
        maclc_map_ram(m);
    }
}

/*
 * The board's RAM is all there even when the configuration uses only part
 * of it (an LC II with 8 MB of SIMMs uses 2 of its 4 MB): the ROM sizes
 * it whole first. -m counts the part in use; the rest is extra.
 */
static void maclc_init_ram(MacLCMachineState *m, MachineState *machine)
{
    uint64_t board = m->model->onboard_mb * MiB;
    uint64_t simm = maclc_simm_size(m);

    memory_region_init(&m->board, OBJECT(m), "maclc.board", board);
    memory_region_init_alias(&m->board_used, OBJECT(m), "maclc.board-used",
                             machine->ram, 0, m->board_size);
    memory_region_add_subregion(&m->board, 0, &m->board_used);
    if (m->board_size < board) {
        memory_region_init_ram(&m->board_spare, NULL,
                               "maclc.board-spare", board - m->board_size,
                               &error_fatal);
        memory_region_add_subregion(&m->board, m->board_size,
                                    &m->board_spare);
    }
    memory_region_init_alias(&m->ram_alias[0], OBJECT(m), "maclc.ram",
                             &m->board, 0, board);
    memory_region_init_alias(&m->ram_alias[1], OBJECT(m), "maclc.simm",
                             machine->ram, m->board_size, simm ?: 1);
    memory_region_init_alias(&m->ram_alias[2], OBJECT(m), "maclc.ram-low",
                             &m->board, 0, 2 * MiB);
}

/* The RAM each model takes; returns an error message, or NULL */
static char *maclc_check_ram(MacLCMachineState *m, uint64_t size)
{
    const MacLCModel *model = m->model;
    uint64_t board = model->onboard_mb * MiB;
    uint64_t simm;

    if (!model->v8) {
        simm = size - board;
        m->board_size = board;
        if (size >= board && (simm == 0 ||
                              (is_power_of_2(simm) && simm >= 1 * MiB &&
                               simm <= 32 * MiB))) {
            return NULL;
        }
        return g_strdup_printf("The %s has %u MB on the board and takes one "
                               "SIMM of 1, 2, 4, 8, 16 or 32 MB: -m must be "
                               "%u, %u, %u, %u, %u, %u or %u MB",
                               model->desc, model->onboard_mb,
                               model->onboard_mb, model->onboard_mb + 1,
                               model->onboard_mb + 2, model->onboard_mb + 4,
                               model->onboard_mb + 8, model->onboard_mb + 16,
                               model->onboard_mb + 32);
    }

    /*
     * V8: SIMM pairs of 2, 4 or 8 MB, 10 MB at most. An LC II with 8 MB
     * of SIMMs (10 MB) uses 2 of its 4 MB, which the ROM does not size
     * right here yet, so it stops at 8 MB.
     */
    m->board_size = size == 10 * MiB ? 2 * MiB : board;
    simm = size - m->board_size;
    if (size >= board && size <= 10 * MiB &&
        !(board > 2 * MiB && size == 10 * MiB) &&
        (simm == 0 || simm == 2 * MiB || simm == 4 * MiB ||
         simm == 8 * MiB)) {
        return NULL;
    }
    return g_strdup_printf("The %s has %u MB on the board and takes a pair "
                           "of SIMMs (2, 4 or 8 MB): -m must be %s",
                           model->desc, model->onboard_mb,
                           model->onboard_mb == 2 ? "2, 4, 6 or 10 MB" :
                                                    "4, 6 or 8 MB");
}

/* The memory map */

static void sonora_init_map(MacLCMachineState *m, MemoryRegion *sysmem)
{
    int i;

    /* RAM from 0, inside its decode area */
    memory_region_init_io(&m->ram_space, OBJECT(m), &open_bus_ops, m,
                          "maclc.ram-space", SONORA_RAM_SPACE);
    memory_region_add_subregion(sysmem, 0, &m->ram_space);

    /* ROM, repeated through its window */
    for (i = 0; i < SONORA_ROM_WINDOW / m->model->rom_size &&
                i < ARRAY_SIZE(m->rom_mirror); i++) {
        memory_region_init_alias(&m->rom_mirror[i], OBJECT(m),
                                 "maclc.rom-mirror", &m->rom, 0,
                                 m->model->rom_size);
        memory_region_add_subregion(sysmem,
                                    SONORA_ROM_ADDR + i * m->model->rom_size,
                                    &m->rom_mirror[i]);
    }

    /* I/O, repeating every MB */
    for (i = 0; i < ARRAY_SIZE(m->io_alias); i++) {
        memory_region_init_alias(&m->io_alias[i], OBJECT(m), "maclc.io-alias",
                                 &m->io, 0, LC_IO_SLICE);
        memory_region_add_subregion(sysmem, SONORA_IO_BASE + i * LC_IO_SLICE,
                                    &m->io_alias[i]);
    }

    memory_region_init_io(&m->machine_id, OBJECT(m), &machine_id_ops, m,
                          "maclc.machine-id", 4);
    memory_region_add_subregion(sysmem, SONORA_MACHINE_ID, &m->machine_id);

    /*
     * VRAM sits at the start of each MB, the rest reads 0: the ROM sizes
     * it by writing past 512 KB
     */
    for (i = 0; i < ARRAY_SIZE(m->vram_alias); i++) {
        memory_region_init_alias(&m->vram_alias[i], OBJECT(m),
                                 "maclc.vram-alias", &m->vram_window, 0,
                                 SONORA_VRAM_WINDOW);
        memory_region_add_subregion(sysmem,
                                    SONORA_VRAM_ADDR + i * SONORA_VRAM_WINDOW,
                                    &m->vram_alias[i]);
    }
}

static void v8_init_map(MacLCMachineState *m, MemoryRegion *sysmem)
{
    int i;

    memory_region_init(&m->v8_space, OBJECT(m), "maclc.v8", V8_SPACE);
    memory_region_init_io(&m->ram_space, OBJECT(m), &open_bus_ops, m,
                          "maclc.ram-space", V8_RAM_SPACE);
    memory_region_add_subregion(&m->v8_space, 0, &m->ram_space);
    for (i = 0; i < V8_ROM_COPIES; i++) {
        memory_region_init_alias(&m->rom_mirror[i], OBJECT(m),
                                 "maclc.rom-mirror", &m->rom, 0,
                                 m->model->rom_size);
        memory_region_add_subregion(&m->v8_space,
                                    V8_ROM_ADDR + i * m->model->rom_size,
                                    &m->rom_mirror[i]);
    }
    memory_region_add_subregion(&m->v8_space, V8_IO_ADDR, &m->io);
    memory_region_add_subregion(&m->io, V8_VRAM_OFFSET, &m->vram_window);

    /* only A0-A23 count below $80000000 */
    for (i = 0; i < ARRAY_SIZE(m->v8_alias); i++) {
        memory_region_init_alias(&m->v8_alias[i], OBJECT(m), "maclc.v8-alias",
                                 &m->v8_space, 0, V8_SPACE);
        memory_region_add_subregion(sysmem, (hwaddr)i * V8_SPACE,
                                    &m->v8_alias[i]);
    }
}

static void maclc_machine_init(MachineState *machine)
{
    MacLCMachineState *m = MACLC_MACHINE(machine);
    const MacLCModel *model = MACLC_MACHINE_GET_CLASS(m)->model;
    const char *bios_name = machine->firmware ?: model->rom_filename;
    MemoryRegion *sysmem = get_system_memory();
    g_autofree char *ram_error = NULL;
    SysBusDevice *sysbus;
    DeviceState *dev;
    BusState *adb_bus;
    DriveInfo *dinfo;
    char *filename;
    int i, size;

    calib_governor_default_on(model->calib_mips);

    m->model = model;
    ram_error = maclc_check_ram(m, machine->ram_size);
    if (ram_error) {
        error_report("%s", ram_error);
        exit(1);
    }

    object_initialize_child(OBJECT(machine), "cpu", &m->cpu,
                            machine->cpu_type);
    qdev_realize(DEVICE(&m->cpu), NULL, &error_fatal);

    memory_region_init_rom(&m->rom, NULL, "maclc.rom", model->rom_size,
                           &error_fatal);
    filename = qemu_find_file(QEMU_FILE_TYPE_BIOS, bios_name);
    size = filename ? load_image_mr(filename, &m->rom) : -1;
    g_free(filename);
    if (size != model->rom_size && !qtest_enabled()) {
        error_report("could not load the %s ROM '%s'", model->desc,
                     bios_name);
        exit(1);
    }

    /* the I/O block; unused addresses read 0 */
    memory_region_init(&m->io, OBJECT(m), "maclc.io", LC_IO_SLICE);
    memory_region_init_io(&m->io_open_bus, OBJECT(m), &open_bus_ops, m,
                          "maclc.io-open-bus", LC_IO_SLICE);
    memory_region_add_subregion_overlap(&m->io, 0, &m->io_open_bus, -1);

    /* the built-in video's VRAM, then nothing */
    object_initialize_child(OBJECT(machine), "video", &m->video,
                            TYPE_MAC_LC_VIDEO);
    dev = DEVICE(&m->video);
    qdev_prop_set_bit(dev, "v8", model->v8);
    if (model->v8) {
        qdev_prop_set_uint32(dev, "vram-size", 256 * KiB);
    }
    if (graphic_width && graphic_height) {
        qdev_prop_set_uint32(dev, "width", graphic_width);
        qdev_prop_set_uint32(dev, "height", graphic_height);
    }
    sysbus = SYS_BUS_DEVICE(dev);
    sysbus_realize(sysbus, &error_fatal);
    size = model->v8 ? V8_VRAM_WINDOW : SONORA_VRAM_WINDOW;
    memory_region_init(&m->vram_window, OBJECT(m), "maclc.vram-window", size);
    memory_region_init_io(&m->vram_open_bus, OBJECT(m), &open_bus_ops, m,
                          "maclc.vram-open-bus", size);
    memory_region_add_subregion_overlap(&m->vram_window, 0, &m->vram_open_bus,
                                        -1);
    memory_region_add_subregion(&m->vram_window, 0,
                                sysbus_mmio_get_region(sysbus,
                                    MAC_LC_VIDEO_MMIO_VRAM));
    memory_region_add_subregion(&m->io, DAC_OFFSET,
                                sysbus_mmio_get_region(sysbus,
                                    MAC_LC_VIDEO_MMIO_DAC));
    if (!model->v8) {
        memory_region_add_subregion(&m->io, VCTRL_OFFSET,
                                    sysbus_mmio_get_region(sysbus,
                                        MAC_LC_VIDEO_MMIO_CTRL));
    }

    if (model->v8) {
        v8_init_map(m, sysmem);
    } else {
        sonora_init_map(m, sysmem);
    }
    maclc_init_ram(m, machine);

    /* VIA 1: Egret (ADB, clock, PRAM, power), 60 Hz and one-second ticks */
    object_initialize_child(OBJECT(machine), "via1", &m->via1,
                            TYPE_MOS6522_Q800_VIA1);
    dev = DEVICE(&m->via1);
    if (model->v8) {
        /* the LC's ID; PA0 high: no diagnostic mode */
        qdev_prop_set_uint8(dev, "cpuid", 0x54);
        qdev_prop_set_uint8(dev, "cpuid-mask", 0x56);
        qdev_prop_set_uint8(dev, "porta-pullup", 0x81);
    } else {
        qdev_prop_set_uint8(dev, "cpuid", 0);
        qdev_prop_set_uint8(dev, "cpuid-mask", 0);
        qdev_prop_set_uint8(dev, "porta-pullup", 0x80);
    }
    qdev_prop_set_bit(dev, "egret", true);
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
                       qemu_allocate_irq(maclc_set_irq, m, LC_IRQ_VIA1));

    adb_bus = qdev_get_child_bus(dev, "adb.0");
    qdev_realize_and_unref(qdev_new(TYPE_ADB_KEYBOARD), adb_bus,
                           &error_fatal);
    qdev_realize_and_unref(qdev_new(TYPE_ADB_MOUSE), adb_bus, &error_fatal);

    /* the pseudo-VIA: slot, SCSI and sound interrupts, RAM configuration */
    object_initialize_child(OBJECT(machine), "pseudovia", &m->pvia,
                            TYPE_MAC_PSEUDOVIA);
    dev = DEVICE(&m->pvia);
    qdev_prop_set_uint8(dev, "variant", model->v8 ? MAC_PSEUDOVIA_V8 :
                                                    MAC_PSEUDOVIA_SONORA);
    if (model->v8) {
        qdev_prop_set_uint8(dev, "video-in",
                            mac_lc_video_v8_monitor_code(&m->video) << 3);
        /* VRAM present; bit 1: only 256 KB of it */
        qdev_prop_set_uint8(dev, "config-in",
                            m->video.vram_size == 256 * KiB ? 0x06 : 0x04);
    }
    sysbus = SYS_BUS_DEVICE(dev);
    sysbus_realize(sysbus, &error_fatal);
    memory_region_add_subregion(&m->io, PVIA_OFFSET,
                                sysbus_mmio_get_region(sysbus, 0));
    sysbus_connect_irq(sysbus, 0,
                       qemu_allocate_irq(maclc_set_irq, m, LC_IRQ_VIA2));
    for (i = 0; i < 8; i++) {
        qdev_connect_gpio_out_named(dev, "config", i,
                                    qemu_allocate_irq(maclc_set_ram_config,
                                                      m, i));
    }
    if (model->v8) {
        for (i = 0; i < 3; i++) {
            qdev_connect_gpio_out_named(dev, "video", i,
                                        qdev_get_gpio_in_named(
                                            DEVICE(&m->video), "depth", i));
        }
    }
    if (model->hmmu) {
        qdev_connect_gpio_out_named(dev, "port-b", V8_PB_HMMU_OFF,
                                    qemu_allocate_irq(v8_set_hmmu_off, m, 0));
    }

    /* the video's VBL on the pseudo-VIA's slot line 6 */
    qdev_connect_gpio_out_named(DEVICE(&m->video), "vbl", 0,
                                qdev_get_gpio_in_named(DEVICE(&m->pvia),
                                                       "slot", 6));

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
                          qemu_allocate_irq(maclc_set_irq, m, LC_IRQ_SCC));

    /* sound: V8's ASC, Sonora's Enhanced ASC; on the pseudo-VIA */
    object_initialize_child(OBJECT(machine), "asc", &m->asc, TYPE_ASC);
    qdev_prop_set_uint8(DEVICE(&m->asc), "asctype",
                        model->v8 ? ASC_TYPE_ASC : ASC_TYPE_EASC);
    if (machine->audiodev) {
        qdev_prop_set_string(DEVICE(&m->asc), "audiodev", machine->audiodev);
    }
    sysbus = SYS_BUS_DEVICE(&m->asc);
    sysbus_realize(sysbus, &error_fatal);
    memory_region_add_subregion(&m->io, ASC_OFFSET,
                                sysbus_mmio_get_region(sysbus, 0));
    sysbus_connect_irq(sysbus, 0, qdev_get_gpio_in(DEVICE(&m->pvia),
                                                   MAC_PSEUDOVIA_ASC));

    /*
     * SCSI: NCR 53C80 registers every 16 bytes, A9 selecting pseudo-DMA
     * data; the handshake window for blind transfers, also at +$6000
     */
    dev = qdev_new(TYPE_NCR5380);
    m->scsi = NCR5380(dev);
    qdev_prop_set_uint8(dev, "it_shift", 4);
    sysbus = SYS_BUS_DEVICE(dev);
    sysbus_realize_and_unref(sysbus, &error_fatal);
    memory_region_init_io(&m->scsi_regs, OBJECT(m), &maclc_scsi_ops, m->scsi,
                          "maclc.scsi", 0x2000);
    memory_region_add_subregion(&m->io, SCSI_OFFSET, &m->scsi_regs);
    memory_region_add_subregion(&m->io, SCSI_HS_OFFSET,
                                sysbus_mmio_get_region(sysbus,
                                                       NCR5380_MMIO_PDMA));
    memory_region_init_alias(&m->scsi_hs_alias, OBJECT(m), "maclc.scsi-hs",
                             sysbus_mmio_get_region(sysbus, NCR5380_MMIO_PDMA),
                             0, 0x2000);
    memory_region_add_subregion(&m->io, SCSI_HS_ALIAS, &m->scsi_hs_alias);
    qdev_connect_gpio_out(dev, NCR5380_GPIO_IRQ,
                          qdev_get_gpio_in(DEVICE(&m->pvia),
                                           MAC_PSEUDOVIA_SCSI_IRQ));
    qdev_connect_gpio_out(dev, NCR5380_GPIO_DRQ,
                          qdev_get_gpio_in(DEVICE(&m->pvia),
                                           MAC_PSEUDOVIA_SCSI_DRQ));
    scsi_bus_legacy_handle_cmdline(&m->scsi->bus);

    /* SWIM (V8: the original one), head select from VIA1 PA5 */
    object_initialize_child(OBJECT(machine), "swim", &m->swim, TYPE_SWIM2);
    qdev_prop_set_bit(DEVICE(&m->swim), "swim1", model->v8);
    qdev_connect_gpio_out_named(DEVICE(&m->via1), "headsel", 0,
                                qemu_allocate_irq(maclc_headsel, &m->swim, 0));
    sysbus = SYS_BUS_DEVICE(&m->swim);
    sysbus_realize(sysbus, &error_fatal);
    memory_region_add_subregion(&m->io, SWIM_OFFSET,
                                sysbus_mmio_get_region(sysbus, 0));

    /*
     * The PDS slot, $E, with its interrupt on the slot line 5. V8 answers
     * only with A31 low, so the card sees everything at $80000000 and up.
     */
    {
        object_initialize_child(OBJECT(machine), "mac-nubus-bridge",
                                &m->nubus_bridge, TYPE_MAC_NUBUS_BRIDGE);
        dev = DEVICE(&m->nubus_bridge);
        qdev_prop_set_uint32(dev, "slot-available-mask", BIT(0xe));
        sysbus = SYS_BUS_DEVICE(dev);
        sysbus_realize(sysbus, &error_fatal);
        memory_region_add_subregion(sysmem,
                                    MAC_NUBUS_FIRST_SLOT *
                                    NUBUS_SUPER_SLOT_SIZE,
                                    sysbus_mmio_get_region(sysbus, 0));
        memory_region_add_subregion(sysmem,
                                    NUBUS_SLOT_BASE +
                                    MAC_NUBUS_FIRST_SLOT * NUBUS_SLOT_SIZE,
                                    sysbus_mmio_get_region(sysbus, 1));
        qdev_connect_gpio_out(dev, 0xe,
                              qdev_get_gpio_in_named(DEVICE(&m->pvia),
                                                     "slot", 5));
        qdev_connect_gpio_out_named(DEVICE(&m->cpu), "reset-out", 0,
                                    qemu_allocate_irq(maclc_reset_out,
                                                      &m->nubus_bridge, 0));
    }
}

static const MacLCModel maclc_model = {
    .desc = "Macintosh LC",
    .cpu = M68K_CPU_TYPE_NAME("m68020"),
    .rom_filename = "LC.rom",
    .rom_size = 512 * KiB,
    .v8 = true,
    .hmmu = true,
    .onboard_mb = 2,
    .default_ram_mb = 4,
    .calib_mips = 3,
};

static const MacLCModel maclc2_model = {
    .desc = "Macintosh LC II",
    .cpu = M68K_CPU_TYPE_NAME("m68030"),
    .rom_filename = "LCII.rom",
    .rom_size = 512 * KiB,
    .v8 = true,
    .onboard_mb = 4,
    .default_ram_mb = 8,
    .calib_mips = 3,
};

static const MacLCModel maclc3_model = {
    .desc = "Macintosh LC III",
    .cpu = M68K_CPU_TYPE_NAME("m68030"),
    .rom_filename = "LCIII.rom",
    .rom_size = 1 * MiB,
    .machine_id = 0xa55a0001,
    .onboard_mb = 4,
    .default_ram_mb = 8,
    .calib_mips = 5,
};

static void maclc_machine_class_init(ObjectClass *oc, const void *data)
{
    static const char * const valid_cpu_types[] = {
        M68K_CPU_TYPE_NAME("m68020"),
        M68K_CPU_TYPE_NAME("m68030"),
        NULL
    };
    MachineClass *mc = MACHINE_CLASS(oc);
    MacLCMachineClass *mmc = MACLC_MACHINE_CLASS(oc);
    const MacLCModel *model = data;

    if (!model) {
        return;                 /* the abstract parent */
    }
    mmc->model = model;
    mc->desc = model->desc;
    mc->default_cpu_type = model->cpu;
    mc->init = maclc_machine_init;
    mc->reset = maclc_machine_reset;
    mc->valid_cpu_types = valid_cpu_types;
    mc->max_cpus = 1;
    mc->block_default_type = IF_SCSI;
    mc->default_ram_id = "maclc.ram";
    mc->default_ram_size = model->default_ram_mb * MiB;
    mac_scsi_compat_add(mc);
    machine_add_audiodev_property(mc);
    calib_governor_add_machine_property(oc);
}

static const TypeInfo maclc_machine_types[] = {
    {
        .name           = TYPE_MACLC_MACHINE,
        .parent         = TYPE_MACHINE,
        .abstract       = true,
        .instance_size  = sizeof(MacLCMachineState),
        .class_size     = sizeof(MacLCMachineClass),
        .class_init     = maclc_machine_class_init,
    }, {
        .name           = MACHINE_TYPE_NAME("maclc"),
        .parent         = TYPE_MACLC_MACHINE,
        .class_init     = maclc_machine_class_init,
        .class_data     = &maclc_model,
    }, {
        .name           = MACHINE_TYPE_NAME("maclc2"),
        .parent         = TYPE_MACLC_MACHINE,
        .class_init     = maclc_machine_class_init,
        .class_data     = &maclc2_model,
    }, {
        .name           = MACHINE_TYPE_NAME("maclc3"),
        .parent         = TYPE_MACLC_MACHINE,
        .class_init     = maclc_machine_class_init,
        .class_data     = &maclc3_model,
    },
};

DEFINE_TYPES(maclc_machine_types)

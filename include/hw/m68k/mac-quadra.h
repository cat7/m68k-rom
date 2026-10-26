/*
 * QEMU Motorla 680x0 Macintosh hardware System Emulator
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef HW_M68K_MAC_QUADRA_H
#define HW_M68K_MAC_QUADRA_H

#include "hw/core/boards.h"
#include "qom/object.h"
#include "target/m68k/cpu-qom.h"
#include "system/memory.h"
#include "hw/m68k/q800-glue.h"
#include "hw/misc/mac_via.h"
#include "hw/net/dp8393x.h"
#include "hw/char/escc.h"
#include "hw/core/or-irq.h"
#include "hw/core/split-irq.h"
#include "hw/scsi/esp.h"
#include "hw/block/swim2.h"
#include "hw/nubus/mac-nubus-bridge.h"
#include "hw/display/macfb.h"
#include "hw/misc/djmemc.h"
#include "hw/misc/iosb.h"
#include "hw/misc/orwell.h"
#include "hw/misc/mac_iop.h"
#include "hw/audio/asc.h"

/*
 * 68040 Macintosh Quadra family. Each model (q800, q700, ...) is a
 * subclass of the abstract TYPE_MAC_QUADRA_MACHINE that only selects a
 * MacQuadraModel describing how it differs from the others.
 */

typedef enum MacMemCtrl {
    MAC_MEMCTRL_DJMEMC,     /* djMEMC + IOSB: Quadra 800, Centris 610/650 */
    MAC_MEMCTRL_ORWELL,     /* Orwell: Quadra 700/900/950 */
} MacMemCtrl;

typedef enum MacFloppyCtrl {
    MAC_FLOPPY_SWIM1,       /* original SWIM (IWM + ISM): Quadra 700/900 */
    MAC_FLOPPY_SWIM2,       /* SWIM2 (ISM only): Quadra 800 */
} MacFloppyCtrl;

typedef struct MacQuadraModel {
    const char *desc;
    const char *rom_filename;     /* default for -bios */
    uint32_t bootinfo_model;      /* Gestalt ID passed to Linux */
    uint8_t via1_cpuid;           /* ID bits on VIA1 port A */
    uint8_t via1_porta_pullup;    /* VIA1 port A inputs that read as 1 */
    uint8_t via2_portb_pullup;    /* VIA2 port B inputs that read as 1 */
    MacMemCtrl memctrl;
    MacFloppyCtrl floppy;
    bool has_machine_id_reg;      /* ID longword at 0x5ffffffc */
    uint32_t machine_id;          /* its value (0: Quadra 800's) */
    bool no_sonic;                /* no on-board Ethernet */
    bool cuda;                    /* ADB, RTC and PRAM behind Cuda */
    bool egret;                   /* RTC and PRAM behind Egret */
    bool rom_mirror;              /* ROM repeats every 1 MiB up to 0x50000000 */
    bool iop;                     /* SCC and SWIM/ADB behind IOPs */
    bool default_easc;
    bool scsi_drq_dafb;           /* SCSI DRQ also readable in DAFB */
    hwaddr esp_base;
    hwaddr esp_pdma;
    hwaddr esp2_base;             /* second (external) SCSI bus, 0: none */
    hwaddr esp2_pdma;
    uint32_t nubus_slots;         /* NuBus slot-available-mask */
    ram_addr_t max_ram_size;
    /* screen when no -g is given (0: 800x600x8) */
    uint32_t fb_width, fb_height;
    uint8_t fb_depth;
    uint32_t vram_size;           /* on-board VRAM (0: 4 MiB) */
    bool lc_video;                /* LC 475 frame layouts (macfb lc-layout) */
} MacQuadraModel;

struct MacQuadraMachineClass {
    MachineClass parent_class;

    const MacQuadraModel *model;
};

struct MacQuadraMachineState {
    MachineState parent_obj;

    bool easc;
    M68kCPU cpu;
    MemoryRegion rom;
    MemoryRegion rom_alias;
    MemoryRegion rom_mirror[16];
    GLUEState glue;
    MOS6522Q800VIA1State via1;
    MOS6522Q800VIA2State via2;
    dp8393xState dp8393x;
    MemoryRegion dp8393x_prom;
    ESCCState escc;
    OrIRQState escc_orgate;
    SplitIRQ scsi_drq_split;
    SysBusESPState esp;
    SysBusESPState esp2;
    OrIRQState scsi_irq_or;
    SWIM2State swim2;
    MacNubusBridge mac_nubus_bridge;
    MacfbNubusState macfb;
    DJMEMCState djmemc;
    IOSBState iosb;
    OrwellState orwell;
    MacIOPState scc_iop;
    MacIOPState swim_iop;
    ASCState asc;
    MemoryRegion ramio;
    MemoryRegion macio;
    MemoryRegion macio_alias;
    MemoryRegion machine_id;
    MemoryRegion escc_alias;
};

#define TYPE_MAC_QUADRA_MACHINE MACHINE_TYPE_NAME("mac-quadra")
OBJECT_DECLARE_TYPE(MacQuadraMachineState, MacQuadraMachineClass,
                    MAC_QUADRA_MACHINE)

#define TYPE_Q800_MACHINE MACHINE_TYPE_NAME("q800")
#define TYPE_Q700_MACHINE MACHINE_TYPE_NAME("q700")
#define TYPE_Q900_MACHINE MACHINE_TYPE_NAME("q900")
#define TYPE_LC475_MACHINE MACHINE_TYPE_NAME("lc475")

/* Base of the on-board I/O area (VIAs, SCC, SCSI, ...) */
#define MAC_QUADRA_IO_BASE    0x50000000

/* class_init for a model type: pass its MacQuadraModel as class_data */
void mac_quadra_model_class_init(ObjectClass *oc, const void *data);

#endif

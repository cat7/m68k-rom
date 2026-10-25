/*
 * Orwell memory controller (Macintosh Quadra 700/900/950)
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_ORWELL_H
#define HW_MISC_ORWELL_H

#include "hw/core/sysbus.h"

#define ORWELL_SIZE        0x2000
#define ORWELL_NUM_REGS    (ORWELL_SIZE / sizeof(uint32_t))

struct OrwellState {
    SysBusDevice parent_obj;

    MemoryRegion mem_regs;

    /* Register layout not yet known: store whatever the ROM writes */
    uint32_t regs[ORWELL_NUM_REGS];
};

#define TYPE_ORWELL "orwell"
OBJECT_DECLARE_SIMPLE_TYPE(OrwellState, ORWELL);

#endif

/*
 * Orwell memory controller (Macintosh Quadra 700/900/950)
 *
 * Experimental: registers are plain storage until their meaning is
 * worked out from what the ROM does with them.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "migration/vmstate.h"
#include "hw/misc/orwell.h"
#include "trace.h"

static uint64_t orwell_read(void *opaque, hwaddr addr, unsigned size)
{
    OrwellState *s = opaque;
    uint64_t val = s->regs[addr >> 2];

    trace_orwell_read(addr, val, size);
    return val;
}

static void orwell_write(void *opaque, hwaddr addr, uint64_t val,
                         unsigned size)
{
    OrwellState *s = opaque;

    trace_orwell_write(addr, val, size);
    s->regs[addr >> 2] = val;
}

static const MemoryRegionOps orwell_mmio_ops = {
    .read = orwell_read,
    .write = orwell_write,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
    .endianness = DEVICE_BIG_ENDIAN,
};

static void orwell_init(Object *obj)
{
    OrwellState *s = ORWELL(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->mem_regs, obj, &orwell_mmio_ops, s, "orwell",
                          ORWELL_SIZE);
    sysbus_init_mmio(sbd, &s->mem_regs);
}

static void orwell_reset_hold(Object *obj, ResetType type)
{
    OrwellState *s = ORWELL(obj);

    memset(s->regs, 0, sizeof(s->regs));
}

static const VMStateDescription vmstate_orwell = {
    .name = "orwell",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, OrwellState, ORWELL_NUM_REGS),
        VMSTATE_END_OF_LIST()
    }
};

static void orwell_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->vmsd = &vmstate_orwell;
    rc->phases.hold = orwell_reset_hold;
}

static const TypeInfo orwell_info_types[] = {
    {
        .name = TYPE_ORWELL,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(OrwellState),
        .instance_init = orwell_init,
        .class_init = orwell_class_init,
    },
};

DEFINE_TYPES(orwell_info_types)

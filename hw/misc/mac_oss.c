/*
 * Apple OSS (Operating System Support) chip of the Macintosh IIfx
 *
 * The OSS takes the place of VIA2: an interrupt controller with a 3-bit
 * level register per source, a raw pending register, the 60.15 Hz tick,
 * ROM control (the reset overlay, power off) and a free-running counter.
 *
 *   $000-$00F  level of source n (0 masks it)
 *   $202-$203  sources pending (raw, not masked)
 *   $204       ROM control: bit 3 = overlay off, bit 7 = power off
 *   $205       counter control
 *   $206       input status (bit 1 must read 0: not burn-in)
 *   $207       60 Hz acknowledge (read or write)
 *   $208-$20F  64-bit counter
 *
 * The CPU's interrupt level is the highest level of the pending sources.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/timer.h"
#include "qemu/log.h"
#include "hw/core/irq.h"
#include "hw/misc/mac_oss.h"
#include "migration/vmstate.h"
#include "system/runstate.h"
#include "trace.h"

#define OSS_TICK_NS     (NANOSECONDS_PER_SECOND * 100 / 6015)

#define OSS_PENDING     0x202
#define OSS_ROM_CTRL    0x204
#define OSS_CNT_CTRL    0x205
#define OSS_INPUTS      0x206
#define OSS_60HZ_ACK    0x207
#define OSS_COUNTER     0x208

#define OSS_ROM_OVERLAY_OFF 0x08
#define OSS_ROM_POWER_OFF   0x80

static uint16_t mac_oss_pending(MacOSSState *s)
{
    return s->inputs | (s->tick ? BIT(OSS_SRC_60HZ) : 0);
}

static void mac_oss_update(MacOSSState *s)
{
    uint16_t pending = mac_oss_pending(s);
    int i, ipl = 0;

    for (i = 0; i < MAC_OSS_NUM_SRC; i++) {
        if ((pending & BIT(i)) && s->level[i] > ipl) {
            ipl = s->level[i];
        }
    }
    trace_mac_oss_ipl(pending, ipl);
    qemu_set_irq(s->ipl, ipl);
}

static void mac_oss_set_input(void *opaque, int n, int level)
{
    MacOSSState *s = opaque;

    s->inputs = deposit32(s->inputs, n, 1, !!level);
    mac_oss_update(s);
}

static void mac_oss_tick(void *opaque)
{
    MacOSSState *s = opaque;

    s->tick = true;
    mac_oss_update(s);
    s->next_tick += OSS_TICK_NS;
    timer_mod(s->timer, s->next_tick);
}

/* a microsecond count; the low 8 bits read 0 */
static uint64_t mac_oss_counter(MacOSSState *s)
{
    if (s->counter_ctrl & 1) {
        return s->counter_hold;
    }
    return (qemu_clock_get_us(QEMU_CLOCK_VIRTUAL) << 8);
}

static uint64_t mac_oss_read(void *opaque, hwaddr addr, unsigned size)
{
    MacOSSState *s = opaque;
    uint64_t val = 0;

    if (addr < MAC_OSS_NUM_SRC) {
        val = s->level[addr];
    } else {
        switch (addr) {
        case OSS_PENDING:
            val = mac_oss_pending(s) >> 8;
            break;
        case OSS_PENDING + 1:
            val = mac_oss_pending(s) & 0xff;
            break;
        case OSS_ROM_CTRL:
            val = s->rom_ctrl;
            break;
        case OSS_CNT_CTRL:
            val = s->counter_ctrl;
            break;
        case OSS_INPUTS:
            val = 0;            /* no phono plug; not burn-in */
            break;
        case OSS_60HZ_ACK:
            s->tick = false;
            mac_oss_update(s);
            break;
        case OSS_COUNTER ... OSS_COUNTER + 7:
            val = mac_oss_counter(s) >> (8 * (7 - (addr - OSS_COUNTER)));
            val &= 0xff;
            break;
        default:
            qemu_log_mask(LOG_UNIMP, "mac-oss: read at 0x%" HWADDR_PRIx "\n",
                          addr);
            break;
        }
    }
    trace_mac_oss_read(addr, val);
    return val;
}

static void mac_oss_write(void *opaque, hwaddr addr, uint64_t val,
                          unsigned size)
{
    MacOSSState *s = opaque;

    trace_mac_oss_write(addr, val);
    if (addr < MAC_OSS_NUM_SRC) {
        s->level[addr] = val & 7;
        mac_oss_update(s);
        return;
    }
    switch (addr) {
    case OSS_ROM_CTRL:
        s->rom_ctrl = val;
        qemu_set_irq(s->overlay, !(val & OSS_ROM_OVERLAY_OFF));
        if (val & OSS_ROM_POWER_OFF) {
            qemu_system_shutdown_request(SHUTDOWN_CAUSE_GUEST_SHUTDOWN);
        }
        break;
    case OSS_CNT_CTRL:
        if ((val & 1) && !(s->counter_ctrl & 1)) {
            s->counter_hold = qemu_clock_get_us(QEMU_CLOCK_VIRTUAL) << 8;
        }
        s->counter_ctrl = val & 3;
        break;
    case OSS_60HZ_ACK:
        s->tick = false;
        mac_oss_update(s);
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "mac-oss: write 0x%02" PRIx64 " at 0x%"
                      HWADDR_PRIx "\n", val, addr);
        break;
    }
}

static const MemoryRegionOps mac_oss_ops = {
    .read = mac_oss_read,
    .write = mac_oss_write,
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

static void mac_oss_reset_hold(Object *obj, ResetType type)
{
    MacOSSState *s = MAC_OSS(obj);

    memset(s->level, 0, sizeof(s->level));
    s->tick = false;
    s->rom_ctrl = 0;
    s->counter_ctrl = 0;
    qemu_set_irq(s->overlay, 1);
    mac_oss_update(s);
    s->next_tick = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + OSS_TICK_NS;
    timer_mod(s->timer, s->next_tick);
}

static void mac_oss_init(Object *obj)
{
    MacOSSState *s = MAC_OSS(obj);

    memory_region_init_io(&s->mem, obj, &mac_oss_ops, s, "mac-oss",
                          MAC_OSS_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mem);
    qdev_init_gpio_in(DEVICE(obj), mac_oss_set_input, MAC_OSS_NUM_SRC);
    qdev_init_gpio_out_named(DEVICE(obj), &s->ipl, "ipl", 1);
    qdev_init_gpio_out_named(DEVICE(obj), &s->overlay, "overlay", 1);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, mac_oss_tick, s);
}

static const VMStateDescription vmstate_mac_oss = {
    .name = "mac-oss",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8_ARRAY(level, MacOSSState, MAC_OSS_NUM_SRC),
        VMSTATE_UINT16(inputs, MacOSSState),
        VMSTATE_BOOL(tick, MacOSSState),
        VMSTATE_UINT8(rom_ctrl, MacOSSState),
        VMSTATE_UINT8(counter_ctrl, MacOSSState),
        VMSTATE_UINT64(counter_hold, MacOSSState),
        VMSTATE_TIMER_PTR(timer, MacOSSState),
        VMSTATE_INT64(next_tick, MacOSSState),
        VMSTATE_END_OF_LIST()
    },
};

static void mac_oss_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->desc = "Apple OSS interrupt controller (Macintosh IIfx)";
    dc->vmsd = &vmstate_mac_oss;
    rc->phases.hold = mac_oss_reset_hold;
}

static const TypeInfo mac_oss_info = {
    .name = TYPE_MAC_OSS,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MacOSSState),
    .instance_init = mac_oss_init,
    .class_init = mac_oss_class_init,
};

static void mac_oss_register_types(void)
{
    type_register_static(&mac_oss_info);
}

type_init(mac_oss_register_types)

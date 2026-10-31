/*
 * Philips SAA7191 colour decoder and SAA7186 video scaler
 *
 * Only their registers: the picture itself comes from the host and goes
 * straight into the frame buffer (see mac_civic.c), which reads the
 * scaler's size and format here.
 *
 * Writes start with a register number, then go on to the next registers.
 * The SAA7191's one readable byte is its status: bit 6 set while it has
 * no horizontal lock, bit 5 set for 60 Hz fields, bit 0 set when it finds
 * colour.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "hw/display/saa71xx.h"
#include "trace.h"

#define SAA7191_STATUS_HLCK     0x40    /* set: not locked */
#define SAA7191_STATUS_FIDT     0x20    /* set: 60 Hz */
#define SAA7191_STATUS_CODE     0x01    /* colour found */

static int saa7191_event(I2CSlave *i2c, enum i2c_event event)
{
    SAA7191State *s = SAA7191(i2c);

    if (event == I2C_START_SEND) {
        s->sub_set = false;
    }
    return 0;
}

static int saa7191_send(I2CSlave *i2c, uint8_t data)
{
    SAA7191State *s = SAA7191(i2c);

    if (!s->sub_set) {
        s->sub = data;
        s->sub_set = true;
        return 0;
    }
    trace_saa7191_write(s->sub, data);
    if (s->sub < SAA7191_NREGS) {
        s->regs[s->sub] = data;
    }
    s->sub++;
    return 0;
}

static uint8_t saa7191_recv(I2CSlave *i2c)
{
    SAA7191State *s = SAA7191(i2c);
    uint8_t status;

    if (!s->signal) {
        status = SAA7191_STATUS_HLCK;
    } else {
        /* the source follows the standard the decoder is set to */
        status = SAA7191_STATUS_CODE;
        if (!saa7191_50hz(s)) {
            status |= SAA7191_STATUS_FIDT;
        }
    }
    trace_saa7191_status(status);
    return status;
}

static void saa7191_reset(DeviceState *dev)
{
    SAA7191State *s = SAA7191(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->sub = 0;
    s->sub_set = false;
}

static const VMStateDescription vmstate_saa7191 = {
    .name = TYPE_SAA7191,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, SAA7191State),
        VMSTATE_UINT8_ARRAY(regs, SAA7191State, SAA7191_NREGS),
        VMSTATE_UINT8(sub, SAA7191State),
        VMSTATE_BOOL(sub_set, SAA7191State),
        VMSTATE_END_OF_LIST()
    },
};

static const Property saa7191_properties[] = {
    DEFINE_PROP_BOOL("signal", SAA7191State, signal, false),
};

static void saa7191_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(oc);

    k->event = saa7191_event;
    k->send = saa7191_send;
    k->recv = saa7191_recv;
    device_class_set_legacy_reset(dc, saa7191_reset);
    dc->vmsd = &vmstate_saa7191;
    device_class_set_props(dc, saa7191_properties);
}

static int saa7186_event(I2CSlave *i2c, enum i2c_event event)
{
    SAA7186State *s = SAA7186(i2c);

    if (event == I2C_START_SEND) {
        s->sub_set = false;
    }
    return 0;
}

static int saa7186_send(I2CSlave *i2c, uint8_t data)
{
    SAA7186State *s = SAA7186(i2c);

    if (!s->sub_set) {
        s->sub = data;
        s->sub_set = true;
        return 0;
    }
    trace_saa7186_write(s->sub, data);
    if (s->sub < SAA7186_NREGS) {
        s->regs[s->sub] = data;
    }
    s->sub++;
    return 0;
}

static uint8_t saa7186_recv(I2CSlave *i2c)
{
    return 0;
}

static void saa7186_reset(DeviceState *dev)
{
    SAA7186State *s = SAA7186(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->sub = 0;
    s->sub_set = false;
}

static const VMStateDescription vmstate_saa7186 = {
    .name = TYPE_SAA7186,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, SAA7186State),
        VMSTATE_UINT8_ARRAY(regs, SAA7186State, SAA7186_NREGS),
        VMSTATE_UINT8(sub, SAA7186State),
        VMSTATE_BOOL(sub_set, SAA7186State),
        VMSTATE_END_OF_LIST()
    },
};

static void saa7186_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(oc);

    k->event = saa7186_event;
    k->send = saa7186_send;
    k->recv = saa7186_recv;
    device_class_set_legacy_reset(dc, saa7186_reset);
    dc->vmsd = &vmstate_saa7186;
}

static const TypeInfo saa71xx_types[] = {
    {
        .name = TYPE_SAA7191,
        .parent = TYPE_I2C_SLAVE,
        .instance_size = sizeof(SAA7191State),
        .class_init = saa7191_class_init,
    },
    {
        .name = TYPE_SAA7186,
        .parent = TYPE_I2C_SLAVE,
        .instance_size = sizeof(SAA7186State),
        .class_init = saa7186_class_init,
    },
};

DEFINE_TYPES(saa71xx_types)

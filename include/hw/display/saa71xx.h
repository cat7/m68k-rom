/*
 * Philips SAA7191 colour decoder and SAA7186 video scaler, the video input
 * chips of the Quadra 660AV and 840AV (on Cuda's I2C bus)
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_DISPLAY_SAA71XX_H
#define HW_DISPLAY_SAA71XX_H

#include "hw/i2c/i2c.h"
#include "qom/object.h"

#define TYPE_SAA7191 "saa7191"
#define TYPE_SAA7186 "saa7186"
OBJECT_DECLARE_SIMPLE_TYPE(SAA7191State, SAA7191)
OBJECT_DECLARE_SIMPLE_TYPE(SAA7186State, SAA7186)

#define SAA7191_I2C_ADDR    0x45        /* 0x8A on the wire */
#define SAA7186_I2C_ADDR    0x5c        /* 0xB8 on the wire */

#define SAA7191_NREGS       0x19
#define SAA7186_NREGS       0x11

struct SAA7191State {
    I2CSlave parent_obj;

    uint8_t regs[SAA7191_NREGS];
    uint8_t sub;                /* register for the next byte */
    bool sub_set;               /* the transfer's first byte has come */
    bool signal;                /* a video source is connected */
};

struct SAA7186State {
    I2CSlave parent_obj;

    uint8_t regs[SAA7186_NREGS];
    uint8_t sub;
    bool sub_set;
};

/* SAA7186 output formats (register 0 bits 1-0) */
enum {
    SAA7186_FMT_RGB555,
    SAA7186_FMT_RGB888,
    SAA7186_FMT_YUV422,
    SAA7186_FMT_MONO,
};

static inline bool saa7186_running(SAA7186State *s)
{
    return s->regs[0] & 0x10;
}

static inline unsigned saa7186_format(SAA7186State *s)
{
    return s->regs[0] & 3;
}

/* the picture's output size */
static inline unsigned saa7186_width(SAA7186State *s)
{
    return s->regs[1] | (s->regs[4] & 3) << 8;
}

/* registers 5 and 8 hold the number of lines plus one */
static inline unsigned saa7186_lines(SAA7186State *s)
{
    unsigned n = s->regs[5] | (s->regs[8] & 3) << 8;

    return n ? n - 1 : 0;
}

/* the decoder is set to a 50 Hz standard (PAL, SECAM) */
static inline bool saa7191_50hz(SAA7191State *s)
{
    return s->regs[0x0d] & 1;
}

#endif

/*
 * Apple PSC ("Peripheral Subsystem Controller") of the Quadra 660AV and
 * 840AV
 *
 * The PSC holds VIA1 (a separate device here), a pseudo-VIA2 for the slot,
 * SCSI, floppy and sound-frame interrupts, four more interrupt levels with
 * flag and enable registers of their own, a DMA engine (SCSI, Ethernet
 * receive and transmit, floppy, SCC) and the sound engine that streams the
 * Singer codec's samples from and to memory.
 *
 * Pseudo-VIA2 ($50F02000): VIA register n at n * $200, byte wide; only the
 * IFR (13), IER (14) and the slot register (1 and 15, active low) work, the
 * ACR (11) and PCR (12) read fixed values.
 *
 * Registers ($50F31000):
 *   +$130 + $10 * (l - 3)  level l flags, +4 its enables (3 <= l <= 6),
 *                          in the most significant byte; bit 7 of the flags
 *                          reads "any enabled", writing a 1 clears an event
 *   +$200  sndComCtl (16 bits): 6 frame interrupt, 7 input, 8 output,
 *          9-10 rate (24, 32, 48 kHz)
 *   +$204  singerCtl          +$20C  sndPhase (read only)
 *   +$210  sndInBase          +$214  sndOutBase
 *   +$218  sndSize (16 bits)  +$21C  DSPOVERRUN (8 bits)
 *   +$804  DMA interrupt status, bit 31 - channel
 *   +$C00 + $10 * c        channel c's control (16 bits)
 *   +$1000 + $20 * c + $10 * s   set s of channel c: address, count,
 *                          command (16 bits, at +8)
 *
 * The sound engine plays and records two halves of a buffer of 2 * sndSize
 * stereo 16-bit samples, interrupting at the end of each half (a "frame").
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/timer.h"
#include "qemu/main-loop.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "system/address-spaces.h"
#include "system/dma.h"
#include "hw/misc/mac_psc.h"
#include "migration/vmstate.h"
#include "trace.h"

/* pseudo-VIA2 IFR bits */
#define PV_SCSI_DRQ     0x01
#define PV_ANY_SLOT     0x02
#define PV_SCSI_IRQ     0x08
#define PV_FDC          0x20
#define PV_SND_FRAME    0x40

/* sndComCtl */
#define SNDCOM_FRMIE    0x0040
#define SNDCOM_INEN     0x0080
#define SNDCOM_OUTEN    0x0100

/* DMA control */
#define CTRL_SET        0x0001
#define CTRL_CIRQ       0x0100
#define CTRL_FLUSH      0x0200
#define CTRL_PAUSE      0x0400
#define CTRL_SWRESET    0x0800
#define CTRL_CIE        0x1000
#define CTRL_BERR       0x2000
#define CTRL_FROZEN     0x4000
#define CTRL_SETBITS    0x8000

/* DMA command */
#define CMD_IF          0x0100
#define CMD_DIR         0x0200      /* device to memory */
#define CMD_TERMCNT     0x0400
#define CMD_ENABLED     0x0800
#define CMD_IE          0x1000

#define DMA_SCSI        0
#define DMA_ENET_RX     1
#define DMA_ENET_TX     2

#define ENET_RX_SLOT    0x800       /* one received frame per 2 KB */

#define OUT_RING        8192        /* stereo samples */

static const uint32_t snd_rates[4] = { 24000, 32000, 48000, 48000 };

static void psc_update_ipl(MacPSCState *s)
{
    int ipl = 0, l;

    for (l = 6; l >= 3 && !ipl; l--) {
        if ((s->lvl_src[l - 3] | s->lvl_latch[l - 3]) & s->lvl_ier[l - 3] &
            0x7f) {
            ipl = l;
        }
    }
    if (!ipl && (s->ifr & s->ier & 0x7f)) {
        ipl = 2;
    }
    if (!ipl && s->via1_irq) {
        ipl = 1;
    }
    if (ipl != s->cur_ipl) {
        s->cur_ipl = ipl;
        qemu_set_irq(s->ipl, ipl);
    }
}

static void psc_update_slots(MacPSCState *s)
{
    if (s->slot_lines & 0x7f) {
        s->ifr |= PV_ANY_SLOT;
    } else {
        s->ifr &= ~PV_ANY_SLOT;
    }
    psc_update_ipl(s);
}

/* DMA: channel interrupts on level 4 bit 3 */
static void psc_update_dma_irq(MacPSCState *s)
{
    int c, set;
    bool any = false;

    for (c = 0; c < MAC_PSC_DMA_CHANNELS; c++) {
        s->dma_ctrl[c] &= ~CTRL_CIRQ;
        for (set = 0; set < 2; set++) {
            if ((s->dma_set[c][set].cmd & (CMD_IE | CMD_IF)) ==
                (CMD_IE | CMD_IF)) {
                s->dma_ctrl[c] |= CTRL_CIRQ;
            }
        }
        if ((s->dma_ctrl[c] & (CTRL_CIRQ | CTRL_CIE)) ==
            (CTRL_CIRQ | CTRL_CIE)) {
            any = true;
        }
    }
    s->lvl_src[1] = deposit32(s->lvl_src[1], 3, 1, any);
    psc_update_ipl(s);
}

static uint32_t psc_dma_irq_status(MacPSCState *s)
{
    uint32_t val = 0;
    int c;

    for (c = 0; c < MAC_PSC_DMA_CHANNELS; c++) {
        if ((s->dma_ctrl[c] & (CTRL_CIRQ | CTRL_CIE)) ==
            (CTRL_CIRQ | CTRL_CIE)) {
            val |= 0x80000000u >> c;
        }
    }
    return val;
}

/*
 * SCSI DMA: while the 53C96 asks for data and the channel's active set is
 * enabled, move words through the chip's pseudo-DMA window.
 */
static void psc_scsi_dma(MacPSCState *s)
{
    MemTxAttrs attrs = MEMTXATTRS_UNSPECIFIED;
    MacPSCDMASet *set;
    uint64_t val;
    unsigned size;

    if (s->pumping || !s->scsi_pdma) {
        return;
    }
    s->pumping = true;
    for (;;) {
        set = &s->dma_set[DMA_SCSI][s->dma_ctrl[DMA_SCSI] & CTRL_SET];
        if (!s->scsi_drq || !(set->cmd & CMD_ENABLED) || !set->count) {
            break;
        }
        size = set->count == 1 ? 1 : 2;
        if (set->cmd & CMD_DIR) {
            memory_region_dispatch_read(s->scsi_pdma, 0, &val,
                                        size_memop(size) | MO_BE, attrs);
            if (size == 2) {
                stw_be_phys(s->dma_as, set->addr, val);
            } else {
                stb_phys(s->dma_as, set->addr, val);
            }
        } else {
            val = size == 2 ? lduw_be_phys(s->dma_as, set->addr)
                            : ldub_phys(s->dma_as, set->addr);
            memory_region_dispatch_write(s->scsi_pdma, 0, val,
                                         size_memop(size) | MO_BE, attrs);
        }
        set->addr += size;
        set->count -= size;
        if (!set->count) {
            set->cmd &= ~CMD_ENABLED;
            set->cmd |= CMD_IF;
            psc_update_dma_irq(s);
        }
    }
    s->pumping = false;
}

static void psc_scsi_bh(void *opaque)
{
    psc_scsi_dma(opaque);
}

/*
 * A set is done: interrupt, and go on with the other set if that one is
 * ready (the Ethernet channels ping-pong between their two sets)
 */
static void psc_dma_set_done(MacPSCState *s, int c)
{
    int cur = s->dma_ctrl[c] & CTRL_SET;
    MacPSCDMASet *set = &s->dma_set[c][cur];

    set->cmd &= ~CMD_ENABLED;
    set->cmd |= CMD_IF;
    if (s->dma_set[c][cur ^ 1].cmd & CMD_ENABLED) {
        s->dma_ctrl[c] ^= CTRL_SET;
    }
    psc_update_dma_irq(s);
}

static MacPSCDMASet *psc_enet_set(MacPSCState *s, int c)
{
    MacPSCDMASet *set;

    if (s->dma_ctrl[c] & CTRL_PAUSE) {
        return NULL;
    }
    set = &s->dma_set[c][s->dma_ctrl[c] & CTRL_SET];
    if (!(set->cmd & CMD_ENABLED) || !set->count) {
        /* this one is used up: the other one, if it is ready */
        set = &s->dma_set[c][(s->dma_ctrl[c] & CTRL_SET) ^ 1];
        if (!(set->cmd & CMD_ENABLED) || !set->count) {
            return NULL;
        }
        s->dma_ctrl[c] ^= CTRL_SET;
    }
    return set;
}

bool mac_psc_enet_receive(void *opaque, const uint8_t *buf, size_t len,
                          const uint8_t status[4])
{
    MacPSCState *s = opaque;
    MacPSCDMASet *set = psc_enet_set(s, DMA_ENET_RX);
    /* the four status bytes, each in both byte lanes of a word */
    uint8_t hdr[16] = { status[0], status[0], status[1], status[1],
                        status[2], status[2], status[3], status[3] };

    if (!set) {
        return false;
    }
    len = MIN(len, ENET_RX_SLOT - sizeof(hdr));
    dma_memory_write(s->dma_as, set->addr, hdr, sizeof(hdr),
                     MEMTXATTRS_UNSPECIFIED);
    dma_memory_write(s->dma_as, set->addr + sizeof(hdr), buf, len,
                     MEMTXATTRS_UNSPECIFIED);
    set->addr += ENET_RX_SLOT;
    set->count--;
    if (set->count) {
        set->cmd |= CMD_IF;
        psc_update_dma_irq(s);
    } else {
        psc_dma_set_done(s, DMA_ENET_RX);
    }
    return true;
}

static void psc_enet_tx(MacPSCState *s)
{
    MacPSCDMASet *set;
    uint8_t buf[2048];
    uint32_t len;

    while (s->mace && mac_mace_can_transmit(s->mace) &&
           (set = psc_enet_set(s, DMA_ENET_TX))) {
        len = MIN(set->count, sizeof(buf));
        dma_memory_read(s->dma_as, set->addr, buf, len,
                        MEMTXATTRS_UNSPECIFIED);
        set->addr += set->count;
        set->count = 0;
        mac_mace_transmit(s->mace, buf, len);
        psc_dma_set_done(s, DMA_ENET_TX);
    }
}

/* a channel was (re)programmed */
static void psc_dma_kick(MacPSCState *s, int c)
{
    switch (c) {
    case DMA_SCSI:
        psc_scsi_dma(s);
        break;
    case DMA_ENET_RX:
        if (s->mace) {
            mac_mace_rx_ready(s->mace);
        }
        break;
    case DMA_ENET_TX:
        psc_enet_tx(s);
        break;
    }
}

/* --- sound ---------------------------------------------------------- */

static uint32_t psc_snd_rate(MacPSCState *s)
{
    return snd_rates[(s->snd_com >> 9) & 3];
}

static uint32_t psc_snd_size(MacPSCState *s)
{
    return MAX(s->snd_size, 4);
}

/* samples played since the epoch, in 64ths */
static uint64_t psc_snd_ticks64(MacPSCState *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    return muldiv64(now - s->snd_epoch_ns, psc_snd_rate(s) * 64ULL,
                    NANOSECONDS_PER_SECOND);
}

static uint32_t psc_snd_phase(MacPSCState *s)
{
    uint64_t t = psc_snd_ticks64(s);
    uint32_t pos = (t >> 6) % (2 * psc_snd_size(s));

    return ((pos & 0xfff) << 6) | (t & 63);
}

static void psc_arm_frame(MacPSCState *s)
{
    uint64_t samples = (uint64_t)s->frame_next * psc_snd_size(s);
    int64_t when = s->snd_epoch_ns +
                   muldiv64(samples, NANOSECONDS_PER_SECOND, psc_snd_rate(s));

    timer_mod(s->frame_timer, when);
}

/* restart the sample counter (rate or size change) */
static void psc_snd_restart(MacPSCState *s)
{
    s->snd_epoch_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    s->frame_next = 1;
    psc_arm_frame(s);
}

static void psc_out_cb(void *opaque, int free_b)
{
    MacPSCState *s = opaque;
    int16_t buf[512];
    int n, i;

    while (free_b >= 4) {
        n = MIN(free_b / 4, ARRAY_SIZE(buf) / 2);
        for (i = 0; i < n; i++) {
            if (s->out_head != s->out_tail) {
                buf[2 * i] = s->out_ring[2 * s->out_head];
                buf[2 * i + 1] = s->out_ring[2 * s->out_head + 1];
                s->out_head = (s->out_head + 1) % OUT_RING;
            } else {
                buf[2 * i] = buf[2 * i + 1] = 0;
            }
        }
        n = audio_be_write(s->audio_be, s->voice, buf, n * 4);
        if (n <= 0) {
            break;
        }
        free_b -= n;
    }
}

static void psc_open_voice(MacPSCState *s)
{
    struct audsettings as = {
        .freq = psc_snd_rate(s),
        .nchannels = 2,
        .fmt = AUDIO_FORMAT_S16,
        .big_endian = HOST_BIG_ENDIAN,
    };

    if (!s->audio_be) {
        return;
    }
    s->voice = audio_be_open_out(s->audio_be, s->voice, "psc.out", s,
                                 psc_out_cb, &as);
}

static void psc_frame(void *opaque)
{
    MacPSCState *s = opaque;
    uint32_t size = psc_snd_size(s);
    uint32_t half = s->frame_next & 1;
    uint32_t i, base, next;
    int16_t l, r;
    bool mute;

    if (s->snd_com & SNDCOM_OUTEN) {
        base = s->snd_out_base + half * size * 4;
        mute = (s->singer_ctl & 0x00400000) || (s->dsp_overrun & 1);
        for (i = 0; i < size; i++) {
            next = (s->out_tail + 1) % OUT_RING;
            if (next == s->out_head) {
                break;
            }
            l = mute ? 0 : lduw_be_phys(s->dma_as, base + i * 4);
            r = mute ? 0 : lduw_be_phys(s->dma_as, base + i * 4 + 2);
            s->out_ring[2 * s->out_tail] = l;
            s->out_ring[2 * s->out_tail + 1] = r;
            s->out_tail = next;
        }
    }
    if (s->snd_com & SNDCOM_INEN) {
        /* no input: a little noise, as silence upsets speech recognition */
        base = s->snd_in_base + half * size * 4;
        for (i = 0; i < size * 2; i++) {
            stw_be_phys(s->dma_as, base + i * 2, (i * 7 + s->frame_next) % 3 - 1);
        }
    }
    if (s->snd_com & SNDCOM_FRMIE) {
        s->ifr |= PV_SND_FRAME;
        psc_update_ipl(s);
    }
    s->frame_next++;
    psc_arm_frame(s);
}

static void psc_snd_com_write(MacPSCState *s, uint16_t val)
{
    uint16_t old = s->snd_com;

    s->snd_com = val;
    if ((old ^ val) & (3 << 9)) {
        psc_snd_restart(s);
        if (val & SNDCOM_OUTEN) {
            psc_open_voice(s);
        }
    }
    if ((old ^ val) & SNDCOM_OUTEN) {
        if (!(val & SNDCOM_OUTEN)) {
            s->out_head = s->out_tail = 0;
        }
        if (s->voice) {
            audio_be_set_active_out(s->audio_be, s->voice,
                                    !!(val & SNDCOM_OUTEN));
        }
    }
}

/* --- pseudo-VIA2 ----------------------------------------------------- */

static uint64_t psc_via2_read(void *opaque, hwaddr addr, unsigned size)
{
    MacPSCState *s = opaque;
    uint8_t val;

    switch ((addr >> 9) & 0xf) {
    case 1:
    case 15:
        val = ~s->slot_lines & 0x7f;
        break;
    case 11:
        val = 0xc0;
        break;
    case 12:
        val = 0x26;
        break;
    case 13:
        val = s->ifr | ((s->ifr & s->ier & 0x7f) ? 0x80 : 0);
        break;
    case 14:
        val = s->ier | 0x80;
        break;
    default:
        val = 0xff;
        break;
    }
    trace_mac_psc_via2_read(addr, val);
    return val;
}

static void psc_via2_write(void *opaque, hwaddr addr, uint64_t val,
                           unsigned size)
{
    MacPSCState *s = opaque;

    trace_mac_psc_via2_write(addr, val);
    switch ((addr >> 9) & 0xf) {
    case 13:
        /* only the sound frame flag is cleared here; others at the source */
        if (val & PV_SND_FRAME) {
            s->ifr &= ~PV_SND_FRAME;
        }
        break;
    case 14:
        if (val & 0x80) {
            s->ier |= val & 0x7f;
        } else {
            s->ier &= ~val & 0x7f;
        }
        break;
    }
    psc_update_ipl(s);
}

static const MemoryRegionOps psc_via2_ops = {
    .read = psc_via2_read,
    .write = psc_via2_write,
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

/* --- registers ------------------------------------------------------- */

static uint32_t psc_reg_read32(MacPSCState *s, hwaddr addr)
{
    MacPSCDMASet *set;
    int c, l;

    if (addr >= 0x130 && addr < 0x170 && (addr & 0xf) < 8) {
        l = (addr - 0x130) >> 4;
        if (addr & 4) {
            return (uint32_t)s->lvl_ier[l] << 24;
        }
        c = s->lvl_src[l] | s->lvl_latch[l];
        if (c & s->lvl_ier[l] & 0x7f) {
            c |= 0x80;
        }
        return (uint32_t)c << 24;
    }
    if (addr >= 0xc00 && addr < 0xc00 + 0x10 * MAC_PSC_DMA_CHANNELS) {
        c = (addr - 0xc00) >> 4;
        return (uint32_t)(s->dma_ctrl[c] | 0x8000) << 16;
    }
    if (addr >= 0x1000 && addr < 0x1000 + 0x20 * MAC_PSC_DMA_CHANNELS) {
        c = (addr - 0x1000) >> 5;
        set = &s->dma_set[c][(addr >> 4) & 1];
        switch (addr & 0xc) {
        case 0:
            return set->addr;
        case 4:
            return set->count;
        case 8:
            return (uint32_t)(set->cmd | (set->count ? 0 : CMD_TERMCNT)) << 16;
        }
        return 0;
    }
    switch (addr) {
    case 0x200:
        return (uint32_t)s->snd_com << 16;
    case 0x204:
        return s->singer_ctl;
    case 0x20c:
        return psc_snd_phase(s);
    case 0x210:
        return s->snd_in_base;
    case 0x214:
        return s->snd_out_base;
    case 0x218:
        return (uint32_t)s->snd_size << 16;
    case 0x21c:
        return (uint32_t)s->dsp_overrun << 24;
    case 0x804:
        return psc_dma_irq_status(s);
    }
    qemu_log_mask(LOG_UNIMP, "mac-psc: read of register 0x%03" HWADDR_PRIx
                  "\n", addr);
    return 0;
}

static void psc_dma_ctrl_write(MacPSCState *s, int c, uint16_t val)
{
    uint16_t *ctrl = &s->dma_ctrl[c];

    if (val & CTRL_SETBITS) {
        *ctrl |= val & (CTRL_CIE | CTRL_PAUSE);
        if (val & CTRL_SWRESET) {
            *ctrl = (*ctrl & CTRL_CIE) | CTRL_FROZEN | CTRL_PAUSE;
            s->dma_set[c][0].cmd &= ~CMD_ENABLED;
            s->dma_set[c][1].cmd &= ~CMD_ENABLED;
        }
        if (val & CTRL_PAUSE) {
            *ctrl |= CTRL_FROZEN;
        }
        if (val & CTRL_FLUSH) {
            s->dma_set[c][*ctrl & CTRL_SET].cmd &= ~CMD_ENABLED;
            *ctrl = (*ctrl ^ CTRL_SET) | CTRL_PAUSE | CTRL_FROZEN;
        }
    } else {
        *ctrl &= ~(val & (CTRL_CIE | CTRL_PAUSE | CTRL_BERR));
        if (val & CTRL_PAUSE) {
            *ctrl &= ~CTRL_FROZEN;
        }
    }
    psc_update_dma_irq(s);
}

static void psc_reg_write32(MacPSCState *s, hwaddr addr, uint32_t val,
                            uint32_t mask)
{
    MacPSCDMASet *set;
    uint16_t bits;
    int c, l;

    if (addr >= 0x130 && addr < 0x170 && (addr & 0xf) < 8) {
        if (!(mask & 0xff000000)) {
            return;
        }
        l = (addr - 0x130) >> 4;
        bits = val >> 24;
        if (addr & 4) {
            if (bits & 0x80) {
                s->lvl_ier[l] |= bits & 0x7f;
            } else {
                s->lvl_ier[l] &= ~bits & 0x7f;
            }
        } else {
            s->lvl_latch[l] &= ~bits;
        }
        psc_update_ipl(s);
        return;
    }
    if (addr >= 0xc00 && addr < 0xc00 + 0x10 * MAC_PSC_DMA_CHANNELS) {
        c = (addr - 0xc00) >> 4;
        if (mask & 0xffff0000) {
            psc_dma_ctrl_write(s, c, val >> 16);
            psc_dma_kick(s, c);
        }
        return;
    }
    if (addr >= 0x1000 && addr < 0x1000 + 0x20 * MAC_PSC_DMA_CHANNELS) {
        c = (addr - 0x1000) >> 5;
        set = &s->dma_set[c][(addr >> 4) & 1];
        switch (addr & 0xc) {
        case 0:
            set->addr = (set->addr & ~mask) | (val & mask);
            break;
        case 4:
            set->count = (set->count & ~mask) | (val & mask);
            break;
        case 8:
            if (mask & 0xffff0000) {
                bits = (val >> 16) & (CMD_IE | CMD_ENABLED | CMD_DIR | CMD_IF);
                if (val & 0x80000000) {
                    set->cmd |= bits;
                } else {
                    set->cmd &= ~bits;
                }
            }
            break;
        }
        psc_update_dma_irq(s);
        psc_dma_kick(s, c);
        return;
    }
    switch (addr) {
    case 0x200:
        if (mask & 0xffff0000) {
            psc_snd_com_write(s, val >> 16);
        }
        return;
    case 0x204:
        s->singer_ctl = (s->singer_ctl & ~mask) | (val & mask);
        return;
    case 0x210:
        s->snd_in_base = ((s->snd_in_base & ~mask) | (val & mask)) & ~0xfu;
        return;
    case 0x214:
        s->snd_out_base = ((s->snd_out_base & ~mask) | (val & mask)) & ~0xfu;
        return;
    case 0x218:
        if (mask & 0xffff0000) {
            s->snd_size = (val >> 16) & 0x07fc;
            psc_snd_restart(s);
        }
        return;
    case 0x21c:
        if (mask & 0xff000000) {
            bits = val >> 24;
            if (bits & 0x80) {
                s->dsp_overrun |= bits & 7;
            } else {
                s->dsp_overrun &= ~bits & 7;
            }
        }
        return;
    }
    qemu_log_mask(LOG_UNIMP, "mac-psc: write 0x%08x to register 0x%03"
                  HWADDR_PRIx "\n", val, addr);
}

static uint64_t psc_regs_read(void *opaque, hwaddr addr, unsigned size)
{
    MacPSCState *s = opaque;
    uint32_t val = psc_reg_read32(s, addr & ~3);
    unsigned shift = (4 - (addr & 3) - size) * 8;

    val = extract32(val, shift, size * 8);
    trace_mac_psc_reg_read(addr, size, val);
    return val;
}

static void psc_regs_write(void *opaque, hwaddr addr, uint64_t val,
                           unsigned size)
{
    MacPSCState *s = opaque;
    unsigned shift = (4 - (addr & 3) - size) * 8;
    uint32_t mask = MAKE_64BIT_MASK(shift, size * 8);

    trace_mac_psc_reg_write(addr, size, val);
    psc_reg_write32(s, addr & ~3, (uint32_t)val << shift, mask);
}

static const MemoryRegionOps psc_regs_ops = {
    .read = psc_regs_read,
    .write = psc_regs_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

/* --- inputs ---------------------------------------------------------- */

static void psc_set_irq(void *opaque, int n, int level)
{
    MacPSCState *s = opaque;

    switch (n) {
    case MAC_PSC_IN_VIA1:
        s->via1_irq = level;
        break;
    case MAC_PSC_IN_SCSI_IRQ:
        s->ifr = deposit32(s->ifr, 3, 1, level);
        break;
    case MAC_PSC_IN_SCSI_DRQ:
        /*
         * The 53C96 raises DRQ while it is updating its own state: move
         * the data later, not from inside the chip
         */
        s->scsi_drq = level;
        s->ifr = deposit32(s->ifr, 0, 1, level);
        if (level) {
            qemu_bh_schedule(s->scsi_bh);
        }
        break;
    case MAC_PSC_IN_FDC:
        s->ifr = deposit32(s->ifr, 5, 1, level);
        break;
    case MAC_PSC_IN_ENET:
        s->lvl_src[0] = deposit32(s->lvl_src[0], 0, 1, level);
        break;
    case MAC_PSC_IN_SCC_A:
        s->lvl_src[1] = deposit32(s->lvl_src[1], 1, 1, level);
        s->lvl_src[3] = deposit32(s->lvl_src[3], 1, 1, level);
        break;
    case MAC_PSC_IN_SCC_B:
        s->lvl_src[1] = deposit32(s->lvl_src[1], 2, 1, level);
        s->lvl_src[3] = deposit32(s->lvl_src[3], 2, 1, level);
        break;
    }
    psc_update_ipl(s);
}

static void psc_set_slot(void *opaque, int n, int level)
{
    MacPSCState *s = opaque;

    s->slot_lines = deposit32(s->slot_lines, n, 1, level);
    psc_update_slots(s);
}

static void psc_reset(DeviceState *dev)
{
    MacPSCState *s = MAC_PSC(dev);
    int c;

    s->ifr &= PV_SCSI_DRQ | PV_SCSI_IRQ | PV_FDC | PV_ANY_SLOT;
    s->ier = 0;
    memset(s->lvl_latch, 0, sizeof(s->lvl_latch));
    memset(s->lvl_ier, 0, sizeof(s->lvl_ier));
    for (c = 0; c < MAC_PSC_DMA_CHANNELS; c++) {
        s->dma_ctrl[c] = 0;
        memset(s->dma_set[c], 0, sizeof(s->dma_set[c]));
    }
    s->snd_com = 0;
    s->singer_ctl = 0x00400000;     /* muted */
    s->snd_size = 240;              /* 10 ms at 24 kHz */
    s->dsp_overrun = 0;
    s->out_head = s->out_tail = 0;
    psc_snd_restart(s);
    psc_update_dma_irq(s);
}

static void psc_realize(DeviceState *dev, Error **errp)
{
    MacPSCState *s = MAC_PSC(dev);

    s->dma_as = &address_space_memory;
    s->scsi_bh = qemu_bh_new_guarded(psc_scsi_bh, s,
                                     &DEVICE(s)->mem_reentrancy_guard);
    s->frame_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, psc_frame, s);
    s->out_ring = g_new0(int16_t, 2 * OUT_RING);
    if (s->audio_be || audio_be_check(&s->audio_be, NULL)) {
        psc_open_voice(s);
    }
}

static void psc_init(Object *obj)
{
    MacPSCState *s = MAC_PSC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->via2_mmio, obj, &psc_via2_ops, s,
                          "mac-psc.via2", 0x2000);
    memory_region_init_io(&s->regs_mmio, obj, &psc_regs_ops, s,
                          "mac-psc.regs", 0x2000);
    sysbus_init_mmio(sbd, &s->via2_mmio);
    sysbus_init_mmio(sbd, &s->regs_mmio);
    qdev_init_gpio_in(DEVICE(obj), psc_set_irq, MAC_PSC_IN_NB);
    qdev_init_gpio_in_named(DEVICE(obj), psc_set_slot, "slot",
                            MAC_PSC_SLOT_LINES);
    qdev_init_gpio_out_named(DEVICE(obj), &s->ipl, "ipl", 1);
}

static const VMStateDescription vmstate_psc_dma_set = {
    .name = "mac-psc/dma-set",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(addr, MacPSCDMASet),
        VMSTATE_UINT32(count, MacPSCDMASet),
        VMSTATE_UINT16(cmd, MacPSCDMASet),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_psc = {
    .name = "mac-psc",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8(ifr, MacPSCState),
        VMSTATE_UINT8(ier, MacPSCState),
        VMSTATE_UINT8(slot_lines, MacPSCState),
        VMSTATE_UINT8_ARRAY(lvl_src, MacPSCState, 4),
        VMSTATE_UINT8_ARRAY(lvl_latch, MacPSCState, 4),
        VMSTATE_UINT8_ARRAY(lvl_ier, MacPSCState, 4),
        VMSTATE_BOOL(via1_irq, MacPSCState),
        VMSTATE_BOOL(scsi_drq, MacPSCState),
        VMSTATE_INT32(cur_ipl, MacPSCState),
        VMSTATE_UINT16_ARRAY(dma_ctrl, MacPSCState, MAC_PSC_DMA_CHANNELS),
        VMSTATE_STRUCT_2DARRAY(dma_set, MacPSCState, MAC_PSC_DMA_CHANNELS, 2,
                               1, vmstate_psc_dma_set, MacPSCDMASet),
        VMSTATE_UINT16(snd_com, MacPSCState),
        VMSTATE_UINT32(singer_ctl, MacPSCState),
        VMSTATE_UINT32(snd_in_base, MacPSCState),
        VMSTATE_UINT32(snd_out_base, MacPSCState),
        VMSTATE_UINT16(snd_size, MacPSCState),
        VMSTATE_UINT8(dsp_overrun, MacPSCState),
        VMSTATE_INT64(snd_epoch_ns, MacPSCState),
        VMSTATE_INT64(frame_next, MacPSCState),
        VMSTATE_TIMER_PTR(frame_timer, MacPSCState),
        VMSTATE_END_OF_LIST()
    },
};

static const Property psc_properties[] = {
    DEFINE_AUDIO_PROPERTIES(MacPSCState, audio_be),
};

static void psc_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = psc_realize;
    device_class_set_legacy_reset(dc, psc_reset);
    dc->vmsd = &vmstate_psc;
    device_class_set_props(dc, psc_properties);
}

static const TypeInfo psc_types[] = {
    {
        .name = TYPE_MAC_PSC,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(MacPSCState),
        .instance_init = psc_init,
        .class_init = psc_class_init,
    },
};

DEFINE_TYPES(psc_types)

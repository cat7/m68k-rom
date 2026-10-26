/*
 *
 * Copyright (c) 2011-2018 Laurent Vivier
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */

#ifndef HW_MISC_MAC_VIA_H
#define HW_MISC_MAC_VIA_H

#include "system/memory.h"
#include "hw/core/sysbus.h"
#include "hw/misc/mos6522.h"
#include "hw/input/adb.h"
#include "qom/object.h"


#define VIA_SIZE   0x2000

/* Machine ID bits on VIA1 port A */
#define VIA1A_CPUID0    0x02    /* CPU id bit 0 on RBV, others */
#define VIA1A_CPUID1    0x04    /* CPU id bit 0 on RBV, others */
#define VIA1A_CPUID2    0x10    /* CPU id bit 0 on RBV, others */
#define VIA1A_CPUID3    0x40    /* CPU id bit 0 on RBV, others */
#define VIA1A_CPUID_MASK (VIA1A_CPUID0 | VIA1A_CPUID1 | \
                          VIA1A_CPUID2 | VIA1A_CPUID3)
#define VIA1A_CPUID_Q800 (VIA1A_CPUID0 | VIA1A_CPUID2)

/* VIA 1 */
#define VIA1_IRQ_ONE_SECOND_BIT CA2_INT_BIT
#define VIA1_IRQ_60HZ_BIT       CA1_INT_BIT
#define VIA1_IRQ_ADB_READY_BIT  SR_INT_BIT
#define VIA1_IRQ_ADB_DATA_BIT   CB2_INT_BIT
#define VIA1_IRQ_ADB_CLOCK_BIT  CB1_INT_BIT

#define VIA1_IRQ_ONE_SECOND     BIT(VIA1_IRQ_ONE_SECOND_BIT)
#define VIA1_IRQ_60HZ           BIT(VIA1_IRQ_60HZ_BIT)
#define VIA1_IRQ_ADB_READY      BIT(VIA1_IRQ_ADB_READY_BIT)
#define VIA1_IRQ_ADB_DATA       BIT(VIA1_IRQ_ADB_DATA_BIT)
#define VIA1_IRQ_ADB_CLOCK      BIT(VIA1_IRQ_ADB_CLOCK_BIT)


#define TYPE_MOS6522_Q800_VIA1 "mos6522-q800-via1"
OBJECT_DECLARE_SIMPLE_TYPE(MOS6522Q800VIA1State, MOS6522_Q800_VIA1)

struct MOS6522Q800VIA1State {
    /*< private >*/
    MOS6522State parent_obj;

    MemoryRegion via_mem;

    qemu_irq auxmode_irq;
    qemu_irq headsel_irq;
    qemu_irq overlay_irq;         /* PA4: ROM at address 0 (Mac II) */
    uint8_t last_b;

    /* Machine ID bits returned on port A (the cpuid_mask bits) */
    uint8_t cpuid;
    uint8_t cpuid_mask;           /* default VIA1A_CPUID_MASK */
    /* Port A input pins pulled high on the board */
    uint8_t porta_pullup;

    /* RTC */
    uint8_t PRAM[256];
    BlockBackend *blk;
    VMChangeStateEntry *vmstate;

    uint32_t tick_offset;

    uint8_t data_out;
    int data_out_cnt;
    uint8_t data_in;
    uint8_t data_in_cnt;
    uint8_t cmd;
    int wprotect;
    int alt;

    /* ADB */
    ADBBusState adb_bus;
    qemu_irq adb_data_ready;
    int adb_data_in_size;
    int adb_data_in_index;
    int adb_data_out_index;
    uint8_t adb_data_in[128];
    uint8_t adb_data_out[16];
    uint8_t adb_autopoll_cmd;

    /* external timers */
    QEMUTimer *one_second_timer;
    int64_t next_second;
    QEMUTimer *sixty_hz_timer;
    int64_t next_sixty_hz;

    /* SETUPTIMEK hack */
    int timer_hack_state;

    /*
     * Cuda (LC 475, Quadra 605 and later): ADB, RTC and PRAM sit behind a
     * microcontroller talking to the VIA shift register, with TIP, TACK
     * and TREQ on port B instead of the ADB state and RTC lines.
     */
    bool cuda;
    bool egret;                   /* Egret: same packets, other handshake */
    bool adb;                     /* ADB transceiver on PB4/PB5/SR */
    QEMUTimer *cuda_sr_timer;
    int64_t cuda_sr_due;          /* pending SR interrupt time, 0: none */
    int cuda_sr_reads;            /* VIA reads left before it is raised */
    uint8_t cuda_last_b;
    int cuda_in_size;
    int cuda_in_index;
    int cuda_out_index;
    uint8_t cuda_in[300];
    uint8_t cuda_out[64];
    uint8_t cuda_one_sec_mode;
    bool cuda_time_sent;
    bool cuda_in_time;      /* cuda_in holds the time packet */
    uint8_t cuda_mem[0x100];     /* 68HC05 page 0 (registers, RAM) */
};


/* VIA 2 */
#define VIA2_IRQ_SCSI_DATA_BIT  CA2_INT_BIT
#define VIA2_IRQ_NUBUS_BIT      CA1_INT_BIT
#define VIA2_IRQ_SCSI_BIT       CB2_INT_BIT
#define VIA2_IRQ_ASC_BIT        CB1_INT_BIT

#define VIA2_IRQ_SCSI_DATA      BIT(VIA2_IRQ_SCSI_DATA_BIT)
#define VIA2_IRQ_NUBUS          BIT(VIA2_IRQ_NUBUS_BIT)
#define VIA2_IRQ_UNUSED         BIT(VIA2_IRQ_SCSI_BIT)
#define VIA2_IRQ_SCSI           BIT(VIA2_IRQ_UNUSED_BIT)
#define VIA2_IRQ_ASC            BIT(VIA2_IRQ_ASC_BIT)

#define VIA2_NUBUS_IRQ_NB       7

#define VIA2_NUBUS_IRQ_9        0
#define VIA2_NUBUS_IRQ_A        1
#define VIA2_NUBUS_IRQ_B        2
#define VIA2_NUBUS_IRQ_C        3
#define VIA2_NUBUS_IRQ_D        4
#define VIA2_NUBUS_IRQ_E        5
#define VIA2_NUBUS_IRQ_INTVIDEO 6

#define TYPE_MOS6522_Q800_VIA2 "mos6522-q800-via2"
OBJECT_DECLARE_SIMPLE_TYPE(MOS6522Q800VIA2State, MOS6522_Q800_VIA2)

struct MOS6522Q800VIA2State {
    /*< private >*/
    MOS6522State parent_obj;

    MemoryRegion via_mem;
    /* Port B inputs pulled high on the board */
    uint8_t portb_pullup;
    /* port pin levels for the board (outputs, or high when inputs) */
    qemu_irq port_a_out[8];
    qemu_irq port_b_out[8];
};

#endif

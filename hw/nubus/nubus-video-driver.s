; Video driver for the QEMU NuBus video card (hw/nubus/nubus-video.c)
;
; This is a standard 'DRVR' format Mac OS video driver.  The card's
; declaration ROM carries it in the sRsrcDrvrDir of its video sResource,
; from where the Slot Manager loads it at startup.
;
; The card registers sit at $0FD000 in each megabyte of the slot space
; but the ones holding VRAM, so the driver uses $FssFD000 (s = slot),
; which works both in 32-bit mode and in 24-bit mode (as $sFD000).
; The hardware knows
; the screen size and works out the row bytes and frame offset of each
; depth, so the driver only tells it the depth.
;
; The code uses 68000 instructions only.
;
; Rebuild hw/nubus/nubus-video-driver.h after changing this file:
;   vasmm68k_mot -quiet -Fbin -m68000 -o drvr.bin nubus-video-driver.s
;   python3 -I nubus-video-driver.py drvr.bin > nubus-video-driver.h
; (vasm: http://sun.hasenbraten.de/vasm/, built with
;  "make CPU=m68k SYNTAX=mot")
;
; SPDX-License-Identifier: GPL-2.0-or-later

; Card registers (keep in sync with nubus-video.c)
REG_OFFSET      equ     $0FD000
REG_MODE        equ     $00             ; depth: 0..5 = 1,2,4,8,16,32 bpp
REG_ROWBYTES    equ     $04
REG_BASE        equ     $08             ; frame offset from the slot base
REG_HEIGHT      equ     $10
REG_NMODES      equ     $18             ; number of depths offered
REG_INT_ENABLE  equ     $20
REG_INT_STATUS  equ     $24             ; bit 0: VBL; write 1 to clear
REG_CLUT_INDEX  equ     $30
REG_CLUT_DATA   equ     $34             ; $00RRGGBB, index auto-increments

FIRST_MODE      equ     $80             ; spID of the 1 bpp mode
DIRECT_MODE     equ     $84             ; spID of the 16 bpp mode

; Low memory
JIODone         equ     $08FC
JVBLTask        equ     $0D28

; Parameter block
ioTrap          equ     6
csCode          equ     26
csParam         equ     28
noQueueBit      equ     9

; Device control entry
dCtlStorage     equ     20
dCtlSlot        equ     40

; Slot interrupt queue element
sqType          equ     4
sqPrio          equ     6
sqAddr          equ     8
sqParm          equ     12
sIQType         equ     6

; Video parameter records
csMode          equ     0               ; VDPageInfo
csData          equ     2
csPage          equ     6
csBaseAddr      equ     8
csTable         equ     0               ; VDSetEntryRecord
csStart         equ     4
csCount         equ     6

; Result codes
noErr           equ     0
openErr         equ     -23
controlErr      equ     -17
statusErr       equ     -18
paramErr        equ     -50

; Private storage
pSQ             equ     0               ; slot interrupt queue element
pRegs           equ     16
pDevBase        equ     20              ; $Fs000000
pMode           equ     24              ; word, current spID
pMaxMode        equ     26              ; word
pGray           equ     28              ; byte, luminance mapping on
pIntOff         equ     29              ; byte, VBL interrupts disabled
pSlot           equ     30              ; byte
pDefMode        equ     31              ; byte
pGamma          equ     32              ; gamma table: 12-byte header
pGammaData      equ     pGamma+12       ; then 3 x 256 bytes
pSize           equ     pGammaData+768

; Traps
        macro   _NewHandleSysClear
        dc.w    $A722
        endm
        macro   _HLock
        dc.w    $A029
        endm
        macro   _DisposHandle
        dc.w    $A023
        endm
        macro   _SIntInstall
        dc.w    $A075
        endm
        macro   _SIntRemove
        dc.w    $A076
        endm
        macro   _SwapMMUMode
        dc.w    $A05D
        endm
        macro   _StripAddress
        dc.w    $A055
        endm

; A3 = private storage of the DCE in A1 (the master pointer may carry
; flags in its high byte in 24-bit mode)
        macro   GetStorage
        move.l  dCtlStorage(a1),a0
        move.l  (a0),d0
        _StripAddress
        move.l  d0,a3
        endm

DrvrStart:
        dc.w    $4C00                   ; dNeedLock, dStatEnable, dCtlEnable
        dc.w    0                       ; delay
        dc.w    0                       ; event mask
        dc.w    0                       ; menu
        dc.w    VidOpen-DrvrStart
        dc.w    VidPrime-DrvrStart
        dc.w    VidCtl-DrvrStart
        dc.w    VidStatus-DrvrStart
        dc.w    VidClose-DrvrStart
        dc.b    DrvrNameEnd-DrvrName-1
DrvrName:
        dc.b    ".Display_Video_QEMU_NuBus"
DrvrNameEnd:
        even

; ---------------------------------------------------------------------
; Open: A0 = parameter block, A1 = DCE

VidOpen:
        movem.l d3-d7/a2-a4,-(sp)
        move.l  a1,a2                   ; a2 = DCE
        move.l  #pSize,d0
        _NewHandleSysClear
        bne     .fail
        move.l  a0,dCtlStorage(a2)
        _HLock
        move.l  a2,a1
        GetStorage

        moveq   #0,d0
        move.b  dCtlSlot(a2),d0
        move.b  d0,pSlot(a3)
        move.l  d0,d1
        ror.l   #8,d0                   ; slot << 24
        or.l    #$F0000000,d0
        move.l  d0,pDevBase(a3)
        swap    d1
        lsl.l   #4,d1                   ; slot << 20
        or.l    d1,d0
        add.l   #REG_OFFSET,d0
        move.l  d0,pRegs(a3)
        move.l  d0,a4                   ; a4 = registers

        clr.l   REG_INT_ENABLE(a4)
        moveq   #1,d0
        move.l  d0,REG_INT_STATUS(a4)
        move.l  REG_NMODES(a4),d0
        add.w   #FIRST_MODE-1,d0
        move.w  d0,pMaxMode(a3)
        move.b  #FIRST_MODE,pDefMode(a3)

        bsr     LinearGamma

        lea     pSQ(a3),a0
        move.w  #sIQType,sqType(a0)
        clr.w   sqPrio(a0)
        lea     VidInt(pc),a1
        move.l  a1,sqAddr(a0)
        move.l  a3,sqParm(a0)
        moveq   #0,d0
        move.b  pSlot(a3),d0
        _SIntInstall
        bne     .fail2

        moveq   #1,d0
        move.l  d0,REG_INT_ENABLE(a4)

        move.w  #FIRST_MODE,d0
        bsr     SetMode
        bsr     GrayScreen
        moveq   #noErr,d0
.exit:
        movem.l (sp)+,d3-d7/a2-a4
        rts
.fail2:
        move.l  dCtlStorage(a2),a0
        _DisposHandle
        clr.l   dCtlStorage(a2)
.fail:
        moveq   #openErr,d0
        bra     .exit

; ---------------------------------------------------------------------
; Close: A0 = parameter block, A1 = DCE

VidClose:
        movem.l a2-a3,-(sp)
        move.l  a1,a2
        tst.l   dCtlStorage(a2)
        beq     .done
        GetStorage
        move.l  pRegs(a3),a0
        clr.l   REG_INT_ENABLE(a0)
        lea     pSQ(a3),a0
        moveq   #0,d0
        move.b  pSlot(a3),d0
        _SIntRemove
        move.l  dCtlStorage(a2),a0
        _DisposHandle
        clr.l   dCtlStorage(a2)
.done:
        moveq   #noErr,d0
        movem.l (sp)+,a2-a3
        rts

; ---------------------------------------------------------------------
; Slot interrupt: A1 = private storage.  Returns D0 = 1 when serviced.

VidInt:
        move.l  pRegs(a1),a0
        moveq   #1,d0
        and.l   REG_INT_STATUS(a0),d0
        beq     .notours
        move.l  d0,REG_INT_STATUS(a0)   ; acknowledge
        moveq   #0,d0
        move.b  pSlot(a1),d0
        move.l  JVBLTask,a0
        jsr     (a0)                    ; run this slot's VBL tasks
        moveq   #1,d0
.notours:
        rts

; ---------------------------------------------------------------------
; Prime is never called (no read/write enable); fail it anyway

VidPrime:
        moveq   #controlErr,d0
        bra     IOExit

; ---------------------------------------------------------------------
; Control: A0 = parameter block, A1 = DCE

VidCtl:
        movem.l d3-d7/a2-a4,-(sp)
        move.l  a0,a4
        GetStorage
        move.l  csParam(a4),a2
        move.w  csCode(a4),d0
        cmp.w   #9,d0
        bhi     CtlBad
        add.w   d0,d0
        move.w  .tab(pc,d0.w),d0
        jmp     .tab(pc,d0.w)
.tab:
        dc.w    CtlReset-.tab           ; 0 cscReset
        dc.w    CtlDone0-.tab           ; 1 cscKillIO
        dc.w    CtlSetMode-.tab         ; 2
        dc.w    CtlSetEntries-.tab      ; 3
        dc.w    CtlSetGamma-.tab        ; 4
        dc.w    CtlGrayPage-.tab        ; 5
        dc.w    CtlSetGray-.tab         ; 6
        dc.w    CtlSetInterrupt-.tab    ; 7
        dc.w    CtlDirectSetEntries-.tab ; 8
        dc.w    CtlSetDefaultMode-.tab  ; 9

CtlBad:
        moveq   #controlErr,d0
        bra     CtlDone
CtlParamErr:
        moveq   #paramErr,d0
        bra     CtlDone
CtlDone0:
        moveq   #noErr,d0
CtlDone:
        move.l  a4,a0
        movem.l (sp)+,d3-d7/a2-a4
IOExit:
        move.w  ioTrap(a0),d1
        btst    #noQueueBit,d1
        beq     .queued
        rts
.queued:
        move.l  JIODone,-(sp)
        rts

CtlReset:
        move.w  #FIRST_MODE,d0
        bsr     SetMode
        bsr     GrayScreen
        move.w  #FIRST_MODE,csMode(a2)
        clr.w   csPage(a2)
        bsr     GetBase
        move.l  d0,csBaseAddr(a2)
        bra     CtlDone0

CtlSetMode:
        tst.w   csPage(a2)
        bne     CtlParamErr
        move.w  csMode(a2),d0
        bsr     CheckMode
        bne     CtlParamErr
        bsr     SetMode
        bsr     GrayScreen
        bsr     GetBase
        move.l  d0,csBaseAddr(a2)
        bra     CtlDone0

CtlGrayPage:
        tst.w   csPage(a2)
        bne     CtlParamErr
        bsr     GrayScreen
        bra     CtlDone0

CtlSetGray:
        tst.b   csMode(a2)
        sne     pGray(a3)
        bra     CtlDone0

CtlSetInterrupt:
        move.l  pRegs(a3),a0
        moveq   #0,d0
        move.b  csMode(a2),pIntOff(a3)  ; 0 = enabled, 1 = disabled
        seq     d0
        and.w   #1,d0
        move.l  d0,REG_INT_ENABLE(a0)
        bra     CtlDone0

CtlSetDefaultMode:
        move.b  csMode(a2),pDefMode(a3)
        bra     CtlDone0

CtlSetEntries:
        cmp.w   #DIRECT_MODE,pMode(a3)
        bhs     CtlBad
        bra     SetEntries
CtlDirectSetEntries:
        cmp.w   #DIRECT_MODE,pMode(a3)
        blo     CtlBad
SetEntries:
        move.l  csTable(a2),d0
        beq     CtlParamErr
        move.l  d0,a0                   ; a0 = ColorSpec array
        move.w  csStart(a2),d1          ; d1 = next index, or -1
        move.w  csCount(a2),d2          ; d2 = count - 1
        cmp.w   #255,d2
        bhi     CtlParamErr
        move.l  pRegs(a3),a1
        lea     pGammaData(a3),a2       ; a2 = red, green, blue tables
.loop:
        moveq   #0,d3
        move.w  d1,d3
        bpl     .seq
        move.w  (a0),d3                 ; csStart = -1: value is the index
        subq.w  #1,d1                   ; (cancel the increment below)
.seq:
        and.w   #$FF,d3
        move.l  d3,REG_CLUT_INDEX(a1)
        moveq   #0,d4
        moveq   #0,d5
        moveq   #0,d6
        move.b  2(a0),d4                ; high bytes of red, green, blue
        move.b  4(a0),d5
        move.b  6(a0),d6
        tst.b   pGray(a3)
        beq     .color
        mulu    #30,d4                  ; luminance
        mulu    #59,d5
        mulu    #11,d6
        add.l   d5,d4
        add.l   d6,d4
        divu    #100,d4
        and.l   #$FF,d4
        move.l  d4,d5
        move.l  d4,d6
.color:
        move.b  0(a2,d4.w),d4           ; gamma correction
        add.w   #256,d5
        move.b  0(a2,d5.w),d5
        add.w   #512,d6
        move.b  0(a2,d6.w),d6
        and.l   #$FF,d4
        and.w   #$FF,d5
        swap    d4                      ; $00RR0000
        lsl.w   #8,d5
        move.w  d5,d4                   ; $00RRGG00
        move.b  d6,d4                   ; $00RRGGBB
        move.l  d4,REG_CLUT_DATA(a1)
        addq.l  #8,a0
        addq.w  #1,d1
        dbra    d2,.loop
        bra     CtlDone0

CtlSetGamma:
        lea     pGammaData(a3),a1
        move.l  (a2),d0                 ; csGTable
        bne     .table
        bsr     LinearGamma
        bra     CtlDone0
.table:
        move.l  d0,a0
        cmp.w   #256,8(a0)              ; gDataCnt
        bne     CtlParamErr
        cmp.w   #8,10(a0)               ; gDataWidth
        bne     CtlParamErr
        move.w  6(a0),d3                ; gChanCnt: 1 or 3
        cmp.w   #1,d3
        beq     .ok
        cmp.w   #3,d3
        bne     CtlParamErr
.ok:
        move.w  4(a0),d0                ; gFormulaSize
        lea     12(a0,d0.w),a0
        moveq   #2,d2
.chan:
        move.w  #255,d1
.copy:
        move.b  (a0)+,(a1)+
        dbra    d1,.copy
        cmp.w   #1,d3
        bne     .next
        lea     -256(a0),a0             ; one table for all channels
.next:
        dbra    d2,.chan
        bra     CtlDone0

; ---------------------------------------------------------------------
; Status: A0 = parameter block, A1 = DCE

VidStatus:
        movem.l d3-d7/a2-a4,-(sp)
        move.l  a0,a4
        GetStorage
        move.l  csParam(a4),a2
        move.w  csCode(a4),d0
        cmp.w   #9,d0
        bhi     StBad
        add.w   d0,d0
        move.w  .tab(pc,d0.w),d0
        jmp     .tab(pc,d0.w)
.tab:
        dc.w    StBad-.tab              ; 0
        dc.w    StBad-.tab              ; 1
        dc.w    StGetMode-.tab          ; 2
        dc.w    StGetEntries-.tab       ; 3
        dc.w    StGetPages-.tab         ; 4
        dc.w    StGetBaseAddr-.tab      ; 5
        dc.w    StGetGray-.tab          ; 6
        dc.w    StGetInterrupt-.tab     ; 7
        dc.w    StGetGamma-.tab         ; 8
        dc.w    StGetDefaultMode-.tab   ; 9

StBad:
        moveq   #statusErr,d0
        bra     CtlDone
StParamErr:
        moveq   #paramErr,d0
        bra     CtlDone

StGetMode:
        move.w  pMode(a3),csMode(a2)
        clr.w   csPage(a2)
        bsr     GetBase
        move.l  d0,csBaseAddr(a2)
        bra     CtlDone0

StGetPages:
        move.w  csMode(a2),d0
        bsr     CheckMode
        bne     StParamErr
        move.w  #1,csPage(a2)
        bra     CtlDone0

StGetBaseAddr:
        tst.w   csPage(a2)
        bne     StParamErr
        bsr     GetBase
        move.l  d0,csBaseAddr(a2)
        bra     CtlDone0

StGetGray:
        moveq   #1,d0
        and.b   pGray(a3),d0
        move.b  d0,csMode(a2)
        bra     CtlDone0

StGetInterrupt:
        move.b  pIntOff(a3),csMode(a2)
        bra     CtlDone0

StGetGamma:
        lea     pGamma(a3),a0
        move.l  a0,(a2)
        bra     CtlDone0

StGetDefaultMode:
        move.b  pDefMode(a3),csMode(a2)
        bra     CtlDone0

StGetEntries:
        move.l  csTable(a2),d0
        beq     StParamErr
        move.l  d0,a0
        move.w  csStart(a2),d1
        move.w  csCount(a2),d2
        cmp.w   #255,d2
        bhi     StParamErr
        move.l  pRegs(a3),a1
.loop:
        moveq   #0,d3
        move.w  d1,d3
        bpl     .seq
        move.w  (a0),d3
        subq.w  #1,d1
.seq:
        and.w   #$FF,d3
        move.l  d3,REG_CLUT_INDEX(a1)
        move.l  REG_CLUT_DATA(a1),d4    ; $00RRGGBB
        move.w  d3,(a0)                 ; value
        moveq   #2,d5
        lea     8(a0),a0
.comp:
        move.b  d4,-(a0)                ; blue, green, red: $xxxx
        move.b  d4,-(a0)
        lsr.l   #8,d4
        dbra    d5,.comp
        addq.l  #6,a0
        addq.w  #1,d1
        dbra    d2,.loop
        bra     CtlDone0

; ---------------------------------------------------------------------
; Helpers.  A3 = private storage; these keep A2-A4 and D3-D7.

; Z set if D0.w is a valid mode
CheckMode:
        cmp.w   #FIRST_MODE,d0
        blo     .bad
        cmp.w   pMaxMode(a3),d0
        bhi     .bad
        cmp.w   d0,d0
        rts
.bad:
        moveq   #1,d1
        tst.w   d1
        rts

; Set depth D0.w (an spID); load a gray ramp into the CLUT
SetMode:
        movem.l d3-d5,-(sp)
        move.w  d0,pMode(a3)
        sub.w   #FIRST_MODE,d0
        ext.l   d0
        move.l  pRegs(a3),a0
        move.l  d0,REG_MODE(a0)
        cmp.w   #DIRECT_MODE-FIRST_MODE,d0
        bhs     .done
        moveq   #1,d1
        lsl.w   d0,d1                   ; bits per pixel
        moveq   #1,d2
        lsl.w   d1,d2
        subq.w  #1,d2                   ; d2 = last index
        clr.l   REG_CLUT_INDEX(a0)
        moveq   #0,d3
.ramp:
        move.w  d3,d4
        mulu    #255,d4
        divu    d2,d4
        move.l  #255,d5
        sub.w   d4,d5                   ; white first, black last
        move.l  d5,d4
        lsl.l   #8,d4
        or.l    d5,d4
        lsl.l   #8,d4
        or.l    d5,d4
        move.l  d4,REG_CLUT_DATA(a0)
        addq.w  #1,d3
        cmp.w   d2,d3
        bls     .ramp
.done:
        movem.l (sp)+,d3-d5
        rts

; D0 = base address of the current frame: $Fss00000 (good in both
; modes) for frames in the 24-bit window, else $Fs000000 + offset
GetBase:
        move.l  pRegs(a3),a0
        move.l  REG_BASE(a0),d0
        bne     .above
        move.l  a0,d0
        sub.l   #REG_OFFSET,d0
        rts
.above:
        add.l   pDevBase(a3),d0
        rts

; Fill the frame with a 50% dither, in 32-bit mode as large frames lie
; above the 24-bit window
GrayScreen:
        movem.l d3-d7/a2,-(sp)
        moveq   #1,d0                   ; true32b
        _SwapMMUMode
        move.l  d0,-(sp)
        move.l  pRegs(a3),a1
        move.l  pDevBase(a3),a0
        add.l   REG_BASE(a1),a0
        move.l  REG_ROWBYTES(a1),d3
        move.l  REG_HEIGHT(a1),d4
        move.l  REG_MODE(a1),d0
        lsl.w   #2,d0
        move.l  .pats(pc,d0.w),d5
        move.l  d5,d7
        cmp.w   #5*4,d0
        bne     .one
        not.l   d7                      ; 32 bpp: alternate pixel by pixel
.one:
        subq.w  #1,d4
.row:
        move.l  a0,a2
        move.l  d3,d6
        lsr.l   #2,d6
        subq.w  #1,d6
.col:
        move.l  d5,(a2)+
        exg     d5,d7
        dbra    d6,.col
        not.l   d5
        not.l   d7
        add.l   d3,a0
        dbra    d4,.row
        move.l  (sp)+,d0
        _SwapMMUMode
        movem.l (sp)+,d3-d7/a2
        rts
.pats:
        dc.l    $AAAAAAAA, $CCCCCCCC, $F0F0F0F0, $FF00FF00
        dc.l    $00007FFF, $00000000

; Linear gamma table into the private storage
LinearGamma:
        lea     pGamma(a3),a0
        clr.l   (a0)+                   ; gVersion, gType
        move.l  #3,(a0)+                ; gFormulaSize 0, gChanCnt 3
        move.l  #$01000008,(a0)+        ; gDataCnt 256, gDataWidth 8
        moveq   #2,d1
.chan:
        moveq   #0,d0
.val:
        move.b  d0,(a0)+
        addq.b  #1,d0
        bne     .val
        dbra    d1,.chan
        rts

DrvrEnd:

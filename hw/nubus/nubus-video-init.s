; PrimaryInit for the QEMU NuBus video card (hw/nubus/nubus-video.c)
;
; The Monitors control panel saves the chosen depth in the card's slot
; PRAM (first vendor byte, the mode's spID), and at startup the ROM opens
; the card in that mode.  ROMs without 32-bit QuickDraw (Mac II, IIx,
; IIcx, SE/30) cannot set up a 16 or 32 bpp screen, and stop with a
; "memory full" system error.  Like Apple's own cards, fall back to a mode
; the ROM can handle (8 bpp) there; once the System has loaded 32-bit
; QuickDraw it switches back to the saved depth from its 'scrn' resource.
;
; Called by the Slot Manager with A0 = the SEBlock; 68000 code.
;
; Rebuild hw/nubus/nubus-video-init.h after changing this file:
;   vasmm68k_mot -quiet -Fbin -m68000 -o init.bin nubus-video-init.s
;   python3 -I nubus-video-driver.py init.bin nubus_video_init > nubus-video-init.h
;
; SPDX-License-Identifier: GPL-2.0-or-later

DIRECT_MODE     equ     $84             ; spID of the 16 bpp mode
FALLBACK_MODE   equ     $83             ; 8 bpp

; SEBlock
seSlot          equ     0
seStatus        equ     2

; SpBlock
spResult        equ     0
spsPointer      equ     4
spSlot          equ     49
spBlockSize     equ     56

sReadPRAMRec    equ     17
sPutPRAMRec     equ     18

        macro   _GetToolTrapAddress
        dc.w    $A746
        endm
        macro   _SlotManager
        dc.w    $A06E
        endm

PrimaryInit:
        movem.l d0-d3/a0-a2,-(sp)
        move.l  a0,a2                   ; a2 = SEBlock
        move.w  #1,seStatus(a2)

        ; 32-bit QuickDraw in ROM? (_QDExtensions implemented)
        move.w  #$AB1D,d0
        _GetToolTrapAddress
        move.l  a0,d3
        move.w  #$A89F,d0               ; _Unimplemented
        _GetToolTrapAddress
        cmp.l   a0,d3
        bne     .done

        lea     -(spBlockSize+8)(sp),sp
        move.l  sp,a0                   ; a0 = SpBlock
        lea     spBlockSize(sp),a1      ; a1 = the 8-byte sPRAM record
        move.b  seSlot(a2),spSlot(a0)
        move.l  a1,spResult(a0)
        moveq   #sReadPRAMRec,d0
        _SlotManager
        bne     .pop
        cmp.b   #DIRECT_MODE,2(a1)
        bcs     .pop
        move.b  #FALLBACK_MODE,2(a1)
        move.l  a1,spsPointer(a0)
        moveq   #sPutPRAMRec,d0
        _SlotManager
.pop:
        lea     spBlockSize+8(sp),sp
.done:
        movem.l (sp)+,d0-d3/a0-a2
        rts

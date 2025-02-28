;
; File:     main.S
; Target:   PIC16F506
; Author:   dan1138
; Date:     2025-02-21
; Compiler: MPASM v5.51
; IDE:      MPLAB v8.92
;
; Depends on files:
;   <InstallPathVaries>\MPLAB\MPASM Suite\LKR\16f506_g.lkr
;   <InstallPathVaries>\MPLAB\MPASM Suite\p16f506.inc
;
; Description:
;
;   Example project for the PIC16F506 controller using the MPASM(v5.51) tool chain.
;
;  
;                         PIC16F506
;               +------------:_:------------+
;      GND -> 1 : VDD                   VSS : 14 <- 5v0
;  DIGIT_2 <> 2 : RB5/OSC1      PGD/AN0/RB0 : 13 <> PGD/SEG_g
;    SEG_e <> 3 : RB4/OSC2      PGC/AN1/RB1 : 12 <> PGC/SEG_f
;  SW1/VPP -> 4 : RB3/VPP           AN2/RB2 : 11 <> SW2
;  DIGIT_1 <> 5 : RC5                   RC0 : 10 <> DIGIT_3
;    SEG_c <> 6 : RC4                   RC1 : 9  <> SEG_a
;    SEG_d <> 7 : RC3                   RC2 : 8  <> SEG_b
;               +---------------------------:
;                          DIP-14
;
; Test hardware see: https://github.com/gooligumelec/Baseline-PIC-training-board/blob/main/Base-mid_dev_inst.pdf
;  
; See: https://forum.microchip.com/s/topic/a5CV400000032DNMAY/t399990
;
;   This implementation was created from scratch. The Original Posters (OP) code
;   seems chaotic. Just finding how the GPIO pins are mapped to the switch and
;   seven segment LED digit was quite a strange experience.
;
    PROCESSOR   16F506
    LIST        n=0,c=132
    RADIX       DEC
    ERRORLEVEL -302         ; Suppress the not in bank 0 warning
;
; Include target specific definitions for special function registers
;
#include <p16f506.inc>
;
; PIC16F506 Configuration Bit Settings

; Oscillator Selection bits (INTRC With RB4 and 1.125 ms DRT)
; Watchdog Timer Enable bit (WDT disabled)
; Code Protect (Code protection off)
; Master Clear Enable bit (RB3/MCLR pin functions as RB3, MCLR tied internally to VDD)
; Internal Oscillator Frequency Select bit (8 MHz INTOSC Speed)

    __config _IntRC_OSC_RB4EN & _WDT_OFF & _CP_OFF & _MCLRE_OFF & _IOSCFS_ON

;
;
; Declare RAM
;
MainData    UDATA   0x10
    global  Counter
Counter:    RES     1

    global  Delay_t0,Delta_t
Delay_ms:   RES     1
Delay_t0:   RES     1
Delta_t:    RES     1
#define TIMER0_COUNTS_IN_ONE_MILLISECOND (125)

    global  SW_BounceCount,SW_FLAGS
SW_BounceCount: RES 1
SW_FLAGS:       RES 1
SW_TEMP:        RES 1
#define SW_FLAGS_SW2_STABLE_POSITION 0
#define SW_FLAGS_SW1_STABLE_POSITION 1
#define SW_FLAGS_SW2_SAMPLE_POSITION 2
#define SW_FLAGS_SW1_SAMPLE_POSITION 3
#define SW_FLAGS_SW2_CHANGE_POSITION 4
#define SW_FLAGS_SW1_CHANGE_POSITION 5
#define SW_FLAGS_SAMPLE_MASK ((1<<SW_FLAGS_SW1_SAMPLE_POSITION)|(1<<SW_FLAGS_SW2_SAMPLE_POSITION))
#define SW_FLAGS_STABLE_MASK ((1<<SW_FLAGS_SW1_STABLE_POSITION)|(1<<SW_FLAGS_SW2_STABLE_POSITION))
#define SW_FLAGS_CHANGE_MASK ((1<<SW_FLAGS_SW1_CHANGE_POSITION)|(1<<SW_FLAGS_SW2_CHANGE_POSITION))
#define SW_BOUNCE_TIME (20)

    global  LED_Segments
LED_Segments:   RES 1
#define PORTC_SEG_MASK   (B'00011110')
#define PORTC_SEG_a_MASK (B'00000010')
#define PORTC_SEG_b_MASK (B'00000100')
#define PORTC_SEG_c_MASK (B'00010000')
#define PORTC_SEG_d_MASK (B'00001000')
#define PORTB_SEG_MASK   (B'00010011')
#define PORTB_SEG_e_MASK (B'00010000')
#define PORTB_SEG_f_MASK (B'00000010')
#define PORTB_SEG_g_MASK (B'00000001')
;
; Power-On-Reset entry with WREG loaded with factory oscillator calibration
;
StartCode   CODE    0x000
    global  Start
Start:
    movwf   OSCCAL                          ; set factory oscillator calibration
    clrf    CM1CON0                         ; make PORTC digital I/O
    clrf    CM2CON0                         ; make PORTC digital I/O
    bsf     CM1CON0,NOT_C1WU                ; make PORTC digital I/O
    bsf     CM2CON0,NOT_C2WU                ; make PORTC digital I/O
    bcf     ADCON0,       ANS0              ; make PORTB digital I/O
    bcf     ADCON0,       ANS1              ; make PORTB digital I/O

    movlw   0x0C        ; PORTB, bits 3,2 are inputs.
    tris    PORTB       ; PORTB, bits 5,4,1,0 are outputs.

    clrw
    tris    PORTC       ; PORTC, bits 5,4,3,2,1,0 are outputs.

    movlw   0xC3        ; TIMER0 clock Fosc/4, prescale 1:16, counts at 125KHz with 8MHz system oscillator
    option

    clrf    PORTB
    clrf    PORTC

    banksel Counter
    clrf    Counter
    clrf    SW_BounceCount
    movf    PORTB,W
    andlw   SW_FLAGS_SAMPLE_MASK
    movwf   SW_FLAGS

    goto    AppLoop
;
; This delay function will spin for 1 to 256 milliseconds.
; TIMER0 is used to count elapse time. This is tricky to do
; with a baseline controller like a PIC16F506 as there are 
; no opcodes that can add or subtract constants from the WREG.
;
; Input:    WREG (delay 1 to 256 milliseconds)
; Output:   none
; Uses:     WREG
;           Delay_ms
;           Delay_t0
;           Delta_t
; Returns:  WREG set to zero
;
Delay:
    movwf   Delay_ms        ; Save number of milliseconds to delay
    movf    TMR0,W          ; Sample the TIMER0 count
    movwf   Delay_t0        ; At start of delay
    movlw   TIMER0_COUNTS_IN_ONE_MILLISECOND
    movwf   Delta_t
Delay_loop:
    movf    Delay_t0,W      ; Find the numer of TIMER0 counts
    subwf   TMR0,W          ; since last sample.
    subwf   Delta_t,W       ; Check if TIMER0 has counted enough.
    skpnc
    goto    Delay_loop      ; Loop if not long enough.
    movf    Delta_t,W
    addwf   Delay_t0,F      ; Adjust for next delay loop.
    decfsz  Delay_ms,F      ; Decrement delay count
    goto    Delay_loop      ; Loop if not delay enough.
    retlw   0               ; Exit delay
;
; Sample switch input and debounce.
;
; Returns:  WREG set to zero
;
SW_Poll:
    movf    PORTB,W
    xorwf   SW_FLAGS,W
    andlw   SW_FLAGS_SAMPLE_MASK
    skpnz                   ; Skip when SW changed
    goto    SW_Same
    xorwf   SW_FLAGS,F      ; Update SW sample
    movlw   SW_BOUNCE_TIME
    movwf   SW_BounceCount  ; Start debounce time
    retlw   0

SW_Same:
    movf    SW_BounceCount,F
    skpnz                   ; skip if SW has not been stable long enough
    goto    SW_Stable
    decf    SW_BounceCount,F
    retlw   0

SW_Stable:
    movf    SW_FLAGS,W
    btfsc   SW_FLAGS,SW_FLAGS_SW1_SAMPLE_POSITION
    xorlw   (1<<SW_FLAGS_SW1_STABLE_POSITION)
    btfsc   SW_FLAGS,SW_FLAGS_SW2_SAMPLE_POSITION
    xorlw   (1<<SW_FLAGS_SW2_STABLE_POSITION)
    andlw   SW_FLAGS_STABLE_MASK
    skpnz
    retlw   0
    xorwf   SW_FLAGS,F
    movwf   SW_TEMP
    swapf   SW_TEMP,W
    iorwf   SW_FLAGS,F
    retlw   0
;
; Function: LookupSegemnts
;
; Input:  WREG - low 4-bits for a HEX digit
;
; Output: WREG - 7-segment mask for LED digit
;
LookupSegemnts:
    andlw   0x0F
    addwf   PCL,F
;             .gfedcba 
    retlw   B'00111111' ; mask for digit 0
    retlw   B'00000110' ; mask for digit 1
    retlw   B'01011011' ; mask for digit 2
    retlw   B'01001111' ; mask for digit 3
    retlw   B'01100110' ; mask for digit 4
    retlw   B'01101101' ; mask for digit 5
    retlw   B'01111101' ; mask for digit 6
    retlw   B'00000111' ; mask for digit 7
    retlw   B'01111111' ; mask for digit 8
    retlw   B'01101111' ; mask for digit 9
    retlw   B'01110111' ; mask for digit A
    retlw   B'01111100' ; mask for digit b
    retlw   B'00111001' ; mask for digit C
    retlw   B'01011110' ; mask for digit d
    retlw   B'01111001' ; mask for digit E
    retlw   B'01110001' ; mask for digit F
;
; Function: Show_digit
;
; Input:  LED_Segments show the segments to be turned on.
;
; Output: none
;
; Changes output bits on PORTB and PORTC
;
; Returns: WREG set to zero
;
Show_digit:
    movlw   0
    btfsc   LED_Segments,0  ; SEG_a
    iorlw   PORTC_SEG_a_MASK
    btfsc   LED_Segments,1  ; SEG_b
    iorlw   PORTC_SEG_b_MASK
    btfsc   LED_Segments,2  ; SEG_c
    iorlw   PORTC_SEG_c_MASK
    btfsc   LED_Segments,3  ; SEG_d
    iorlw   PORTC_SEG_d_MASK
    xorwf   PORTC,W
    andlw   PORTC_SEG_MASK
    xorwf   PORTC,F

    movlw   0
    btfsc   LED_Segments,4  ; SEG_e
    iorlw   PORTB_SEG_e_MASK
    btfsc   LED_Segments,5  ; SEG_f
    iorlw   PORTB_SEG_f_MASK
    btfsc   LED_Segments,6  ; SEG_g
    iorlw   PORTB_SEG_g_MASK
    xorwf   PORTB,W
    andlw   PORTB_SEG_MASK
    xorwf   PORTB,F

    retlw   0
;
; Do something when SW1 is pressed
;
; Returns:  WREG set to zero
;
SW1_ChangedToAsserted:
    movlw   1
    addwf   Counter,F
    movlw   10
    subwf   Counter,W
    skpnc
    clrf    Counter
    retlw   0
;
; Do something when SW1 is released
;
; Returns:  WREG set to zero
;
SW1_ChangedToReleased:
    retlw   0
;
; Do something when SW2 is pressed
;
; Returns:  WREG set to zero
;
SW2_ChangedToAsserted:
    movlw   -1
    addwf   Counter,F
    movlw   9
    skpc
    movwf   Counter
    retlw   0
;
; Do something when SW2 is released
;
; Returns:  WREG set to zero
;
SW2_ChangedToReleased:
    retlw   0
;
; Application loop
;
AppLoop:
    movlw   1
    call    Delay
    call    SW_Poll

    movf    SW_FLAGS,W
    andlw   SW_FLAGS_CHANGE_MASK
    skpnz
    goto    AppLoop
;
; Check if SW1 changed
    btfss   SW_FLAGS,SW_FLAGS_SW1_CHANGE_POSITION
    goto    CheckSW1_Exit
    btfss   SW_FLAGS,SW_FLAGS_SW1_STABLE_POSITION
    call    SW1_ChangedToAsserted
    btfsc   SW_FLAGS,SW_FLAGS_SW1_STABLE_POSITION
    call    SW1_ChangedToReleased
    bcf     SW_FLAGS,SW_FLAGS_SW1_CHANGE_POSITION
CheckSW1_Exit:
;
; Check if SW2 changed
    btfss   SW_FLAGS,SW_FLAGS_SW2_CHANGE_POSITION
    goto    CheckSW2_Exit
    btfss   SW_FLAGS,SW_FLAGS_SW2_STABLE_POSITION
    call    SW2_ChangedToAsserted
    btfsc   SW_FLAGS,SW_FLAGS_SW2_STABLE_POSITION
    call    SW2_ChangedToReleased
    bcf     SW_FLAGS,SW_FLAGS_SW2_CHANGE_POSITION
CheckSW2_Exit:

    movf    Counter,W
    call    LookupSegemnts
    movwf   LED_Segments

    call    Show_digit

    goto    AppLoop
;
; In the PIC16F506 the factory calibration for
; the on chip oscillator is stored as a MOVLW 0xnn
; opcode as the last instruction of code space.
; On RESET the PC is set to all ones and this
; opcode is executed, then the PC rolls over
; to zero and the first opcode of the application
; will execute.
;
OscCalVec   CODE    0x3FF
    global  OscCal
OscCal:

    end
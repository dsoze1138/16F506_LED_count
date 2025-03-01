;
; File:     main.S
; Target:   PIC16F506
; Author:   dan1138
; Date:     2025-02-21
; Compiler: pic-as(v3.00)
; IDE:      MPLABX v6.20
;
; Depends on files:
;   <InstallPathVaries>\xc8\v3.00\pic\include\xc.inc
;   <InstallPathVaries>\xc8\v3.00\pic\include\pic.inc
;   <InstallPathVaries>\xc8\v3.00\pic\include\pic_as_chip_select.inc
;   <InstallPathVaries>\xc8\v3.00\pic\include\proc\pic16f506.inc
;
; Description:
;
;   Example project for the PIC16F506 controller using the pic-as(v3.00) tool chain.
;
; Add this line in the project properties box "pic-as Global Options -> Additional options": 
;
;   -Wa,-a -Wl,-DCODE=2,-pStartCode=0h,-pOscCalVec=3ffh
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
    PAGEWIDTH   132
    RADIX       DEC

; PIC16F506 Configuration Bit Settings

; CONFIG
  CONFIG  OSC = IntRC_RB4EN     ; Oscillator Selection bits (INTRC With RB4 and 1.125 ms DRT)
  CONFIG  WDT = OFF             ; Watchdog Timer Enable bit (WDT disabled)
  CONFIG  CP = OFF              ; Code Protect (Code protection off)
  CONFIG  MCLRE = OFF           ; Master Clear Enable bit (RB3/MCLR pin functions as RB3, MCLR tied internally to VDD)
  CONFIG  IOSCFS = ON           ; Internal Oscillator Frequency Select bit (8 MHz INTOSC Speed)
;
; Include target specific definitions for special function registers
;
#include <xc.inc>
;
;
; Declare RAM
;
    PSECT   MainData,class=RAM,space=SPACE_DATA,noexec
    global  Counter
Counter:    DS      1

    global  Delay_t0,Delta_t
Delay_ms:   DS      1
Delay_t0:   DS      1
Delta_t:    DS      1
#define TIMER0_COUNTS_IN_ONE_MILLISECOND (125)

    global  SW_BounceCount,SW_FLAGS
SW_BounceCount: DS  1
SW_FLAGS:       DS  1
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
LED_Segments:   DS  1
#define PORTC_SEG_MASK   (0b00011110)
#define PORTC_SEG_a_MASK (0b00000010)
#define PORTC_SEG_b_MASK (0b00000100)
#define PORTC_SEG_c_MASK (0b00010000)
#define PORTC_SEG_d_MASK (0b00001000)
#define PORTB_SEG_MASK   (0b00010011)
#define PORTB_SEG_e_MASK (0b00010000)
#define PORTB_SEG_f_MASK (0b00000010)
#define PORTB_SEG_g_MASK (0b00000001)
;
; Power-On-Reset entry with WREG loaded with factory oscillator calibration
;
    PSECT   StartCode,class=CODE,space=SPACE_CODE,delta=2
    global  Start
Start:
    movwf   OSCCAL                          ; set factory oscillator calibration
    clrf    CM1CON0                         ; make PORTC digital I/O
    clrf    CM2CON0                         ; make PORTC digital I/O
    bsf     CM1CON0,CM1CON0_nC1WU_POSITION  ; make PORTC digital I/O
    bsf     CM2CON0,CM2CON0_nC2WU_POSITION  ; make PORTC digital I/O
    bcf     ADCON0,ADCON0_ANS0_POSITION     ; make PORTB digital I/O
    bcf     ADCON0,ADCON0_ANS1_POSITION     ; make PORTB digital I/O

    movlw   0b00001100  ; PORTB, bits 3,2 are inputs.
    nop     ; MPLABX simulator bug workaround
    tris    PORTB       ; PORTB, bits 5,4,1,0 are outputs.

    movlw   0b00000000
    nop     ; MPLABX simulator bug workaround
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
    btfsc   STATUS,STATUS_C_POSITION
    goto    Delay_loop      ; Loop if not long enough.
    movf    Delta_t,W
    addwf   Delay_t0,F      ; Adjust for next delay loop.
    decfsz  Delay_ms        ; Decrement delay count
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
    btfsc   STATUS,STATUS_Z_POSITION ; Skip when SW changed
    goto    SW_Same
    xorwf   SW_FLAGS,F      ; Update SW sample
    movlw   SW_BOUNCE_TIME
    movwf   SW_BounceCount  ; Start debounce time
    retlw   0

SW_Same:
    movf    SW_BounceCount,F
    btfsc   STATUS,STATUS_Z_POSITION ; skip if SW has not been stable long enough
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
    btfsc   STATUS,STATUS_Z_POSITION
    retlw   0
    xorwf   SW_FLAGS,F
    swapf   SW_FLAGS,F
    iorwf   SW_FLAGS,F      ; update changed flags
    swapf   SW_FLAGS,F
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
    retlw   0b00111111  ; mask for digit 0
    retlw   0b00000110  ; mask for digit 1
    retlw   0b01011011  ; mask for digit 2
    retlw   0b01001111  ; mask for digit 3
    retlw   0b01100110  ; mask for digit 4
    retlw   0b01101101  ; mask for digit 5
    retlw   0b01111101  ; mask for digit 6
    retlw   0b00000111  ; mask for digit 7
    retlw   0b01111111  ; mask for digit 8
    retlw   0b01101111  ; mask for digit 9
    retlw   0b01110111  ; mask for digit A
    retlw   0b01111100  ; mask for digit b
    retlw   0b00111001  ; mask for digit C
    retlw   0b01011110  ; mask for digit d
    retlw   0b01111001  ; mask for digit E
    retlw   0b01110001  ; mask for digit F
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
    btfsc   STATUS,STATUS_C_POSITION
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
    btfss   STATUS,STATUS_C_POSITION
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
    btfsc   STATUS,STATUS_Z_POSITION
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
    PSECT   OscCalVec,class=CODE,delta=2
    global  OscCal
OscCal:

    end     OscCal
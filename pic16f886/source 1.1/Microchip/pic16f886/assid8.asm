; ASSID [acid] Asynchronous Serial SID Interface Device
; - A 500kbps serial interface for the MOS 6501 SID
;   senseitg@hotmail.com
	
;  V1.1 (read-fix) by gh0stless 2018
;  gh0stless@t-online.de

; Designed for MPASM

; To attain the highest possible speed, the SID is driven by a timer
; set up to generate the 1MHz signal to the SID. Since the SID requires
; that read/write operations are aligned to the clock, the code loop is
; designed to always run an exact multiple of clock cycles thus keeping
; the code synchronized to the timer output.

	list			p=16f886
	radix			dec
	#include		p16f886.inc

; Configuration bits
;   INTOSCIO  - allows power up without 12MHz clock from FTDI
;	WTD_OFF   - watchdog is not kicked by code
;   MCLRE_OFF - else will require jumper on programming header
;   LVP_OFF   - frees up RB3 which is very much required
	__CONFIG _CONFIG1, _INTOSCIO & _WDT_OFF & _MCLRE_OFF & _LVP_OFF

SID_CTL	    EQU		PORTC			;SID bus control port
SID_ADDR    EQU		PORTA			;SID bus address port
SID_DATA    EQU		PORTB			;SID bus data port
SID_DDR	    EQU		TRISB			;SID bus data direction

CTL_LED     EQU		3

CTL_CLK     EQU		2			;SID Ø2  on SID_CTL
CTL_RW      EQU		1			;SID R/W on SID_CTL
CTL_CS      EQU		0			;SID CS  on SID_CTL

ADDR_SYN    EQU		7			;SYN     on SID_ADDR
ADDR_DDR    EQU		6			;DDR	 on SID_ADDR
ADDR_RST    EQU		5			;SID RST on SID_ADDR

TEMP	    EQU		0x70			;Temp memory
	
	ORG	0

	GOTO	__INIT

	ORG	4
__ISR
	RETFIE

__INIT


	BTFSC   INTCON,     GIE

	;Disable analog	
	BSF		STATUS,		RP0
	BSF		STATUS,		RP1
	CLRF	ANSEL					;ANSEL   <- 0b00000000
	CLRF	ANSELH					;ANSELH  <- 0b00000000

	;Preset data
	BCF		STATUS,		RP0
	BCF		STATUS,		RP1
	CLRF	PORTA					;PORTA   <- 0b00000000
	CLRF	PORTB					;PORTB   <- 0b00000000
	MOVLW	0x01
	MOVWF	PORTC					;PORTC   <- 0b00000001

	;Enable outputs
	BSF		STATUS,		RP0
	MOVLW	0xC0
	MOVWF	TRISA					;TRISA   <- 0b11000000: <0:4>:address <5>:rst
	MOVLW	0xFF
	MOVWF	TRISB					;TRISB   <- 0b11111111: <0:7>:data
	MOVLW	0xB0
	MOVWF	TRISC					;TRISC   <- 0b10110000: <0>:cs <1>:rw <2>:clk <3> led <6>:tx <7>:rx
	
	;Set up T1/PWM output (SID_CLK)
	MOVLW	0x77
	MOVWF	OSCCON					;Switch to 8MHz

	MOVLW	0x04
	BCF		STATUS,		RP0
	MOVWF	T2CON					;T2CON   <- 0b00000100:	T2 = sysclk
	MOVLW	0x01
	BSF		STATUS,		RP0
	MOVWF	PR2					;PR2     <- 0b00000001: reload at 1 = 2 cycles
	MOVLW	0x01
	BCF		STATUS,		RP0
	MOVWF	CCPR1L					;CCPR1L  <- 0b00000001:	pwm duty 50%
	MOVLW	0x0F
	MOVWF	CCP1CON					;CCP1CON <- 0b00001111: pwm enable @P1A/SID_CLK

	;UART setup
	BSF		STATUS,		RP0
	BSF		STATUS,		RP1
	MOVLW   0x48
	MOVWF   BAUDCTL					;BAUDCTL <- 0b01001000: Use 16bit BRG
	BCF		STATUS,		RP1
	;CLRF	SPBRG					;SPBRG   <- 0b00000000: 2000kbps @ 8Mhz
	MOVLW   3						
	MOVWF	SPBRG					;SPBRG   <- 0b00000011: 500kbps @ 8Mhz
	MOVLW	0x24					
	MOVWF	TXSTA					;TXSTA   <- 0b00100100: Enable transmitter
	BCF		STATUS,		RP0
	MOVLW	0x90
	MOVWF	RCSTA					;RCSTA   <- 0b10010000: Enable receiver

	NOP

__MAIN
	;@00:0 Wait for data
wait_for_address
	NOP
	BTFSS	PIR1,		RCIF
	GOTO	wait_for_address
	NOP

	;@04:0 Copy received byte to address port
	MOVF	RCREG,		W
	MOVWF	PORTA

	;@06:2 Ensure sync bit set - helps fix sync errors
	MOVWF	TEMP
	BTFSS	TEMP,		ADDR_SYN
	GOTO    wait_for_address

		
	;@12:0 Requesting read or write?
	BTFSC	TEMP,		ADDR_DDR
	GOTO    __WRITE_REGISTER

__READ_REGISTER
	;@18:2 Pre-align
	NOP
	;@14:2 Set data port to input
	BSF		STATUS,		RP0
	MOVLW	0xFF
	MOVWF	SID_DDR
	BCF		STATUS, 	RP0
		
	;Disable analog	
	BSF		STATUS,		RP0
	BSF		STATUS,		RP1
	CLRF		ANSELH					;ANSELH  <- 0b00000000
	BCF		STATUS,		RP0
	BCF		STATUS,		RP1
	
	
	;@18:2 SID transfer
	BSF		SID_CTL,	CTL_RW
	BCF		SID_CTL,	CTL_CS
	NOP
	
	MOVF	SID_DATA,	W
		
	;@23:3 Send
	MOVWF	TXREG
	
	BSF		SID_CTL,	CTL_CS
	BCF		SID_CTL,	CTL_RW

	;@24:0 Post-align
	NOP

	;@26:2
	GOTO	__MAIN

__WRITE_REGISTER

	BSF     SID_CTL,	CTL_LED

	;@15:3 Wait for data byte
wait_for_data
	NOP
	BTFSS	PIR1,		RCIF
	GOTO	wait_for_data

	;@18:2 Set data port to output
	BSF		STATUS,		RP0
	CLRF	SID_DDR
	BCF		STATUS, 	RP0
	
	;@21:1 Copy received byte to data port
	MOVF	RCREG,		W
	MOVWF	PORTB

	;@23:3 Pre-align

	;@23:3 SID transfer
	BCF		SID_CTL,	CTL_RW
	BCF		SID_CTL,	CTL_CS
	NOP
	BSF		SID_CTL,	CTL_CS

	BCF     SID_CTL,	CTL_LED

	;@27:3 Post-align

	;@30:2
	GOTO	__MAIN

__END
	END
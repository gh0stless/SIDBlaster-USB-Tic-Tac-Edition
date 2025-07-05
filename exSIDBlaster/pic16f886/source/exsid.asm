; Firmware version 11 for exSID USB Rev.B
; (C) 2015-2016 T. Varene
; License: CC BY-NC-SA 4.0

; Designed for MPASM
; This program is optimized for timing accuracy, speed and size, in that order.
; Changelog at the bottom of the file.

; SIDs are driven by a timer set up to generate the 1MHz square signal. 
; Since the SID requires that read/write operations are aligned to the clock,
; the code is designed to always run an exact multiple of SID clock cycles thus
; keeping the code synchronized to the timer output.
	
; NOTE: While the SID will apparently respond to rising clock edge, some voices
; may not properly work that way. Thus clocking on falling edge (per datasheet)
; appears to be more reliable.

; This program assumes full operation within Page 0 (compatible with PIC16F722/882)
; PCLATH is preset in init routine assuming PCL is directly modified only in the address jump table
	
; This program was initially written for PIC16F88X and later adapted to PIC16F72X
; PIC16F72X support is only for "educational" purposes and is not actively supported.

	title		"exSID rev.C Firmware v11"
	
#define	FWVERS		.11
#define HWVERS		"C"


#ifdef	__16F882
	processor	16f882
	list		p=16f882
 #include		p16f882.inc
 #define		__PIC16F88X
#endif

#ifdef	__16F883
	processor	16f883
	list		p=16f883
 #include		p16f883.inc
 #define		__PIC16F88X
#endif


#ifdef	__16F886
	processor	16f886
	list		p=16f886
 #include		p16f886.inc
 #define		__PIC16F88X
#endif

; adding non-A variants left as an exercise
#ifdef	__16F722A
	processor	16f722a
	list		p=16f722a
 #include		p16f722a.inc
 #define		__PIC16F72X
#endif

#ifdef	__16F723A
	processor	16f723a
	list		p=16f723a
 #include		p16f722a.inc
 #define		__PIC16F72X
#endif

#ifdef	__16F726
	processor	16f726
	list		p=16f726
 #include		p16f726.inc
 #define		__PIC16F72X
#endif

#ifdef	__PIC16F88X
; Configuration bits
;   FOSC_EC	- enables external 24MHz clock from FTDI
;   WTDE_OFF	- watchdog is not kicked by code
;   PWRTE_ON	- power-on timer enabled
;   MCLRE_ON	- /MCLR enabled for delayed boot after secondary power rises
;   CP_OFF	- No code protection
;   CPD_OFF	- No data code protection
;   BOREN_ON	- Brown-out reset enabled
;   IESO_OFF	- Internal/external clock switchover disabled
;   FCMEN_OFF	- Fail-safe clock monitor disabled
;   LVP_OFF	- LVP disabled: frees up RB3
;   BOR40V	- Brown-out at 4.0V
;   WRT_OFF	- Write protection off
	__CONFIG _CONFIG1, _FOSC_EC & _WDTE_OFF & _PWRTE_ON & _MCLRE_ON & _CP_OFF & _CPD_OFF & _BOREN_ON & _IESO_OFF & _FCMEN_OFF & _LVP_OFF
	__CONFIG _CONFIG2, _BOR4V_BOR40V & _WRT_OFF
	
 #define SETHIBDRT	; if defined: 2Mbps, 1-cycle resolution, else 750kbps, 2-cycle resolution
#endif	;__PIC16F88X


#ifdef	__PIC16F72X
	;   BORV_25	- Brown-out at 2.5V
	__CONFIG _CONFIG1, _FOSC_EC & _WDTE_OFF & _PWRTE_ON & _MCLRE_ON & _CP_OFF & _BOREN_ON & _BORV_25 & _PLLEN_ON
 #if HWVERS == "B"
	__CONFIG _CONFIG2, _VCAPEN_RA6
 #else	; No support for VCAP on other revisions
	__CONFIG _CONFIG2, _VCAPEN_DIS
 #endif	;HWVERS

 #undefine SETHIBDRT	; 72X family can only do 750kpbs
 
 #define	ANSEL	ANSELA
 #define	ANSELH	ANSELB
 #define	EECON1	PMCON1
 #define	EEDAT	PMDATL
 #define	EEDATH	PMDATH
 #define	EEADR	PMADRL
 #define	EEADRH	PMADRH
#endif	;__PIC16F72X

#define	SIDCTL		PORTC		; SID bus control port
#define	SIDADR		PORTA		; SID bus address port
#define	SIDDAT		PORTB		; SID bus data port
#define	SIDDDR		TRISB		; SID bus data direction

#define	EXTCLK		RA7

#if HWVERS == "C"
 #define	CTLCS1		RC0
 #define	CTL_RW		RC1
 #define	CTLCS0		RC3
 #define	CTLRST		RC4
#else	; not "C"
 #if HWVERS == "B"
  #define	CTL_RW		RC0		; SID R/W on SIDCTL
  #define	CTLCS0		RC1		; SID CS0 (6581) on SIDCTL
 #else ; "A"
  #define	CTLCS0		RC0		; SID CS0 (6581) on SIDCTL
  #define	CTL_RW		RC1		; SID R/W on SIDCTL
 #endif	; HWVERS == "B"
 #define	CTLRST		RC3		; SID RST on SIDCTL
 #define	CTLCS1		RC5		; SID CS1 (8580) on SIDCTL
#endif	; HWVERS == "C"
#define	CTLCLK		RC2		; SID CLK on SIDCTL
#define	FTDIRX		RC6
#define	FTDITX		RC7
	
; By default RW is held low. The read routine must bring it back low after exec.
; Chip select bitmask.
#define	CSBBITM		(B'1' << CTLCS0 | B'1' << CTLCS1)
#define	CS0BITM		(B'1' << CTLCS0)
#define	CS1BITM		(B'1' << CTLCS1)

#define	JMPTBA		0x0700		; Jump table base address. 0x07xx stays in Page 0 and ensures no screw up with GOTO/CALL

; Cross-bank data registers
	CBLOCK	0x70
TEMPBUF					; typically holds current address
WAITCNT:2				; SID clock loop counter
CSENCTL					; CS Enable bits
CSDICTL					; CS Disable bits
LPMBYTE					; Pgm memory LSB
HPMBYTE					; Pgm memory MSB
	ENDC


; Program
	ORG	0x0
__INIT

	BANKSEL	OPTION_REG
	MOVLW	B'00000000'		; OPTION bits, nothing fancy
	MOVWF	OPTION_REG

	; Disable interrupts
	BANKSEL INTCON
	CLRF	INTCON			; Disable interrupts and clear IFs
	CLRF	PIE1			; Disable peripheral interrupts
	CLRF	PIE2			; Disable peripheral interrupts

	; Disable analog	
	BANKSEL	ANSEL
	CLRF	ANSEL			; Clear ANSEL
	CLRF	ANSELH			; Clear ANSELH
	
	; Preset data
	BANKSEL PORTA
	CLRF	SIDADR			; Clear SID address
	CLRF	SIDDAT			; Clear SID data
	MOVLW	CSBBITM			; disable both sids as initial condition
	MOVWF	SIDCTL			; Clear SID control except CS0/1 (preset high). This enables SID /reset
	
	; By default we enable 'stereo' operation
	MOVLW	~CSBBITM
	MOVWF	CSENCTL			; CS enable bits
	MOVLW	CSBBITM
	MOVWF	CSDICTL			; CS disable bits
	
	MOVLW	high JMPTBA
	MOVWF	PCLATH			; Preset PCLATH for jump table == MSB of JMPTBA
	
	MOVLW	.20
	MOVWF	WAITCNT			; Initial reset time
	
	; Enable outputs
	BANKSEL	TRISA
	MOVLW	B'1' << EXTCLK
	MOVWF	TRISA			; TRISA   <0:4>:address <7>:clk12 [in]
	CLRF	TRISB			; TRISB   <0:7>:data
	MOVLW	B'1' << FTDITX
	MOVWF	TRISC			; TRISC   <0>:cs0 <1>:rw <2>:clk <3>: rst <5>:cs1 <6>:tx <7>:rx [in]
	
	; UART setup: In high baudrate, at 2Mpbs, chars arrive at max 200kHz (2000/(1+8+1)).
	; We have exactly CYCCHR=5 SID cycles between chars ("CYCCHR"), 10 cycles between tuples.
	; We have a max of 100kHz SID instruction rate (address + data)
	; @24MHz FOSC, we have 30 PIC cycles between chars.
	; In low baudrate config, at 750kbps words arrive at 75kHz, we have just over
	; 13 SID cycles between words (13.33), and 37.5kHz instruction rate.
	; For reference, C64 scanline is < 16kHz
	; This also means that since we don't have flow control, routines must
	; ensure that the receive buffer will always be emptied in a timely fashion,
	; i.e. no more than 2*CYCCHR SID cycles will pass between two fetches.
	; NB: since at 750kbps words timings aren't exact multiples of SID clock cycles,
	; this has a negative impact on timing accuracy with a drift accumulating over time.
	; it can be compensated, preferably in host software, by adjusting every 3 SIDclk.
	; NOTE: The PIC FIFO buffering allows reception of two complete characters and
	; the start of a third character before software must start servicing the EUSART receiver.
#ifdef __PIC16F88X
	BANKSEL	BAUDCTL
 #ifdef SETHIBDRT
	MOVLW   B'01001000'		; BAUDCTL: Use 16bit BRG
 #else
	MOVLW   B'01000000'		; BAUDCTL: Use 8bit BRG
 #endif
	MOVWF   BAUDCTL
#endif ;__PIC16F88X
	BANKSEL SPBRG
#ifdef __PIC16F88X
	CLRF	SPBRGH
 #ifdef SETHIBDRT
	MOVLW   .2			; 2Mbps 0% error @24MHz on 16F88X
 #else
	MOVLW   .1			; 750kbps 0% error @24MHz on 16F88X
 #endif
#else ;__PIC16F72X
	MOVLW	.1			; 750kbps 0% error @24MHz on 16F7XX
#endif
	MOVWF	SPBRG
	MOVLW	B'00100100'					
	MOVWF	TXSTA			; TXSTA: Enable transmitter
	BANKSEL RCSTA
	MOVLW	B'10010000'
	MOVWF	RCSTA			; RCSTA: Enable receiver


	; Set up T1/PWM output (SIDCLK)
	; * PWM registers configuration
	; * Fosc = 24000000 Hz
	; * Fpwm = 1000000.00 Hz
	; * Duty Cycle = 50 %
	; * Resolution is 4 bits
	; * Prescaler is 1
	; */
	;PR2 = 0b00000101 ;
	;T2CON = 0b00000100 ;
	;CCPR1L = 0b00000011 ;
	;CCP1CON = 0b00001100 ;

	BANKSEL	TRISC		; BANK 1
	BCF	TRISC, RC2	; Make sure pin is an output
	MOVLW	B'00000101'
	MOVWF	PR2
	
	BANKSEL	CCP1CON		; BANK 0
	CLRF	CCP1CON		; CCP Module is off
	CLRF	CCP2CON		; for extra safety, manually disable CCP2 as well
	MOVLW	B'00000011'
	MOVWF	CCPR1L		; Duty Cycle is 50% of PWM Period
	MOVLW	B'00001100'	; PWM mode, 2 LSbs of Duty cycle = 00
	MOVWF	CCP1CON
	CLRF	PIR1		; Clear peripheral interrupts Flags
	CLRF	TMR2		; Clear Timer2
	MOVLW	B'00000100'
	MOVWF	T2CON		; Timer2 starts to increment, SIDCLK rise

	; Timing SIDCLKs from here
	; The instruction executing on SIDCLK front is 1/4 cycle ahead of the actual edge.
	; @24MHz, that's 40ns

	; This NOP can be used as a global offset on timings: Clock edges happen 1 INSN earlier
;	NOP

	BSF	SIDCTL,	CTLRST	; Clear reset condition before playing the startup tune
	; start address must be set before the jump
	BCF	STATUS,	RP0
	BSF	STATUS, RP1	; BANK 2, SC low
	
	MOVLW	high __TUNEDATA
	MOVWF	EEADRH
	MOVLW	low __TUNEDATA
	
	MOVWF	EEADR		
	CALL	__INITTUNE	; SC low
	
sid_reset	; 1 SIDCLK loop: SID /RST for WAITCNT SIDCLKs (min 10 per datasheet)
	BCF	STATUS,	RP0
	BCF	STATUS,	RP1	; Back in BANK 0, because __INITTUNE returns from BANK 2
	BCF	SIDCTL,	CTLRST	; Re-enable reset

	DECFSZ	WAITCNT,F
	GOTO	sid_reset
	BSF	SIDCTL,	CTLRST	; SIDCLK. Clear reset condition.
	
	; WARN: We are in BANK 0 from now on: the rest of the code assumes this
	
	; @24MHz: INSNCLK = 6MHz => INSN period = 166ns | 1 SIDCLK = 6 cycles = 1000ns/1MHz
	
__MAIN
	; We service incoming serial data via polling to ensure proper SIDCLK timing
	GOTO	$+1
	GOTO	$+1
	GOTO	__GET_ADDR	; SIDCLK
	
; Long delayed data write
; When called, WAITCNT contains 0000ddd0, ddd == delay in SIDCLKs, non-null
; Execution time: CYCCHR + WAITCNT SIDCLKs max. Write is effective after CYCCHR+WAITCNT SIDCLKs
; WAITCNT must NOT exceed CYCCHR: maximum execution time after clearing RCREG MUST BE <= CYCCHR
__WRITE_REGD	
wrd_data	; Wait for data byte
	GOTO	$+1
	NOP
	
	BTFSS	PIR1,	RCIF
	GOTO	wrd_data
	MOVF	RCREG,	W				; SIDCLK

wrd_waitone
#ifdef SETHIBDRT	; WAITCNT is twice the desired value, decrement 2-by-2 on each loop
	DECF	WAITCNT,F
#else
	NOP
#endif
	NOP
	MOVWF	SIDDAT	; Done here to compensate shifted GOTO
	
	DECFSZ	WAITCNT,F
	GOTO	wrd_waitone
							; SIDCLK
	GOTO	__wri	; this GOTO will be 1 cycle late, so we shift inside __WRITE_REGI		

	
; Refer to SID datasheet for timing considerations

; Immediate data write
; SID writes are latched on falling SIDCLK edge
; Execution time: CYCCHR + 2 SIDCLKs max
__WRITE_REGI
wri_data	; Wait for data byte
	GOTO	$+1		; 1 op / 2 cycles
	NOP			; 1 op / 1 cycle
	
	BTFSS	PIR1,	RCIF	; 1 op / 1 cycle until branch, 2 cycles when branch
	GOTO	wri_data	; 1 op / 2 cycles
	MOVF	RCREG,	W	; 1 op / 1 cycle	; SIDCLK fall

	; Copy received byte to data port
	MOVWF	SIDDAT		; 1 op / 1 cycle
	; SID control lines
__wri	MOVF	CSENCTL,W	; 1 op / 1 cycle	
	NOP
	
	NOP
	ANDWF	SIDCTL,	F	; 1 op / 1 cycle	
	MOVF	CSDICTL,W	; 1 op / 1 cycle	; SIDCLK fall: write effective

	IORWF	SIDCTL,	F	; 1 op / 1 cycle 
	NOP			; 1 op / 1 cycle
	GOTO	$+1		; 1 op / 2 cycles
	GOTO	__GET_ADDR	; 1 op / 2 cycles	; SIDCLK


; Get requested address
; In order for cycle-accurate writes to work, execution time after clearing RCREG
; MUST NOT take more than CYCCHR/2
; Execution time: CYCCHR + 1 + 1 (jump table) SIDCLKs max
__GET_ADDR
ga_waddr
	GOTO	$+1
	NOP
	
	BTFSS	PIR1,	RCIF
	GOTO	ga_waddr 
	MOVF	RCREG,	W				; SIDCLK
	
	MOVWF	TEMPBUF		; Copy address to TEMPBUF, used by jump table
	ANDLW	B'11100000'	; W: ddd00000
	MOVWF	WAITCNT		; WAITCNT: ddd00000
	
	SWAPF	WAITCNT, F	; WAITCNT: 0000ddd0 => ddd is delay cycles
	GOTO	BTABLE					; SIDCLK
	; JUMP TABLE BASED ON CONTENT OF TEMPBUF, aka ADDRESS


; Delayed data read
; Code flow continues into __READ_REGI to save a goto and maintain clock alignment
; Execution time: WAITCNT SIDCLKs. Ensures read happens after exactly WAITCNT SIDclks
__READ_REGD
rrd_waitone
#ifdef SETHIBDRT	; WAITCNT is twice the desired value, decrement 2-by-2 on each loop
	DECF	WAITCNT,F
#else
	NOP
#endif
	GOTO	$+1
	
	DECFSZ	WAITCNT,F
	GOTO	rrd_waitone
	NOP						; SIDCLK
	
; SID reads are latched before falling SIDCLK edge within max 350ns of /CS
; Immediate data read
; Execution time: 3 SIDCLKs.
; Register is read on current clock cycle
; Transfer time: CYCCHR + 1 1/2 SIDCLK.
__READ_REGI
	MOVF	CSENCTL,W
	BSF	SIDCTL,	CTL_RW	; set RW high ahead of clock rise
	ANDWF	SIDCTL,	F				; SC high
	
	; Set data port to input. Apparently this can be done after SIDCTL. Saves a full cycle
	BSF	STATUS,	RP0
	COMF	SIDDDR,	F
	BCF	STATUS, RP0				; SC low, data valid
	
	NOP
	MOVF	SIDDAT,	W
	MOVWF	TXREG		; start RS232 1/2 SID cycle	; SC high
	
	MOVF	CSDICTL,W 
	IORWF	SIDCTL,	F
	BCF	SIDCTL,	CTL_RW				; SIDCLK low

	; Set data port back to output
	BSF	STATUS,	RP0
	CLRF	SIDDDR
	BCF	STATUS, RP0				; SC high

	NOP
	GOTO	__GET_ADDR				; SIDCLK low

	
__IOCTLS
; Reset IOCTL
; Trigger hardware SID reset
; Execution time: 1 SIDCLKs (+reset time)
; This will result in 1+20+1 SIDCLKs minimum total time (reset loop + rsync)
; Caution must thus be taken when calling this routine to not overflow the RX buffer
IOCTRS
	MOVLW	.20	; per datasheet, min 10 SIDCLKs for reset
	MOVWF	WAITCNT
	GOTO	$+1
	GOTO	sid_reset				; SIDCLK

; Select SID0 IOCTL
; Set bitmasks for SID0-only operation
; Execution time: 1 SIDCLKs
IOCTS0
	MOVLW	~CS0BITM
	MOVWF	CSENCTL			; Write enable bits
	MOVLW	CS0BITM
	
	MOVWF	CSDICTL			; Write disable bits
	GOTO	__GET_ADDR				;SIDCLK
	

; Select SID1 IOCTL
; Set bitmasks for SID1-only operation
; Execution time: 1 SIDCLKs
IOCTS1
	MOVLW	~CS1BITM
	MOVWF	CSENCTL			; Write enable bits
	MOVLW	CS1BITM
	
	MOVWF	CSDICTL			; Write disable bits
	GOTO	__GET_ADDR				; SIDCLK

; Select both SIDs IOCTL
; Set bitmasks for dual SID operation
; Execution time: 1 SIDCLKs
IOCTSB
	MOVLW	~CSBBITM
	MOVWF	CSENCTL			; Write enable bits
	MOVLW	CSBBITM
	
	MOVWF	CSDICTL			; Write disable bits
	GOTO	__GET_ADDR				; SIDCLK

; Firmware version IOCTL
; Returns current firmware version
; Execution time: 1 SIDCLKs (+send time)
IOCTFV
	MOVLW	FWVERS
	MOVWF	TXREG
	GOTO	$+1
	GOTO	__GET_ADDR				; SIDCLK

; Hardware version IOCTL
; Returns current hardware version
; Execution time: 1 SIDCLKs (+send time)
IOCTHV
	MOVLW	HWVERS
	MOVWF	TXREG
	GOTO	$+1
	GOTO	__GET_ADDR				; SIDCLK

; Long Delay IOCTL
; Delay value MUST BE >0
; TODO: implement as a timer interrupt?
; Execution time:
; Total SIDCLK count: 2*CYCCHR [addr+data] + WAITCNT(250*2 + 1) + CYCCHR [tx]
;			3*CYCCHR + WAITCNT(500 + 1)
IOCTLD
	GOTO $+1
	NOP
	
	BTFSS	PIR1,	RCIF	; 1 op / 1 cycle until branch, 2 cycles when branch
	GOTO	IOCTLD		; 1 op / 2 cycles
	MOVF	RCREG,	W	; 1 op / 1 cycle	; SIDCLK

	; 2*CYCCHR before we arrive here: address decoding + own read
	MOVWF	WAITCNT+1
	GOTO	$+1
	
icld_waitlong
	MOVLW	0xFA		; wait x times 250 SIDCLKs
	MOVWF	WAITCNT
	NOP						; SIDCLK
	
icld_waitcycle
	GOTO	$+1
	GOTO	$+1
	GOTO	$+1					; SIDCLK
	
	GOTO	$+1
	NOP
	
	DECFSZ	WAITCNT,F
	GOTO	icld_waitcycle
	NOP						; SIDCLK
	; This loop executes in 2*SIDCLK.
	
	DECFSZ	WAITCNT+1,F
	GOTO	icld_waitlong
	NOP
	; When here, we are at SIDCLK/2
	
	CLRF	TXREG		; 1 op	dummy write / 1 CYCCHR before fully transmitted
	GOTO	__GET_ADDR				; SIDCLK
	
	
; A simple start tune routine that reads data from program memory
; Currently only handles writes and delay: addresses are written to the SIDs,
; unless it starts with 001xxxxx, in which case address<0:4> + data byte are
; used to form a 13bit delay value in multiples of 10 SIDCLKs.
; Since this routine is called before external input can mess with the device,
; by default it will play the tune on both SIDs simultaneously.
;
; Note: this works because the PIC can store 14 bits per address location, and
; the SID address space fits 5 bits. We can thus pack SID address (5 bits) + SID
; data (8 bits) on a single location and have 1 extra bit for other use.
__INITTUNE
	BANKSEL EECON1		; 2 cycles Bank 3
#ifdef __PIC16F88X
	BSF	EECON1, EEPGD	; Point to PROGRAM memory
#else ;__PIC16F72X
	NOP
#endif
	BSF	EECON1, RD	; Toggle Read
	NOP			; Contrary to what the Datasheet says, the next instruction
				; after BSF EECON1,RD does *NOT* execute normally
	NOP	;SC		; Instruction here is ignored as program
				; memory is read in second cycle after BSF EECON1,RD
	
	; Fetch data read
	BCF	STATUS, RP0	; Bank 2
	MOVF	EEDAT,	W	; W = LSB of Program Memory
	MOVWF	LPMBYTE
	MOVF	EEDATH, W	; W = MSB of Program EEDAT
	MOVWF	HPMBYTE
	
	; prepare next memory address
	INCF	EEADRH,	F	; SC
	INCFSZ	EEADR,	F
	DECF	EEADRH,	F
	
	; Routine exit condition
	; 0x3F address byte marks end of tune: can't use 0x3Fxx delays
	SUBLW	0x3F
	
	BTFSC	STATUS, Z
	RETURN			; SC if Z (exit) -- XXX returns in Bank 2
	BCF	STATUS,	RP1	; SC if !Z - BANK 0 necessary for the rest of the code

	; Check if delay is requested
	BTFSC	HPMBYTE,5
	GOTO	st_delay
	
	; Copy received address to address port
	MOVF	HPMBYTE,W	; sc h
	MOVWF	SIDADR
	
	; Copy received data to data port
	MOVF	LPMBYTE,W
	MOVWF	SIDDAT		; SC l

	; SID control lines
	GOTO	$+1
	NOP			; sc h
	
	MOVF	CSENCTL,W	
	ANDWF	SIDCTL,	F
	MOVF	CSDICTL,W	; SIDCLK fall

	IORWF	SIDCTL,	F
	GOTO	$+1
	
	NOP
	GOTO	__INITTUNE	; SC
	
	; Expects non-zero delay. delay = 1+1+10*cnt cycles
	; MSB can only be coded on 5 bits.
st_delay
	MOVF	HPMBYTE,W
	ANDLW	B'00011111'
	MOVWF	HPMBYTE		; SC low	

st_dloop2	; 10 SIDCLKs (0.01 ms) loop
	MOVF	LPMBYTE,W
	BTFSC	STATUS, Z
	DECF	HPMBYTE,F	; if low = 0, dec high
	DECF	LPMBYTE,F
	MOVF	HPMBYTE,W
	IORWF	LPMBYTE,W	; SC

	GOTO	$+1
	GOTO	$+1
	GOTO	$+1		; SC

	GOTO	$+1
	GOTO	$+1
	GOTO	$+1		; SC

	GOTO	$+1
	GOTO	$+1
	GOTO	$+1		; SC

	GOTO	$+1
	GOTO	$+1
	GOTO	$+1		; SC

	GOTO	$+1
	GOTO	$+1
	GOTO	$+1		; SC

	GOTO	$+1
	GOTO	$+1
	GOTO	$+1		; SC

	GOTO	$+1
	GOTO	$+1
	GOTO	$+1		; SC

	GOTO	$+1
	GOTO	$+1
	GOTO	$+1		; SC
	
	GOTO	$+1
	NOP
	
	BTFSS	STATUS, Z
	GOTO	st_dloop2	; SC if !Z
	NOP			; SC if Z
	
	GOTO	$+1
	GOTO	$+1	
	GOTO	__INITTUNE	; SC - both numbers 0, we're done.
				

		
; Voice frequency set macro
Mvpitch	MACRO	Voice, Fout
	local Fn = Fout*.10000 / .596	; per datasheet for 1MHz clock
	dw	((Voice-.1)*.7+.0) << .8 | low Fn
	dw	((Voice-.1)*.7+.1) << .8 | high Fn
	ENDM

; Delay (approx) macro, Time in ms
Mdelay	MACRO	Time
	if (Time > .75)
	error "Time cannot be more than 75ms"
	endif
	dw	(Time*.100)|0x2000
	ENDM
	
; Combo macro to set pitches on all 3 voices with pre-delay
Mcombo	MACRO	Delay, P1, P2, P3
	Mdelay	Delay
	Mvpitch	.1, P1
	Mvpitch .2, P2
	Mvpitch	.3, P3
	ENDM

; SID reg clearing macro
Mreset	MACRO
	local Reg=0
	while (Reg <= 0x18)
	dw	Reg << .8
Reg++
	endw
	ENDM
	
; Init tune data. Play a custom deep note rendition. Takes most of the memory space.
; Could probably have been done slightly more elegantly with programmatic macros,
; but in the end it sounds credible.
; Filter settings are middle ground between 8580 (too bright) and 6581 (too deep)
__TUNEDATA	ORG	0x0200	; 14bit max values
	; There's a big DC spike at power on, lasting for nearly 500ms.
	; Avoid the worst of it with this pre-delay
	Mdelay	.50
	Mdelay	.50
	; 100ms
	
	Mreset			; reset SID

	dw	0x1500
	dw	0x1680		; cutoff to mid
	dw	0x1707		; no res / filter all voices
	dw	0x181F		; LP filter / turn up volume
	
	Mvpitch	.1, .200
	dw	0x05D0		; OSC1 -> Atk=D (3000ms), Dcy=0 (6ms)
	dw	0x06F9		; OSC1 -> Stn=F, Rls=9 (750ms)
	
	Mvpitch	.2, .205
	dw	0x0CD0		; OSC2 -> Atk=D (3000ms), Dcy=1 (6ms)
	dw	0x0DF9		; OSC2 -> Stn=F, Rls=9 (750ms)
	
	Mvpitch	.3, .195
	dw	0x13D0		; OSC3 -> Atk=D (3000ms), Dcy=1 (6ms)
	dw	0x14F9		; OSC3 -> Stn=F, Rls=9 (750ms)

	dw	0x0421		; OSC1 -> saw/gate
	dw	0x0B31		; OSC2 -> saw+tri/gate
	dw	0x1221		; OSC3 -> saw/gate
	
	Mcombo	.25, .198, .202, .196
	Mcombo	.25, .196, .200, .195
	Mcombo	.25, .195, .201, .194
	Mcombo	.25, .197, .203, .193
	Mcombo	.25, .199, .200, .194
	Mcombo	.25, .203, .198, .195
	Mcombo	.25, .201, .199, .196
	Mcombo	.25, .200, .201, .195
	Mcombo	.25, .202, .200, .194
	Mcombo	.25, .205, .197, .193
	; 350ms
	
	Mcombo	.25, .198, .202, .196
	Mcombo	.25, .196, .200, .195
	Mcombo	.25, .195, .201, .194
	Mcombo	.25, .197, .203, .193
	Mcombo	.25, .199, .200, .194
	Mcombo	.25, .203, .198, .195
	Mcombo	.25, .201, .199, .196
	Mcombo	.25, .200, .201, .195
	Mcombo	.25, .202, .200, .194
	Mcombo	.25, .205, .197, .193
	; 600ms
	
	Mcombo	.25, .208, .196, .194
	Mcombo	.25, .206, .198, .195
	Mcombo	.25, .209, .199, .194
	Mcombo	.25, .211, .201, .193
	Mcombo	.25, .212, .199, .192
	Mcombo	.25, .210, .197, .193
	Mcombo	.25, .208, .195, .192
	Mcombo	.25, .211, .196, .191
	Mcombo	.25, .214, .194, .190
	Mcombo	.25, .215, .192, .191
	; 850ms
	
	Mcombo	.25, .218, .193, .192
	Mcombo	.25, .217, .191, .191
	Mcombo	.25, .219, .188, .190
	Mcombo	.25, .222, .186, .191
	Mcombo	.25, .223, .189, .190
	Mcombo	.25, .221, .187, .189
	Mcombo	.25, .220, .184, .188
	Mcombo	.25, .224, .180, .189
	Mcombo	.25, .228, .182, .190
	Mcombo	.25, .231, .179, .189
	; 1100ms
	
	Mcombo	.25, .229, .176, .190
	Mcombo	.25, .232, .177, .188
	Mcombo	.25, .238, .174, .189
	Mcombo	.25, .240, .171, .187
	Mcombo	.25, .237, .173, .188
	Mcombo	.25, .241, .170, .185
	Mcombo	.25, .246, .167, .187
	Mcombo	.25, .250, .163, .185
	Mcombo	.25, .248, .165, .183
	Mcombo	.25, .255, .166, .182
	; 1350ms
	
	Mcombo	.25, .257, .164, .184
	Mcombo	.25, .254, .161, .181
	Mcombo	.25, .259, .158, .179
	Mcombo	.25, .265, .160, .182
	Mcombo	.25, .271, .159, .178
	Mcombo	.25, .275, .157, .177
	Mcombo	.25, .273, .154, .180
	Mcombo	.25, .274, .153, .176
	Mcombo	.25, .279, .155, .173
	Mcombo	.25, .285, .156, .175
	; 1600ms
	
	Mcombo	.25, .292, .154, .170
	Mcombo	.25, .296, .153, .172
	Mcombo	.25, .294, .151, .166
	Mcombo	.25, .299, .150, .162
	Mcombo	.25, .310, .152, .158
	Mcombo	.25, .314, .149, .161
	Mcombo	.25, .312, .147, .163
	Mcombo	.25, .318, .148, .159
	Mcombo	.25, .325, .150, .154
	Mcombo	.25, .333, .151, .148
	; 1850ms
	
	Mcombo	.25, .345, .149, .151
	Mcombo	.25, .340, .145, .147
	Mcombo	.25, .348, .142, .140
	Mcombo	.25, .359, .143, .142
	Mcombo	.25, .375, .144, .135
	Mcombo	.25, .370, .141, .128
	Mcombo	.25, .388, .138, .131
	Mcombo	.25, .392, .136, .125
	Mcombo	.25, .404, .137, .118
	Mcombo	.25, .418, .139, .110
	; 2100ms
	
	Mcombo	.25, .410, .142, .112
	Mcombo	.25, .415, .144, .106
	Mcombo	.25, .438, .146, .101
	Mcombo	.25, .447, .148, .095
	Mcombo	.25, .440, .149, .097
	Mcombo	.25, .451, .150, .093
	Mcombo	.25, .470, .149, .088
	Mcombo	.25, .495, .148, .091
	Mcombo	.25, .512, .147, .085
	Mcombo	.25, .504, .146, .082
	; 2350ms
	
	Mcombo	.25, .516, .145, .084
	Mcombo	.25, .533, .146, .081
	Mcombo	.25, .552, .147, .079
	Mcombo	.25, .568, .148, .077
	Mcombo	.25, .579, .149, .078
	Mcombo	.25, .589, .148, .076
	Mcombo	.25, .597, .147, .074
	Mcombo	.25, .590, .146, .075
	Mcombo	.25, .582, .145, .074
	Mcombo	.25, .576, .146, .073
	; 2600ms
	
	Mcombo	.25, .583, .147, .072
	Mcombo	.25, .588, .148, .071
	Mcombo	.25, .597, .149, .072
	Mcombo	.25, .587, .148, .073
	Mcombo	.25, .582, .147, .074
	Mcombo	.25, .587, .146, .073
	Mcombo	.25, .590, .145, .072
	Mcombo	.25, .587, .146, .073
	Mcombo	.25, .583, .147, .073
	Mcombo	.25, .587, .146, .074
	; 2850ms

	
	Mcombo	.25, .583, .147, .073
	Mcombo	.25, .588, .148, .072
	Mcombo	.25, .597, .147, .071
	Mcombo	.25, .587, .146, .072
	Mcombo	.25, .582, .147, .073
	Mcombo	.25, .587, .148, .074
	Mcombo	.25, .590, .147, .073
	Mcombo	.25, .587, .146, .072
	Mcombo	.25, .583, .147, .073
	Mcombo	.25, .587, .148, .074
	; 3100ms
	
	Mcombo	.25, .583, .147, .073
	Mcombo	.25, .588, .148, .072
	Mcombo	.25, .597, .147, .071
	Mcombo	.25, .587, .146, .072
	Mcombo	.25, .582, .147, .073
	Mcombo	.25, .587, .148, .074
	Mcombo	.25, .590, .147, .073
	Mcombo	.25, .587, .146, .072
	Mcombo	.25, .583, .147, .073
	Mcombo	.25, .587, .148, .074
	; 3350ms
	
	Mcombo	.25, .583, .147, .073
	Mcombo	.25, .588, .148, .072
	Mcombo	.25, .597, .147, .071
	Mcombo	.25, .587, .146, .072
	Mcombo	.25, .582, .147, .073
	Mcombo	.25, .587, .148, .074
	Mcombo	.25, .590, .147, .073
	Mcombo	.25, .587, .146, .072
	Mcombo	.25, .583, .147, .073
	Mcombo	.25, .587, .148, .074
	; 3600ms
	
	Mcombo	.25, .583, .147, .073
	Mcombo	.25, .588, .148, .072
	Mcombo	.25, .597, .147, .071
	Mcombo	.25, .587, .146, .072
	Mcombo	.25, .582, .147, .073
	Mcombo	.25, .587, .148, .074
	Mcombo	.25, .590, .147, .073
	Mcombo	.25, .587, .146, .072
	Mcombo	.25, .583, .147, .073
	Mcombo	.25, .587, .147, .073
	; 3850ms

	dw	0x0420		; OSC1 -> saw/nogate	
	dw	0x0B30		; OSC2 -> saw+tri/nogate
	dw	0x1220		; OSC3 -> saw/nogate

	; delay enough for release cycle
	
	Mdelay	.50
	Mdelay	.50
	Mdelay	.50
	; 4000ms
	Mdelay	.50
	Mdelay	.50
	; 4100ms
	Mdelay	.50
	Mdelay	.50
	; 4200ms
	Mdelay	.50
	Mdelay	.50
	; 4300ms
	Mdelay	.50
	Mdelay	.50
	; 4400ms
	Mdelay	.50
	Mdelay	.50
	; 4500ms
	Mdelay	.50
	Mdelay	.50
	; 4600ms
	Mdelay	.50
	Mdelay	.50
	; 4700ms
	Mdelay	.50
	Mdelay	.50
	; 4800ms

	dw	0x3FFF		; END


; Branch table. IOCTL are interleaved with valid SID addresses
; Execution time: 6 cycles / 1 SIDCLK
; XXX predelay 5 cycles: I suspect the MOVWF PCL requires 2 cycles to take effect
BTABLE	ORG	JMPTBA-3
	MOVF	TEMPBUF,W
	MOVWF	SIDADR		; Output address to SID A-lines
	MOVWF	PCL		; PCLATH must be set before the jump
	FILL	(GOTO	__WRITE_REGI), 0x19	; 25 valid write addresses up to 0x18, data
	FILL	(GOTO	__READ_REGI), 0x4	; 4 valid read addresses up to 0x1C, no data
	NOP				; 0x1D UNUSED
	NOP				; 0x1E UNUSED
	NOP				; 0x1F UNUSED
	; Wrapped addresses for delayed I/O start here
	FILL	(GOTO	__WRITE_REGD), 0x19	; calling delayed write
	FILL	(GOTO	__READ_REGD), 0x4
	NOP				; 0x3D UNUSED
	NOP				; 0x3E UNUSED
	NOP				; 0x3F UNUSED
	FILL	(GOTO	__WRITE_REGD), 0x19
	FILL	(GOTO	__READ_REGD), 0x4
	NOP				; 0x5D UNUSED
	NOP				; 0x5E UNUSED
	NOP				; 0x5F UNUSED
	FILL	(GOTO	__WRITE_REGD), 0x19
	FILL	(GOTO	__READ_REGD), 0x4
	NOP				; 0x7D UNUSED
	NOP				; 0x7E UNUSED
	NOP				; 0x7F UNUSED
	FILL	(GOTO	__WRITE_REGD), 0x19
	FILL	(GOTO	__READ_REGD), 0x4
	GOTO	__GET_ADDR		; 0x9D IOCTD1 CYCCHR SIDCLK delay, no data
	GOTO	IOCTLD			; 0x9E IOCTLD long delay, amount in data, returns 1 byte
	NOP				; 0x9F UNUSED
	FILL	(GOTO	__WRITE_REGD), 0x19
	FILL	(GOTO	__READ_REGD), 0x4
	GOTO	IOCTS0			; 0xBD IOCTS0 select Chip 0, no data
	GOTO	IOCTS1			; 0xBE IOCTS1 select Chip 1, no data
	GOTO	IOCTSB			; 0xBF IOCTSB select both chips, no data
	FILL	(GOTO	__WRITE_REGD), 0x19
	FILL	(GOTO	__READ_REGD), 0x4
	NOP				; 0xDD UNUSED
	NOP				; 0xDE UNUSED
	NOP				; 0xDF UNUSED
	FILL	(GOTO	__WRITE_REGD), 0x19
	FILL	(GOTO	__READ_REGD), 0x4
	GOTO	IOCTFV			; 0xFD Firmware version, no data, returns 1 byte
	GOTO	IOCTHV			; 0xFE Hardware version, no data, returns 1 byte
__END	GOTO	IOCTRS			; 0xFF Reset, no data

; !!! 0x07FF Program must stay in Page 0 !!!		
	END
	
; CHANGELOG
	
; Version 10:	- Initial release, support for 750kbps/1Mbps
; Version 11:	- Switch to integer versionning (1.0 became 10)
;		- Support for 2Mbps on 88X
;		- Support for rev.C hardware
;		- Improved SID clocking
;		- Fixed init tune timing bug
;		- Switched to deep note init tune
;		- Fixed timing bug (buffer overrun when consecutive delayed writes with maximum adjustment)

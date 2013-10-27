;--------------------------------------------------------
; File Created by SDCC : free open source ANSI-C Compiler
; Version 2.9.4 #5595 (Oct 15 2013) (Mac OS X ppc)
; This file was generated Sun Oct 27 21:13:55 2013
;--------------------------------------------------------
; PIC16 port for the Microchip 16-bit core micros
;--------------------------------------------------------
	list	p=18f97j60

	radix dec

;--------------------------------------------------------
; public variables in this module
;--------------------------------------------------------
	global _time
	global _refresh_lcd
	global _alarm
	global _button
	global _inc_ahour
	global _inc_amin
	global _button1
	global _button2
	global _overflows
	global _sec
	global _decisec
	global _ds
	global _h
	global _m
	global _s
	global _thour
	global _tmin
	global _tsec
	global _ahour
	global _amin
	global _ahour_o
	global _amin_o
	global _alarm_set
	global _snooze
	global _stop_ringing
	global _whereami
	global _on
	global _display
	global _high_isr
	global _low_isr
	global _main

;--------------------------------------------------------
; extern variables in this module
;--------------------------------------------------------
	extern _EBSTCONbits
	extern _MISTATbits
	extern _EFLOCONbits
	extern _MACON1bits
	extern _MACON2bits
	extern _MACON3bits
	extern _MACON4bits
	extern _MACLCON1bits
	extern _MACLCON2bits
	extern _MICONbits
	extern _MICMDbits
	extern _EWOLIEbits
	extern _EWOLIRbits
	extern _ERXFCONbits
	extern _EIEbits
	extern _ESTATbits
	extern _ECON2bits
	extern _EIRbits
	extern _EDATAbits
	extern _SSP2CON2bits
	extern _SSP2CON1bits
	extern _SSP2STATbits
	extern _ECCP2DELbits
	extern _ECCP2ASbits
	extern _ECCP3DELbits
	extern _ECCP3ASbits
	extern _RCSTA2bits
	extern _TXSTA2bits
	extern _CCP5CONbits
	extern _CCP4CONbits
	extern _T4CONbits
	extern _ECCP1DELbits
	extern _BAUDCON2bits
	extern _BAUDCTL2bits
	extern _BAUDCONbits
	extern _BAUDCON1bits
	extern _BAUDCTLbits
	extern _BAUDCTL1bits
	extern _PORTAbits
	extern _PORTBbits
	extern _PORTCbits
	extern _PORTDbits
	extern _PORTEbits
	extern _PORTFbits
	extern _PORTGbits
	extern _PORTHbits
	extern _PORTJbits
	extern _LATAbits
	extern _LATBbits
	extern _LATCbits
	extern _LATDbits
	extern _LATEbits
	extern _LATFbits
	extern _LATGbits
	extern _LATHbits
	extern _LATJbits
	extern _DDRAbits
	extern _TRISAbits
	extern _DDRBbits
	extern _TRISBbits
	extern _DDRCbits
	extern _TRISCbits
	extern _DDRDbits
	extern _TRISDbits
	extern _DDREbits
	extern _TRISEbits
	extern _DDRFbits
	extern _TRISFbits
	extern _DDRGbits
	extern _TRISGbits
	extern _DDRHbits
	extern _TRISHbits
	extern _DDRJbits
	extern _TRISJbits
	extern _OSCTUNEbits
	extern _MEMCONbits
	extern _PIE1bits
	extern _PIR1bits
	extern _IPR1bits
	extern _PIE2bits
	extern _PIR2bits
	extern _IPR2bits
	extern _PIE3bits
	extern _PIR3bits
	extern _IPR3bits
	extern _EECON1bits
	extern _RCSTAbits
	extern _RCSTA1bits
	extern _TXSTAbits
	extern _TXSTA1bits
	extern _PSPCONbits
	extern _T3CONbits
	extern _CMCONbits
	extern _CVRCONbits
	extern _ECCP1ASbits
	extern _CCP3CONbits
	extern _ECCP3CONbits
	extern _CCP2CONbits
	extern _ECCP2CONbits
	extern _CCP1CONbits
	extern _ECCP1CONbits
	extern _ADCON2bits
	extern _ADCON1bits
	extern _ADCON0bits
	extern _SSP1CON2bits
	extern _SSPCON2bits
	extern _SSP1CON1bits
	extern _SSPCON1bits
	extern _SSP1STATbits
	extern _SSPSTATbits
	extern _T2CONbits
	extern _T1CONbits
	extern _RCONbits
	extern _WDTCONbits
	extern _ECON1bits
	extern _OSCCONbits
	extern _T0CONbits
	extern _STATUSbits
	extern _INTCON3bits
	extern _INTCON2bits
	extern _INTCONbits
	extern _STKPTRbits
	extern _stdin
	extern _stdout
	extern _LCDText
	extern _MAADR5
	extern _MAADR6
	extern _MAADR3
	extern _MAADR4
	extern _MAADR1
	extern _MAADR2
	extern _EBSTSD
	extern _EBSTCON
	extern _EBSTCS
	extern _EBSTCSL
	extern _EBSTCSH
	extern _MISTAT
	extern _EFLOCON
	extern _EPAUS
	extern _EPAUSL
	extern _EPAUSH
	extern _MACON1
	extern _MACON2
	extern _MACON3
	extern _MACON4
	extern _MABBIPG
	extern _MAIPG
	extern _MAIPGL
	extern _MAIPGH
	extern _MACLCON1
	extern _MACLCON2
	extern _MAMXFL
	extern _MAMXFLL
	extern _MAMXFLH
	extern _MICON
	extern _MICMD
	extern _MIREGADR
	extern _MIWR
	extern _MIWRL
	extern _MIWRH
	extern _MIRD
	extern _MIRDL
	extern _MIRDH
	extern _EHT0
	extern _EHT1
	extern _EHT2
	extern _EHT3
	extern _EHT4
	extern _EHT5
	extern _EHT6
	extern _EHT7
	extern _EPMM0
	extern _EPMM1
	extern _EPMM2
	extern _EPMM3
	extern _EPMM4
	extern _EPMM5
	extern _EPMM6
	extern _EPMM7
	extern _EPMCS
	extern _EPMCSL
	extern _EPMCSH
	extern _EPMO
	extern _EPMOL
	extern _EPMOH
	extern _EWOLIE
	extern _EWOLIR
	extern _ERXFCON
	extern _EPKTCNT
	extern _EWRPT
	extern _EWRPTL
	extern _EWRPTH
	extern _ETXST
	extern _ETXSTL
	extern _ETXSTH
	extern _ETXND
	extern _ETXNDL
	extern _ETXNDH
	extern _ERXST
	extern _ERXSTL
	extern _ERXSTH
	extern _ERXND
	extern _ERXNDL
	extern _ERXNDH
	extern _ERXRDPT
	extern _ERXRDPTL
	extern _ERXRDPTH
	extern _ERXWRPT
	extern _ERXWRPTL
	extern _ERXWRPTH
	extern _EDMAST
	extern _EDMASTL
	extern _EDMASTH
	extern _EDMAND
	extern _EDMANDL
	extern _EDMANDH
	extern _EDMADST
	extern _EDMADSTL
	extern _EDMADSTH
	extern _EDMACS
	extern _EDMACSL
	extern _EDMACSH
	extern _EIE
	extern _ESTAT
	extern _ECON2
	extern _EIR
	extern _EDATA
	extern _SSP2CON2
	extern _SSP2CON1
	extern _SSP2STAT
	extern _SSP2ADD
	extern _SSP2BUF
	extern _ECCP2DEL
	extern _ECCP2AS
	extern _ECCP3DEL
	extern _ECCP3AS
	extern _RCSTA2
	extern _TXSTA2
	extern _TXREG2
	extern _RCREG2
	extern _SPBRG2
	extern _CCP5CON
	extern _CCPR5
	extern _CCPR5L
	extern _CCPR5H
	extern _CCP4CON
	extern _CCPR4
	extern _CCPR4L
	extern _CCPR4H
	extern _T4CON
	extern _PR4
	extern _TMR4
	extern _ECCP1DEL
	extern _ERDPT
	extern _ERDPTL
	extern _ERDPTH
	extern _BAUDCON2
	extern _BAUDCTL2
	extern _SPBRGH2
	extern _BAUDCON
	extern _BAUDCON1
	extern _BAUDCTL
	extern _BAUDCTL1
	extern _SPBRGH
	extern _SPBRGH1
	extern _PORTA
	extern _PORTB
	extern _PORTC
	extern _PORTD
	extern _PORTE
	extern _PORTF
	extern _PORTG
	extern _PORTH
	extern _PORTJ
	extern _LATA
	extern _LATB
	extern _LATC
	extern _LATD
	extern _LATE
	extern _LATF
	extern _LATG
	extern _LATH
	extern _LATJ
	extern _DDRA
	extern _TRISA
	extern _DDRB
	extern _TRISB
	extern _DDRC
	extern _TRISC
	extern _DDRD
	extern _TRISD
	extern _DDRE
	extern _TRISE
	extern _DDRF
	extern _TRISF
	extern _DDRG
	extern _TRISG
	extern _DDRH
	extern _TRISH
	extern _DDRJ
	extern _TRISJ
	extern _OSCTUNE
	extern _MEMCON
	extern _PIE1
	extern _PIR1
	extern _IPR1
	extern _PIE2
	extern _PIR2
	extern _IPR2
	extern _PIE3
	extern _PIR3
	extern _IPR3
	extern _EECON1
	extern _EECON2
	extern _RCSTA
	extern _RCSTA1
	extern _TXSTA
	extern _TXSTA1
	extern _TXREG
	extern _TXREG1
	extern _RCREG
	extern _RCREG1
	extern _SPBRG
	extern _SPBRG1
	extern _PSPCON
	extern _T3CON
	extern _TMR3L
	extern _TMR3H
	extern _CMCON
	extern _CVRCON
	extern _ECCP1AS
	extern _CCP3CON
	extern _ECCP3CON
	extern _CCPR3
	extern _CCPR3L
	extern _CCPR3H
	extern _CCP2CON
	extern _ECCP2CON
	extern _CCPR2
	extern _CCPR2L
	extern _CCPR2H
	extern _CCP1CON
	extern _ECCP1CON
	extern _CCPR1
	extern _CCPR1L
	extern _CCPR1H
	extern _ADCON2
	extern _ADCON1
	extern _ADCON0
	extern _ADRES
	extern _ADRESL
	extern _ADRESH
	extern _SSP1CON2
	extern _SSPCON2
	extern _SSP1CON1
	extern _SSPCON1
	extern _SSP1STAT
	extern _SSPSTAT
	extern _SSP1ADD
	extern _SSPADD
	extern _SSP1BUF
	extern _SSPBUF
	extern _T2CON
	extern _PR2
	extern _TMR2
	extern _T1CON
	extern _TMR1L
	extern _TMR1H
	extern _RCON
	extern _WDTCON
	extern _ECON1
	extern _OSCCON
	extern _T0CON
	extern _TMR0L
	extern _TMR0H
	extern _STATUS
	extern _FSR2L
	extern _FSR2H
	extern _PLUSW2
	extern _PREINC2
	extern _POSTDEC2
	extern _POSTINC2
	extern _INDF2
	extern _BSR
	extern _FSR1L
	extern _FSR1H
	extern _PLUSW1
	extern _PREINC1
	extern _POSTDEC1
	extern _POSTINC1
	extern _INDF1
	extern _WREG
	extern _FSR0L
	extern _FSR0H
	extern _PLUSW0
	extern _PREINC0
	extern _POSTDEC0
	extern _POSTINC0
	extern _INDF0
	extern _INTCON3
	extern _INTCON2
	extern _INTCON
	extern _PROD
	extern _PRODL
	extern _PRODH
	extern _TABLAT
	extern _TBLPTR
	extern _TBLPTRL
	extern _TBLPTRH
	extern _TBLPTRU
	extern _PC
	extern _PCL
	extern _PCLATH
	extern _PCLATU
	extern _STKPTR
	extern _TOS
	extern _TOSL
	extern _TOSH
	extern _TOSU
	extern _sprintf
	extern _LCDInit
	extern _DisplayString
	extern ___ulong2fs
	extern ___fsdiv
	extern ___fs2ulong
	extern __mullong
	extern __modulong
	extern __divulong
	extern ___fssub
	extern ___fsadd
	extern __modsint
	extern __divsint
;--------------------------------------------------------
;	Equates to used internal registers
;--------------------------------------------------------
STATUS	equ	0xfd8
PCL	equ	0xff9
PCLATH	equ	0xffa
PCLATU	equ	0xffb
WREG	equ	0xfe8
BSR	equ	0xfe0
FSR0L	equ	0xfe9
FSR0H	equ	0xfea
FSR1L	equ	0xfe1
FSR2L	equ	0xfd9
POSTDEC1	equ	0xfe5
PREINC1	equ	0xfe4
PLUSW2	equ	0xfdb
PRODL	equ	0xff3
PRODH	equ	0xff4


	idata
_button1	db	0x00
_button2	db	0x00
_overflows	db	0x00, 0x00, 0x00, 0x00
_thour	db	0x00
_tmin	db	0x00
_tsec	db	0x00
_ahour	db	0x00
_amin	db	0x00
_ahour_o	db	0x00
_amin_o	db	0x00
_alarm_set	db	0x00
_snooze	db	0x00
_stop_ringing	db	0x00
_whereami	db	0x00
_on	db	0x00


; Internal registers
.registers	udata_ovr	0x0000
r0x00	res	1
r0x01	res	1
r0x02	res	1
r0x03	res	1
r0x04	res	1
r0x05	res	1
r0x06	res	1
r0x07	res	1
r0x08	res	1
r0x09	res	1
r0x0a	res	1
r0x0b	res	1
r0x0c	res	1
r0x0d	res	1
r0x0e	res	1
r0x0f	res	1
r0x10	res	1

udata_reveil_0	udata
_sec	res	4

udata_reveil_1	udata
_decisec	res	4

udata_reveil_2	udata
_ds	res	1

udata_reveil_3	udata
_h	res	1

udata_reveil_4	udata
_m	res	1

udata_reveil_5	udata
_s	res	1

udata_reveil_6	udata
_display	res	32

;--------------------------------------------------------
; interrupt vector 
;--------------------------------------------------------

;--------------------------------------------------------
; global & static initialisations
;--------------------------------------------------------
; ; Starting pCode block for absolute section
; ;-----------------------------------------
S_reveil_ivec_0x1_high_isr	code	0X000008
ivec_0x1_high_isr:
	GOTO	_high_isr

; ; Starting pCode block for absolute section
; ;-----------------------------------------
S_reveil_ivec_0x2_low_isr	code	0X000018
ivec_0x2_low_isr:
	GOTO	_low_isr

; I code from now on!
; ; Starting pCode block
S_reveil__main	code
_main:
;	.line	154; reveil.c	RCONbits.IPEN       = 1; // enable interrupts priority levels
	BSF	_RCONbits, 7
;	.line	155; reveil.c	INTCONbits.GIE      = 1; // enables all high-priority interrupts
	BSF	_INTCONbits, 7
;	.line	156; reveil.c	INTCONbits.PEIE     = 1; // enables all low-priority peripheral interrupts
	BSF	_INTCONbits, 6
;	.line	159; reveil.c	T0CONbits.TMR0ON    = 1; // enables Timer0
	BSF	_T0CONbits, 7
;	.line	160; reveil.c	INTCONbits.TMR0IE   = 1; // enables the TMR0 overflow interrupt
	BSF	_INTCONbits, 5
;	.line	161; reveil.c	INTCONbits.TMR0IF   = 0; // clear Timer0 overflow bit
	BCF	_INTCONbits, 2
;	.line	162; reveil.c	INTCON2bits.TMR0IP  = 1; // high priority
	BSF	_INTCON2bits, 2
;	.line	163; reveil.c	T0CONbits.T08BIT    = 0; // timer0 is configured as a 16-bit timer/counter
	BCF	_T0CONbits, 6
;	.line	164; reveil.c	T0CONbits.T0CS      = 0; // internal instruction cycle clock (CLKO)
	BCF	_T0CONbits, 5
;	.line	165; reveil.c	T0CONbits.PSA       = 1; // timer0 prescaler is not assigned
	BSF	_T0CONbits, 3
;	.line	166; reveil.c	TMR0L = 0;    TMR0H = 0;
	CLRF	_TMR0L
	CLRF	_TMR0H
;	.line	169; reveil.c	LED0_TRIS = 0; // configure 1st led pin as output (yellow)
	BCF	_TRISJbits, 0
;	.line	170; reveil.c	LED1_TRIS = 0; // configure 2nd led pin as output (red)
	BCF	_TRISJbits, 1
;	.line	171; reveil.c	LED2_TRIS = 0; // configure 3rd led pin as output (red)
	BCF	_TRISJbits, 2
;	.line	172; reveil.c	LATJbits.LATJ0 = 0; // switch LED 1 off
	BCF	_LATJbits, 0
;	.line	173; reveil.c	LATJbits.LATJ1 = 0; // switch LED 2 off
	BCF	_LATJbits, 1
;	.line	174; reveil.c	LATJbits.LATJ2 = 0; // switch LED 3 off
	BCF	_LATJbits, 2
;	.line	177; reveil.c	BUTTON0_TRIS        = 1; // configure 1st button as input
	BSF	_TRISBbits, 3
;	.line	178; reveil.c	BUTTON1_TRIS        = 1; // configure 2nd button as input
	BSF	_TRISBbits, 1
;	.line	179; reveil.c	INTCON3bits.INT3E   = 1; // enable INT3 interrupt (button 1)
	BSF	_INTCON3bits, 5
;	.line	180; reveil.c	INTCON3bits.INT3F   = 0; // clear INT3 flag
	BCF	_INTCON3bits, 2
;	.line	181; reveil.c	INTCON3bits.INT1E   = 1; // enable INT1 interrupt (button 2)
	BSF	_INTCON3bits, 3
;	.line	182; reveil.c	INTCON3bits.INT1F   = 0; // clear INT1 flag
	BCF	_INTCON3bits, 0
;	.line	183; reveil.c	INTCON3bits.INT1IP  = 0; // low priority
	BCF	_INTCON3bits, 6
;	.line	184; reveil.c	INTCON2bits.INT3IP  = 0; // low priority
	BCF	_INTCON2bits, 1
;	.line	186; reveil.c	LCDInit();
	CALL	_LCDInit
;	.line	187; reveil.c	whereami = TIME_MENU;
	MOVLW	0x01
	BANKSEL	_whereami
	MOVWF	_whereami, B
;	.line	189; reveil.c	T0CONbits.TMR0ON = 1; // start timer0
	BSF	_T0CONbits, 7
_00123_DS_:
;	.line	192; reveil.c	time(); // met à jour l'heure
	CALL	_time
;	.line	193; reveil.c	refresh_lcd(); // met à jour l'affichage
	CALL	_refresh_lcd
;	.line	194; reveil.c	alarm(); // vérifie l'alarme
	CALL	_alarm
;	.line	195; reveil.c	button(); // vérifie les boutons
	CALL	_button
	BRA	_00123_DS_
	RETURN	

; ; Starting pCode block
S_reveil__inc_amin	code
_inc_amin:
;	.line	471; reveil.c	void inc_amin(BYTE val)
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	MOVFF	r0x00, POSTDEC1
	MOVFF	r0x01, POSTDEC1
	MOVFF	r0x02, POSTDEC1
	MOVFF	r0x03, POSTDEC1
	MOVLW	0x02
	MOVFF	PLUSW2, r0x00
;	.line	474; reveil.c	mod_amin = (amin + val) / 60;
	MOVFF	_amin, r0x01
	CLRF	r0x02
	CLRF	r0x03
	MOVF	r0x00, W
	ADDWF	r0x01, F
	MOVF	r0x03, W
	ADDWFC	r0x02, F
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x3c
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	CALL	__divsint
	MOVWF	r0x01
	MOVFF	PRODL, r0x02
	MOVLW	0x04
	ADDWF	FSR1L, F
;	.line	475; reveil.c	if (mod_amin) // vérifie s'il faut incrémenter les heures
	MOVF	r0x01, W
	BZ	_00349_DS_
;	.line	476; reveil.c	inc_ahour(mod_amin);
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	CALL	_inc_ahour
	INCF	FSR1L, F
_00349_DS_:
;	.line	478; reveil.c	amin = (amin + val) % 60;
	MOVFF	_amin, r0x01
	CLRF	r0x02
	MOVF	r0x01, W
	ADDWF	r0x00, F
	MOVF	r0x02, W
	ADDWFC	r0x03, F
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x3c
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__modsint
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
	MOVLW	0x04
	ADDWF	FSR1L, F
	MOVF	r0x00, W
	BANKSEL	_amin
	MOVWF	_amin, B
	MOVFF	PREINC1, r0x03
	MOVFF	PREINC1, r0x02
	MOVFF	PREINC1, r0x01
	MOVFF	PREINC1, r0x00
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_reveil__inc_ahour	code
_inc_ahour:
;	.line	465; reveil.c	void inc_ahour(BYTE val)
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	MOVFF	r0x00, POSTDEC1
	MOVFF	r0x01, POSTDEC1
	MOVFF	r0x02, POSTDEC1
	MOVFF	r0x03, POSTDEC1
	MOVLW	0x02
	MOVFF	PLUSW2, r0x00
;	.line	467; reveil.c	ahour = (ahour + val) % 24;
	MOVFF	_ahour, r0x01
	CLRF	r0x02
	CLRF	r0x03
	MOVF	r0x00, W
	ADDWF	r0x01, F
	MOVF	r0x03, W
	ADDWFC	r0x02, F
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x18
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	CALL	__modsint
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
	MOVLW	0x04
	ADDWF	FSR1L, F
	MOVF	r0x00, W
	BANKSEL	_ahour
	MOVWF	_ahour, B
	MOVFF	PREINC1, r0x03
	MOVFF	PREINC1, r0x02
	MOVFF	PREINC1, r0x01
	MOVFF	PREINC1, r0x00
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_reveil__button	code
_button:
;	.line	335; reveil.c	void button(void)
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	MOVFF	r0x00, POSTDEC1
	MOVFF	r0x01, POSTDEC1
	MOVFF	r0x02, POSTDEC1
	MOVFF	r0x03, POSTDEC1
	BANKSEL	_button1
;	.line	343; reveil.c	if (button1) {
	MOVF	_button1, W, B
	BTFSC	STATUS, 2
	BRA	_00303_DS_
;	.line	344; reveil.c	switch (whereami) {
	MOVLW	0x01
	BANKSEL	_whereami
	SUBWF	_whereami, W, B
	BTFSS	STATUS, 0
	BRA	_00267_DS_
	MOVLW	0x0c
	BANKSEL	_whereami
	SUBWF	_whereami, W, B
	BTFSC	STATUS, 0
	BRA	_00267_DS_
	BANKSEL	_whereami
	DECF	_whereami, W, B
	MOVWF	r0x00
	MOVFF	r0x04, POSTDEC1
	MOVFF	r0x05, POSTDEC1
	CLRF	r0x05
	RLCF	r0x00, W
	RLCF	r0x05, F
	RLCF	WREG, W
	RLCF	r0x05, F
	ANDLW	0xfc
	MOVWF	r0x04
	MOVLW	UPPER(_00322_DS_)
	MOVWF	PCLATU
	MOVLW	HIGH(_00322_DS_)
	MOVWF	PCLATH
	MOVLW	LOW(_00322_DS_)
	ADDWF	r0x04, F
	MOVF	r0x05, W
	ADDWFC	PCLATH, F
	BTFSC	STATUS, 0
	INCF	PCLATU, F
	MOVF	r0x04, W
	MOVFF	PREINC1, r0x05
	MOVFF	PREINC1, r0x04
	MOVWF	PCL
_00322_DS_:
	GOTO	_00255_DS_
	GOTO	_00256_DS_
	GOTO	_00257_DS_
	GOTO	_00258_DS_
	GOTO	_00259_DS_
	GOTO	_00260_DS_
	GOTO	_00261_DS_
	GOTO	_00262_DS_
	GOTO	_00263_DS_
	GOTO	_00264_DS_
	GOTO	_00265_DS_
_00255_DS_:
;	.line	346; reveil.c	whereami = ALARM_MENU;
	MOVLW	0x05
	BANKSEL	_whereami
	MOVWF	_whereami, B
;	.line	347; reveil.c	break;
	BRA	_00267_DS_
_00256_DS_:
;	.line	349; reveil.c	whereami = SET_MINUTE;
	MOVLW	0x03
	BANKSEL	_whereami
	MOVWF	_whereami, B
;	.line	350; reveil.c	break;
	BRA	_00267_DS_
_00257_DS_:
;	.line	352; reveil.c	whereami = SET_SECOND;
	MOVLW	0x04
	BANKSEL	_whereami
	MOVWF	_whereami, B
;	.line	353; reveil.c	break;
	BRA	_00267_DS_
_00258_DS_:
;	.line	355; reveil.c	whereami = ALARM_MENU;
	MOVLW	0x05
	BANKSEL	_whereami
	MOVWF	_whereami, B
;	.line	356; reveil.c	break;
	BRA	_00267_DS_
_00259_DS_:
;	.line	358; reveil.c	whereami = DISPLAY;
	MOVLW	0x09
	BANKSEL	_whereami
	MOVWF	_whereami, B
;	.line	359; reveil.c	break;
	BRA	_00267_DS_
_00260_DS_:
;	.line	361; reveil.c	whereami = SET_A_HOUR;
	MOVLW	0x07
	BANKSEL	_whereami
	MOVWF	_whereami, B
;	.line	362; reveil.c	break;
	BRA	_00267_DS_
_00261_DS_:
;	.line	364; reveil.c	whereami = SET_A_MIN;
	MOVLW	0x08
	BANKSEL	_whereami
	MOVWF	_whereami, B
;	.line	365; reveil.c	break;
	BRA	_00267_DS_
_00262_DS_:
;	.line	367; reveil.c	whereami = DISPLAY;
	MOVLW	0x09
	BANKSEL	_whereami
	MOVWF	_whereami, B
;	.line	368; reveil.c	break;
	BRA	_00267_DS_
_00263_DS_:
;	.line	370; reveil.c	whereami = TIME_MENU;
	MOVLW	0x01
	BANKSEL	_whereami
	MOVWF	_whereami, B
;	.line	371; reveil.c	break;
	BRA	_00267_DS_
_00264_DS_:
;	.line	373; reveil.c	stop_ringing = 1; // l'alarme doit s'arrêter
	MOVLW	0x01
	BANKSEL	_stop_ringing
	MOVWF	_stop_ringing, B
;	.line	374; reveil.c	LATJbits.LATJ1 = 0; // switch LED 2 off
	BCF	_LATJbits, 1
;	.line	375; reveil.c	LATJbits.LATJ2 = 0; // switch LED 3 off
	BCF	_LATJbits, 2
;	.line	376; reveil.c	whereami = DISPLAY;
	MOVLW	0x09
	BANKSEL	_whereami
	MOVWF	_whereami, B
;	.line	377; reveil.c	break;
	BRA	_00267_DS_
_00265_DS_:
;	.line	379; reveil.c	stop_ringing = 1; // l'alarme doit s'arrêter
	MOVLW	0x01
	BANKSEL	_stop_ringing
	MOVWF	_stop_ringing, B
;	.line	380; reveil.c	amin = amin_o; // remet le réveil à l'heure d'origine
	MOVFF	_amin_o, _amin
;	.line	381; reveil.c	ahour = ahour_o;
	MOVFF	_ahour_o, _ahour
	BANKSEL	_snooze
;	.line	382; reveil.c	snooze = 0;
	CLRF	_snooze, B
;	.line	383; reveil.c	LATJbits.LATJ1 = 0; // switch LED 2 off
	BCF	_LATJbits, 1
;	.line	384; reveil.c	LATJbits.LATJ2 = 0; // switch LED 3 off
	BCF	_LATJbits, 2
;	.line	385; reveil.c	whereami = DISPLAY;
	MOVLW	0x09
	BANKSEL	_whereami
	MOVWF	_whereami, B
_00267_DS_:
	BANKSEL	_button1
;	.line	390; reveil.c	button1 = 0; // remet le flag du bouton 1 à 0
	CLRF	_button1, B
	GOTO	_00305_DS_
_00303_DS_:
	BANKSEL	_button2
;	.line	393; reveil.c	} else if (button2) {
	MOVF	_button2, W, B
	BTFSC	STATUS, 2
	GOTO	_00305_DS_
;	.line	394; reveil.c	switch (whereami) {
	MOVLW	0x01
	BANKSEL	_whereami
	SUBWF	_whereami, W, B
	BTFSS	STATUS, 0
	GOTO	_00299_DS_
	MOVLW	0x0c
	BANKSEL	_whereami
	SUBWF	_whereami, W, B
	BTFSC	STATUS, 0
	GOTO	_00299_DS_
	BANKSEL	_whereami
	DECF	_whereami, W, B
	MOVWF	r0x00
	MOVFF	r0x04, POSTDEC1
	MOVFF	r0x05, POSTDEC1
	CLRF	r0x05
	RLCF	r0x00, W
	RLCF	r0x05, F
	RLCF	WREG, W
	RLCF	r0x05, F
	ANDLW	0xfc
	MOVWF	r0x04
	MOVLW	UPPER(_00325_DS_)
	MOVWF	PCLATU
	MOVLW	HIGH(_00325_DS_)
	MOVWF	PCLATH
	MOVLW	LOW(_00325_DS_)
	ADDWF	r0x04, F
	MOVF	r0x05, W
	ADDWFC	PCLATH, F
	BTFSC	STATUS, 0
	INCF	PCLATU, F
	MOVF	r0x04, W
	MOVFF	PREINC1, r0x05
	MOVFF	PREINC1, r0x04
	MOVWF	PCL
_00325_DS_:
	GOTO	_00268_DS_
	GOTO	_00269_DS_
	GOTO	_00273_DS_
	GOTO	_00277_DS_
	GOTO	_00281_DS_
	GOTO	_00282_DS_
	GOTO	_00283_DS_
	GOTO	_00287_DS_
	GOTO	_00291_DS_
	GOTO	_00292_DS_
	GOTO	_00295_DS_
_00268_DS_:
;	.line	396; reveil.c	whereami = SET_HOUR;
	MOVLW	0x02
	BANKSEL	_whereami
	MOVWF	_whereami, B
;	.line	397; reveil.c	break;
	GOTO	_00299_DS_
_00269_DS_:
	BANKSEL	_thour
;	.line	399; reveil.c	if (thour == 23) // l'heure doit repasser à 0
	MOVF	_thour, W, B
	XORLW	0x17
	BZ	_00327_DS_
	BRA	_00271_DS_
_00327_DS_:
	BANKSEL	(_overflows + 3)
;	.line	400; reveil.c	overflows -= 59*F*3600;
	MOVF	(_overflows + 3), W, B
	MOVWF	POSTDEC1
	BANKSEL	(_overflows + 2)
	MOVF	(_overflows + 2), W, B
	MOVWF	POSTDEC1
	BANKSEL	(_overflows + 1)
	MOVF	(_overflows + 1), W, B
	MOVWF	POSTDEC1
	BANKSEL	_overflows
	MOVF	_overflows, W, B
	MOVWF	POSTDEC1
	CALL	___ulong2fs
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
	MOVFF	PRODH, r0x02
	MOVFF	FSR0L, r0x03
	MOVLW	0x04
	ADDWF	FSR1L, F
	MOVLW	0x4b
	MOVWF	POSTDEC1
	MOVLW	0x9a
	MOVWF	POSTDEC1
	MOVLW	0x89
	MOVWF	POSTDEC1
	MOVLW	0xfd
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	___fssub
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
	MOVFF	PRODH, r0x02
	MOVFF	FSR0L, r0x03
	MOVLW	0x08
	ADDWF	FSR1L, F
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	___fs2ulong
	BANKSEL	_overflows
	MOVWF	_overflows, B
	MOVFF	PRODL, (_overflows + 1)
	MOVFF	PRODH, (_overflows + 2)
	MOVFF	FSR0L, (_overflows + 3)
	MOVLW	0x04
	ADDWF	FSR1L, F
	BRA	_00299_DS_
_00271_DS_:
	BANKSEL	(_overflows + 3)
;	.line	402; reveil.c	overflows += F*3600;
	MOVF	(_overflows + 3), W, B
	MOVWF	POSTDEC1
	BANKSEL	(_overflows + 2)
	MOVF	(_overflows + 2), W, B
	MOVWF	POSTDEC1
	BANKSEL	(_overflows + 1)
	MOVF	(_overflows + 1), W, B
	MOVWF	POSTDEC1
	BANKSEL	_overflows
	MOVF	_overflows, W, B
	MOVWF	POSTDEC1
	CALL	___ulong2fs
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
	MOVFF	PRODH, r0x02
	MOVFF	FSR0L, r0x03
	MOVLW	0x04
	ADDWF	FSR1L, F
	MOVLW	0x48
	MOVWF	POSTDEC1
	MOVLW	0xa7
	MOVWF	POSTDEC1
	MOVLW	0xa2
	MOVWF	POSTDEC1
	MOVLW	0xb3
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	___fsadd
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
	MOVFF	PRODH, r0x02
	MOVFF	FSR0L, r0x03
	MOVLW	0x08
	ADDWF	FSR1L, F
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	___fs2ulong
	BANKSEL	_overflows
	MOVWF	_overflows, B
	MOVFF	PRODL, (_overflows + 1)
	MOVFF	PRODH, (_overflows + 2)
	MOVFF	FSR0L, (_overflows + 3)
	MOVLW	0x04
	ADDWF	FSR1L, F
;	.line	403; reveil.c	break;
	BRA	_00299_DS_
_00273_DS_:
	BANKSEL	_tmin
;	.line	405; reveil.c	if (tmin == 59) // les minutes doivent repasser à 0
	MOVF	_tmin, W, B
	XORLW	0x3b
	BZ	_00329_DS_
	BRA	_00275_DS_
_00329_DS_:
	BANKSEL	(_overflows + 3)
;	.line	406; reveil.c	overflows -= 59*F*60;
	MOVF	(_overflows + 3), W, B
	MOVWF	POSTDEC1
	BANKSEL	(_overflows + 2)
	MOVF	(_overflows + 2), W, B
	MOVWF	POSTDEC1
	BANKSEL	(_overflows + 1)
	MOVF	(_overflows + 1), W, B
	MOVWF	POSTDEC1
	BANKSEL	_overflows
	MOVF	_overflows, W, B
	MOVWF	POSTDEC1
	CALL	___ulong2fs
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
	MOVFF	PRODH, r0x02
	MOVFF	FSR0L, r0x03
	MOVLW	0x04
	ADDWF	FSR1L, F
	MOVLW	0x48
	MOVWF	POSTDEC1
	MOVLW	0xa4
	MOVWF	POSTDEC1
	MOVLW	0xd7
	MOVWF	POSTDEC1
	MOVLW	0x74
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	___fssub
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
	MOVFF	PRODH, r0x02
	MOVFF	FSR0L, r0x03
	MOVLW	0x08
	ADDWF	FSR1L, F
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	___fs2ulong
	BANKSEL	_overflows
	MOVWF	_overflows, B
	MOVFF	PRODL, (_overflows + 1)
	MOVFF	PRODH, (_overflows + 2)
	MOVFF	FSR0L, (_overflows + 3)
	MOVLW	0x04
	ADDWF	FSR1L, F
	BRA	_00299_DS_
_00275_DS_:
	BANKSEL	(_overflows + 3)
;	.line	408; reveil.c	overflows += F*60;
	MOVF	(_overflows + 3), W, B
	MOVWF	POSTDEC1
	BANKSEL	(_overflows + 2)
	MOVF	(_overflows + 2), W, B
	MOVWF	POSTDEC1
	BANKSEL	(_overflows + 1)
	MOVF	(_overflows + 1), W, B
	MOVWF	POSTDEC1
	BANKSEL	_overflows
	MOVF	_overflows, W, B
	MOVWF	POSTDEC1
	CALL	___ulong2fs
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
	MOVFF	PRODH, r0x02
	MOVFF	FSR0L, r0x03
	MOVLW	0x04
	ADDWF	FSR1L, F
	MOVLW	0x45
	MOVWF	POSTDEC1
	MOVLW	0xb2
	MOVWF	POSTDEC1
	MOVLW	0xcf
	MOVWF	POSTDEC1
	MOVLW	0xae
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	___fsadd
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
	MOVFF	PRODH, r0x02
	MOVFF	FSR0L, r0x03
	MOVLW	0x08
	ADDWF	FSR1L, F
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	___fs2ulong
	BANKSEL	_overflows
	MOVWF	_overflows, B
	MOVFF	PRODL, (_overflows + 1)
	MOVFF	PRODH, (_overflows + 2)
	MOVFF	FSR0L, (_overflows + 3)
	MOVLW	0x04
	ADDWF	FSR1L, F
;	.line	409; reveil.c	break;
	BRA	_00299_DS_
_00277_DS_:
	BANKSEL	_tsec
;	.line	411; reveil.c	if (tsec == 59) // les secondes doivent repasser à 0
	MOVF	_tsec, W, B
	XORLW	0x3b
	BZ	_00331_DS_
	BRA	_00279_DS_
_00331_DS_:
	BANKSEL	(_overflows + 3)
;	.line	412; reveil.c	overflows -= 59*F;
	MOVF	(_overflows + 3), W, B
	MOVWF	POSTDEC1
	BANKSEL	(_overflows + 2)
	MOVF	(_overflows + 2), W, B
	MOVWF	POSTDEC1
	BANKSEL	(_overflows + 1)
	MOVF	(_overflows + 1), W, B
	MOVWF	POSTDEC1
	BANKSEL	_overflows
	MOVF	_overflows, W, B
	MOVWF	POSTDEC1
	CALL	___ulong2fs
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
	MOVFF	PRODH, r0x02
	MOVFF	FSR0L, r0x03
	MOVLW	0x04
	ADDWF	FSR1L, F
	MOVLW	0x45
	MOVWF	POSTDEC1
	MOVLW	0xaf
	MOVWF	POSTDEC1
	MOVLW	0xd4
	MOVWF	POSTDEC1
	MOVLW	0xc1
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	___fssub
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
	MOVFF	PRODH, r0x02
	MOVFF	FSR0L, r0x03
	MOVLW	0x08
	ADDWF	FSR1L, F
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	___fs2ulong
	BANKSEL	_overflows
	MOVWF	_overflows, B
	MOVFF	PRODL, (_overflows + 1)
	MOVFF	PRODH, (_overflows + 2)
	MOVFF	FSR0L, (_overflows + 3)
	MOVLW	0x04
	ADDWF	FSR1L, F
	BRA	_00299_DS_
_00279_DS_:
	BANKSEL	(_overflows + 3)
;	.line	414; reveil.c	overflows += F;
	MOVF	(_overflows + 3), W, B
	MOVWF	POSTDEC1
	BANKSEL	(_overflows + 2)
	MOVF	(_overflows + 2), W, B
	MOVWF	POSTDEC1
	BANKSEL	(_overflows + 1)
	MOVF	(_overflows + 1), W, B
	MOVWF	POSTDEC1
	BANKSEL	_overflows
	MOVF	_overflows, W, B
	MOVWF	POSTDEC1
	CALL	___ulong2fs
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
	MOVFF	PRODH, r0x02
	MOVFF	FSR0L, r0x03
	MOVLW	0x04
	ADDWF	FSR1L, F
	MOVLW	0x42
	MOVWF	POSTDEC1
	MOVLW	0xbe
	MOVWF	POSTDEC1
	MOVLW	0xbb
	MOVWF	POSTDEC1
	MOVLW	0x64
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	___fsadd
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
	MOVFF	PRODH, r0x02
	MOVFF	FSR0L, r0x03
	MOVLW	0x08
	ADDWF	FSR1L, F
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	___fs2ulong
	BANKSEL	_overflows
	MOVWF	_overflows, B
	MOVFF	PRODL, (_overflows + 1)
	MOVFF	PRODH, (_overflows + 2)
	MOVFF	FSR0L, (_overflows + 3)
	MOVLW	0x04
	ADDWF	FSR1L, F
;	.line	415; reveil.c	break;
	BRA	_00299_DS_
_00281_DS_:
;	.line	417; reveil.c	whereami = SET_ALARM;
	MOVLW	0x06
	BANKSEL	_whereami
	MOVWF	_whereami, B
;	.line	418; reveil.c	break;
	BRA	_00299_DS_
_00282_DS_:
;	.line	420; reveil.c	alarm_set ^= 1;
	MOVLW	0x01
	BANKSEL	_alarm_set
	XORWF	_alarm_set, F, B
;	.line	421; reveil.c	break;
	BRA	_00299_DS_
_00283_DS_:
	BANKSEL	_ahour
;	.line	423; reveil.c	if (ahour == 23) // l'heure d'alarme doit repasser à 0
	MOVF	_ahour, W, B
	XORLW	0x17
	BNZ	_00285_DS_
_00334_DS_:
	BANKSEL	_ahour
;	.line	424; reveil.c	ahour = 0;
	CLRF	_ahour, B
	BRA	_00286_DS_
_00285_DS_:
	BANKSEL	_ahour
;	.line	426; reveil.c	ahour++;
	INCF	_ahour, F, B
_00286_DS_:
;	.line	427; reveil.c	ahour_o = ahour;
	MOVFF	_ahour, _ahour_o
;	.line	428; reveil.c	break;
	BRA	_00299_DS_
_00287_DS_:
	BANKSEL	_amin
;	.line	430; reveil.c	if (amin == 59) // les minutes d'alarme doivent repasser à 0
	MOVF	_amin, W, B
	XORLW	0x3b
	BNZ	_00289_DS_
_00336_DS_:
	BANKSEL	_amin
;	.line	431; reveil.c	amin = 0;
	CLRF	_amin, B
	BRA	_00290_DS_
_00289_DS_:
	BANKSEL	_amin
;	.line	433; reveil.c	amin++;
	INCF	_amin, F, B
_00290_DS_:
;	.line	434; reveil.c	amin_o = amin;
	MOVFF	_amin, _amin_o
;	.line	435; reveil.c	break;
	BRA	_00299_DS_
_00291_DS_:
;	.line	437; reveil.c	break; // bouton sans fonction
	BRA	_00299_DS_
_00292_DS_:
;	.line	440; reveil.c	if (snooze < SNOOZE_MAX) {
	MOVLW	0x0c
	BANKSEL	_snooze
	SUBWF	_snooze, W, B
	BC	_00299_DS_
;	.line	441; reveil.c	inc_amin(SNOOZE_MINUTE); // modifie le réveil
	MOVLW	0x05
	MOVWF	POSTDEC1
	CALL	_inc_amin
	INCF	FSR1L, F
	BANKSEL	_snooze
;	.line	442; reveil.c	snooze++; // augmente le compteur de snooze
	INCF	_snooze, F, B
;	.line	443; reveil.c	LATJbits.LATJ1 = 0; // switch LED 2 off
	BCF	_LATJbits, 1
;	.line	444; reveil.c	LATJbits.LATJ2 = 0; // switch LED 3 off
	BCF	_LATJbits, 2
;	.line	445; reveil.c	whereami = SNOOZE;
	MOVLW	0x0b
	BANKSEL	_whereami
	MOVWF	_whereami, B
;	.line	448; reveil.c	break;
	BRA	_00299_DS_
_00295_DS_:
;	.line	451; reveil.c	if (snooze < SNOOZE_MAX) {
	MOVLW	0x0c
	BANKSEL	_snooze
	SUBWF	_snooze, W, B
	BC	_00299_DS_
;	.line	452; reveil.c	inc_amin(SNOOZE_MINUTE); // modifie le réveil
	MOVLW	0x05
	MOVWF	POSTDEC1
	CALL	_inc_amin
	INCF	FSR1L, F
	BANKSEL	_snooze
;	.line	453; reveil.c	snooze++; // augmente le compteur de snooze
	INCF	_snooze, F, B
_00299_DS_:
	BANKSEL	_button2
;	.line	460; reveil.c	button2 = 0; // remet le flag du bouton 2 à 0
	CLRF	_button2, B
_00305_DS_:
	MOVFF	PREINC1, r0x03
	MOVFF	PREINC1, r0x02
	MOVFF	PREINC1, r0x01
	MOVFF	PREINC1, r0x00
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_reveil__alarm	code
_alarm:
;	.line	305; reveil.c	void alarm(void)
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	BANKSEL	_thour
;	.line	308; reveil.c	if (thour == ahour && tmin == amin && alarm_set) {
	MOVF	_thour, W, B
	BANKSEL	_ahour
	XORWF	_ahour, W, B
	BZ	_00240_DS_
	BRA	_00228_DS_
_00240_DS_:
	BANKSEL	_tmin
	MOVF	_tmin, W, B
	BANKSEL	_amin
	XORWF	_amin, W, B
	BZ	_00242_DS_
	BRA	_00228_DS_
_00242_DS_:
	BANKSEL	_alarm_set
	MOVF	_alarm_set, W, B
	BZ	_00228_DS_
;	.line	312; reveil.c	if (tsec < 31 && stop_ringing == 0) {
	MOVLW	0x1f
	BANKSEL	_tsec
	SUBWF	_tsec, W, B
	BC	_00221_DS_
	BANKSEL	_stop_ringing
	MOVF	_stop_ringing, W, B
	BNZ	_00221_DS_
	BANKSEL	_whereami
;	.line	313; reveil.c	if (whereami == DISPLAY || whereami == SNOOZE)
	MOVF	_whereami, W, B
	XORLW	0x09
	BZ	_00210_DS_
_00245_DS_:
	BANKSEL	_whereami
	MOVF	_whereami, W, B
	XORLW	0x0b
	BNZ	_00228_DS_
_00210_DS_:
;	.line	314; reveil.c	whereami = ALARM;
	MOVLW	0x0a
	BANKSEL	_whereami
	MOVWF	_whereami, B
	BRA	_00228_DS_
_00221_DS_:
;	.line	317; reveil.c	} else if (tsec > 30) {
	MOVLW	0x1f
	BANKSEL	_tsec
	SUBWF	_tsec, W, B
	BNC	_00228_DS_
	BANKSEL	_stop_ringing
;	.line	318; reveil.c	stop_ringing = 0; // l'alarme doit sonner à nouveau dans 24h
	CLRF	_stop_ringing, B
;	.line	319; reveil.c	LATJbits.LATJ1 = 0; // switch LED 2 off
	BCF	_LATJbits, 1
;	.line	320; reveil.c	LATJbits.LATJ2 = 0; // switch LED 3 off
	BCF	_LATJbits, 2
	BANKSEL	_snooze
;	.line	323; reveil.c	if (snooze) {
	MOVF	_snooze, W, B
	BZ	_00216_DS_
;	.line	324; reveil.c	ahour = ahour_o; // remet le réveil à l'heure d'origine
	MOVFF	_ahour_o, _ahour
;	.line	325; reveil.c	amin = amin_o;
	MOVFF	_amin_o, _amin
	BANKSEL	_snooze
;	.line	326; reveil.c	snooze = 0;
	CLRF	_snooze, B
;	.line	327; reveil.c	whereami = DISPLAY;
	MOVLW	0x09
	BANKSEL	_whereami
	MOVWF	_whereami, B
	BRA	_00228_DS_
_00216_DS_:
	BANKSEL	_whereami
;	.line	328; reveil.c	} else if (whereami == ALARM) // si l'alarme sonnait toujours, on
	MOVF	_whereami, W, B
	XORLW	0x0a
	BNZ	_00228_DS_
;	.line	329; reveil.c	whereami = DISPLAY;       // revient à l'affichage de l'heure
	MOVLW	0x09
	BANKSEL	_whereami
	MOVWF	_whereami, B
_00228_DS_:
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_reveil__refresh_lcd	code
_refresh_lcd:
;	.line	240; reveil.c	void refresh_lcd(void)
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	MOVFF	r0x00, POSTDEC1
	MOVFF	r0x01, POSTDEC1
	MOVFF	r0x02, POSTDEC1
	MOVFF	r0x03, POSTDEC1
	MOVFF	r0x04, POSTDEC1
	MOVFF	r0x05, POSTDEC1
	MOVFF	r0x06, POSTDEC1
	MOVFF	r0x07, POSTDEC1
	MOVFF	r0x08, POSTDEC1
	MOVFF	r0x09, POSTDEC1
	MOVFF	r0x0a, POSTDEC1
	MOVFF	r0x0b, POSTDEC1
	MOVFF	r0x0c, POSTDEC1
	MOVFF	r0x0d, POSTDEC1
	MOVFF	r0x0e, POSTDEC1
;	.line	243; reveil.c	switch (whereami) {
	MOVLW	0x01
	BANKSEL	_whereami
	SUBWF	_whereami, W, B
	BTFSS	STATUS, 0
	GOTO	_00193_DS_
	MOVLW	0x0c
	BANKSEL	_whereami
	SUBWF	_whereami, W, B
	BTFSC	STATUS, 0
	GOTO	_00193_DS_
	BANKSEL	_whereami
	DECF	_whereami, W, B
	MOVWF	r0x00
	MOVFF	r0x0f, POSTDEC1
	MOVFF	r0x10, POSTDEC1
	CLRF	r0x10
	RLCF	r0x00, W
	RLCF	r0x10, F
	RLCF	WREG, W
	RLCF	r0x10, F
	ANDLW	0xfc
	MOVWF	r0x0f
	MOVLW	UPPER(_00204_DS_)
	MOVWF	PCLATU
	MOVLW	HIGH(_00204_DS_)
	MOVWF	PCLATH
	MOVLW	LOW(_00204_DS_)
	ADDWF	r0x0f, F
	MOVF	r0x10, W
	ADDWFC	PCLATH, F
	BTFSC	STATUS, 0
	INCF	PCLATU, F
	MOVF	r0x0f, W
	MOVFF	PREINC1, r0x10
	MOVFF	PREINC1, r0x0f
	MOVWF	PCL
_00204_DS_:
	GOTO	_00173_DS_
	GOTO	_00174_DS_
	GOTO	_00175_DS_
	GOTO	_00176_DS_
	GOTO	_00177_DS_
	GOTO	_00178_DS_
	GOTO	_00182_DS_
	GOTO	_00183_DS_
	GOTO	_00184_DS_
	GOTO	_00188_DS_
	GOTO	_00189_DS_
_00173_DS_:
;	.line	245; reveil.c	sprintf(display, "Do you want to  set the time ?  ");
	MOVLW	HIGH(_display)
	MOVWF	r0x01
	MOVLW	LOW(_display)
	MOVWF	r0x00
	MOVLW	0x80
	MOVWF	r0x02
	MOVLW	UPPER(__str_0)
	MOVWF	POSTDEC1
	MOVLW	HIGH(__str_0)
	MOVWF	POSTDEC1
	MOVLW	LOW(__str_0)
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	_sprintf
	MOVLW	0x06
	ADDWF	FSR1L, F
;	.line	246; reveil.c	break;
	GOTO	_00194_DS_
_00174_DS_:
;	.line	249; reveil.c	thour, tmin, tsec);
	MOVFF	_tsec, r0x00
	CLRF	r0x01
	MOVFF	_tmin, r0x02
	CLRF	r0x03
	MOVFF	_thour, r0x04
	CLRF	r0x05
;	.line	248; reveil.c	sprintf(display, " [%02u]: %02u : %02u                  ",
	MOVLW	HIGH(_display)
	MOVWF	r0x07
	MOVLW	LOW(_display)
	MOVWF	r0x06
	MOVLW	0x80
	MOVWF	r0x08
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x05, W
	MOVWF	POSTDEC1
	MOVF	r0x04, W
	MOVWF	POSTDEC1
	MOVLW	UPPER(__str_1)
	MOVWF	POSTDEC1
	MOVLW	HIGH(__str_1)
	MOVWF	POSTDEC1
	MOVLW	LOW(__str_1)
	MOVWF	POSTDEC1
	MOVF	r0x08, W
	MOVWF	POSTDEC1
	MOVF	r0x07, W
	MOVWF	POSTDEC1
	MOVF	r0x06, W
	MOVWF	POSTDEC1
	CALL	_sprintf
	MOVLW	0x0c
	ADDWF	FSR1L, F
;	.line	250; reveil.c	break;
	GOTO	_00194_DS_
_00175_DS_:
;	.line	253; reveil.c	thour, tmin, tsec);
	MOVFF	_tsec, r0x00
	CLRF	r0x01
	MOVFF	_tmin, r0x02
	CLRF	r0x03
	MOVFF	_thour, r0x04
	CLRF	r0x05
;	.line	252; reveil.c	sprintf(display, "  %02u :[%02u]: %02u                  ",
	MOVLW	HIGH(_display)
	MOVWF	r0x07
	MOVLW	LOW(_display)
	MOVWF	r0x06
	MOVLW	0x80
	MOVWF	r0x08
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x05, W
	MOVWF	POSTDEC1
	MOVF	r0x04, W
	MOVWF	POSTDEC1
	MOVLW	UPPER(__str_2)
	MOVWF	POSTDEC1
	MOVLW	HIGH(__str_2)
	MOVWF	POSTDEC1
	MOVLW	LOW(__str_2)
	MOVWF	POSTDEC1
	MOVF	r0x08, W
	MOVWF	POSTDEC1
	MOVF	r0x07, W
	MOVWF	POSTDEC1
	MOVF	r0x06, W
	MOVWF	POSTDEC1
	CALL	_sprintf
	MOVLW	0x0c
	ADDWF	FSR1L, F
;	.line	254; reveil.c	break;
	BRA	_00194_DS_
_00176_DS_:
;	.line	257; reveil.c	thour, tmin, tsec);
	MOVFF	_tsec, r0x00
	CLRF	r0x01
	MOVFF	_tmin, r0x02
	CLRF	r0x03
	MOVFF	_thour, r0x04
	CLRF	r0x05
;	.line	256; reveil.c	sprintf(display, "  %02u : %02u :[%02u]                 ",
	MOVLW	HIGH(_display)
	MOVWF	r0x07
	MOVLW	LOW(_display)
	MOVWF	r0x06
	MOVLW	0x80
	MOVWF	r0x08
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x05, W
	MOVWF	POSTDEC1
	MOVF	r0x04, W
	MOVWF	POSTDEC1
	MOVLW	UPPER(__str_3)
	MOVWF	POSTDEC1
	MOVLW	HIGH(__str_3)
	MOVWF	POSTDEC1
	MOVLW	LOW(__str_3)
	MOVWF	POSTDEC1
	MOVF	r0x08, W
	MOVWF	POSTDEC1
	MOVF	r0x07, W
	MOVWF	POSTDEC1
	MOVF	r0x06, W
	MOVWF	POSTDEC1
	CALL	_sprintf
	MOVLW	0x0c
	ADDWF	FSR1L, F
;	.line	258; reveil.c	break;
	BRA	_00194_DS_
_00177_DS_:
;	.line	260; reveil.c	sprintf(display, "Do you want to  set the alarm ? ");
	MOVLW	HIGH(_display)
	MOVWF	r0x01
	MOVLW	LOW(_display)
	MOVWF	r0x00
	MOVLW	0x80
	MOVWF	r0x02
	MOVLW	UPPER(__str_4)
	MOVWF	POSTDEC1
	MOVLW	HIGH(__str_4)
	MOVWF	POSTDEC1
	MOVLW	LOW(__str_4)
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	_sprintf
	MOVLW	0x06
	ADDWF	FSR1L, F
;	.line	261; reveil.c	break;
	BRA	_00194_DS_
_00178_DS_:
	BANKSEL	_alarm_set
;	.line	263; reveil.c	if (alarm_set)
	MOVF	_alarm_set, W, B
	BZ	_00180_DS_
;	.line	264; reveil.c	sprintf(display, "  Alarm [ON ]                   ");
	MOVLW	HIGH(_display)
	MOVWF	r0x01
	MOVLW	LOW(_display)
	MOVWF	r0x00
	MOVLW	0x80
	MOVWF	r0x02
	MOVLW	UPPER(__str_5)
	MOVWF	POSTDEC1
	MOVLW	HIGH(__str_5)
	MOVWF	POSTDEC1
	MOVLW	LOW(__str_5)
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	_sprintf
	MOVLW	0x06
	ADDWF	FSR1L, F
	BRA	_00194_DS_
_00180_DS_:
;	.line	266; reveil.c	sprintf(display, "  Alarm [OFF]                   ");
	MOVLW	HIGH(_display)
	MOVWF	r0x01
	MOVLW	LOW(_display)
	MOVWF	r0x00
	MOVLW	0x80
	MOVWF	r0x02
	MOVLW	UPPER(__str_6)
	MOVWF	POSTDEC1
	MOVLW	HIGH(__str_6)
	MOVWF	POSTDEC1
	MOVLW	LOW(__str_6)
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	_sprintf
	MOVLW	0x06
	ADDWF	FSR1L, F
;	.line	267; reveil.c	break;
	BRA	_00194_DS_
_00182_DS_:
;	.line	270; reveil.c	ahour, amin);
	MOVFF	_amin, r0x00
	CLRF	r0x01
	MOVFF	_ahour, r0x02
	CLRF	r0x03
;	.line	269; reveil.c	sprintf(display, "    Alarm at        [%02u]: %02u    ",
	MOVLW	HIGH(_display)
	MOVWF	r0x05
	MOVLW	LOW(_display)
	MOVWF	r0x04
	MOVLW	0x80
	MOVWF	r0x06
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVLW	UPPER(__str_7)
	MOVWF	POSTDEC1
	MOVLW	HIGH(__str_7)
	MOVWF	POSTDEC1
	MOVLW	LOW(__str_7)
	MOVWF	POSTDEC1
	MOVF	r0x06, W
	MOVWF	POSTDEC1
	MOVF	r0x05, W
	MOVWF	POSTDEC1
	MOVF	r0x04, W
	MOVWF	POSTDEC1
	CALL	_sprintf
	MOVLW	0x0a
	ADDWF	FSR1L, F
;	.line	271; reveil.c	break;
	BRA	_00194_DS_
_00183_DS_:
;	.line	274; reveil.c	ahour, amin);
	MOVFF	_amin, r0x00
	CLRF	r0x01
	MOVFF	_ahour, r0x02
	CLRF	r0x03
;	.line	273; reveil.c	sprintf(display, "    Alarm at         %02u :[%02u]   ",
	MOVLW	HIGH(_display)
	MOVWF	r0x05
	MOVLW	LOW(_display)
	MOVWF	r0x04
	MOVLW	0x80
	MOVWF	r0x06
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVLW	UPPER(__str_8)
	MOVWF	POSTDEC1
	MOVLW	HIGH(__str_8)
	MOVWF	POSTDEC1
	MOVLW	LOW(__str_8)
	MOVWF	POSTDEC1
	MOVF	r0x06, W
	MOVWF	POSTDEC1
	MOVF	r0x05, W
	MOVWF	POSTDEC1
	MOVF	r0x04, W
	MOVWF	POSTDEC1
	CALL	_sprintf
	MOVLW	0x0a
	ADDWF	FSR1L, F
;	.line	275; reveil.c	break;
	BRA	_00194_DS_
_00184_DS_:
	BANKSEL	_alarm_set
;	.line	277; reveil.c	if (alarm_set)
	MOVF	_alarm_set, W, B
	BZ	_00186_DS_
;	.line	279; reveil.c	thour, tmin, tsec, ahour, amin);
	MOVFF	_amin, r0x00
	CLRF	r0x01
	MOVFF	_ahour, r0x02
	CLRF	r0x03
	MOVFF	_tsec, r0x04
	CLRF	r0x05
	MOVFF	_tmin, r0x06
	CLRF	r0x07
	MOVFF	_thour, r0x08
	CLRF	r0x09
;	.line	278; reveil.c	sprintf(display, "    %02u:%02u:%02u    Alarm ON  %02u:%02u ",
	MOVLW	HIGH(_display)
	MOVWF	r0x0b
	MOVLW	LOW(_display)
	MOVWF	r0x0a
	MOVLW	0x80
	MOVWF	r0x0c
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x05, W
	MOVWF	POSTDEC1
	MOVF	r0x04, W
	MOVWF	POSTDEC1
	MOVF	r0x07, W
	MOVWF	POSTDEC1
	MOVF	r0x06, W
	MOVWF	POSTDEC1
	MOVF	r0x09, W
	MOVWF	POSTDEC1
	MOVF	r0x08, W
	MOVWF	POSTDEC1
	MOVLW	UPPER(__str_9)
	MOVWF	POSTDEC1
	MOVLW	HIGH(__str_9)
	MOVWF	POSTDEC1
	MOVLW	LOW(__str_9)
	MOVWF	POSTDEC1
	MOVF	r0x0c, W
	MOVWF	POSTDEC1
	MOVF	r0x0b, W
	MOVWF	POSTDEC1
	MOVF	r0x0a, W
	MOVWF	POSTDEC1
	CALL	_sprintf
	MOVLW	0x10
	ADDWF	FSR1L, F
	BRA	_00194_DS_
_00186_DS_:
;	.line	282; reveil.c	thour, tmin, tsec);
	MOVFF	_tsec, r0x00
	CLRF	r0x01
	MOVFF	_tmin, r0x02
	CLRF	r0x03
	MOVFF	_thour, r0x04
	CLRF	r0x05
;	.line	281; reveil.c	sprintf(display, "    %02u:%02u:%02u       Alarm  OFF   ",
	MOVLW	HIGH(_display)
	MOVWF	r0x07
	MOVLW	LOW(_display)
	MOVWF	r0x06
	MOVLW	0x80
	MOVWF	r0x08
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x05, W
	MOVWF	POSTDEC1
	MOVF	r0x04, W
	MOVWF	POSTDEC1
	MOVLW	UPPER(__str_10)
	MOVWF	POSTDEC1
	MOVLW	HIGH(__str_10)
	MOVWF	POSTDEC1
	MOVLW	LOW(__str_10)
	MOVWF	POSTDEC1
	MOVF	r0x08, W
	MOVWF	POSTDEC1
	MOVF	r0x07, W
	MOVWF	POSTDEC1
	MOVF	r0x06, W
	MOVWF	POSTDEC1
	CALL	_sprintf
	MOVLW	0x0c
	ADDWF	FSR1L, F
;	.line	283; reveil.c	break;
	BRA	_00194_DS_
_00188_DS_:
;	.line	286; reveil.c	thour, tmin, tsec);
	MOVFF	_tsec, r0x00
	CLRF	r0x01
	MOVFF	_tmin, r0x02
	CLRF	r0x03
	MOVFF	_thour, r0x04
	CLRF	r0x05
;	.line	285; reveil.c	sprintf(display, "    %02u:%02u:%02u      I am ringing! ",
	MOVLW	HIGH(_display)
	MOVWF	r0x07
	MOVLW	LOW(_display)
	MOVWF	r0x06
	MOVLW	0x80
	MOVWF	r0x08
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x05, W
	MOVWF	POSTDEC1
	MOVF	r0x04, W
	MOVWF	POSTDEC1
	MOVLW	UPPER(__str_11)
	MOVWF	POSTDEC1
	MOVLW	HIGH(__str_11)
	MOVWF	POSTDEC1
	MOVLW	LOW(__str_11)
	MOVWF	POSTDEC1
	MOVF	r0x08, W
	MOVWF	POSTDEC1
	MOVF	r0x07, W
	MOVWF	POSTDEC1
	MOVF	r0x06, W
	MOVWF	POSTDEC1
	CALL	_sprintf
	MOVLW	0x0c
	ADDWF	FSR1L, F
;	.line	287; reveil.c	break;
	BRA	_00194_DS_
_00189_DS_:
;	.line	289; reveil.c	if (snooze < 10)
	MOVLW	0x0a
	BANKSEL	_snooze
	SUBWF	_snooze, W, B
	BTFSC	STATUS, 0
	BRA	_00191_DS_
;	.line	291; reveil.c	thour, tmin, tsec, snooze, ahour_o, amin_o);
	MOVFF	_amin_o, r0x00
	CLRF	r0x01
	MOVFF	_ahour_o, r0x02
	CLRF	r0x03
	MOVFF	_snooze, r0x04
	CLRF	r0x05
	MOVFF	_tsec, r0x06
	CLRF	r0x07
	MOVFF	_tmin, r0x08
	CLRF	r0x09
	MOVFF	_thour, r0x0a
	CLRF	r0x0b
;	.line	290; reveil.c	sprintf(display, "    %02u:%02u:%02u    Snooze %u  %02u:%02u ",
	MOVLW	HIGH(_display)
	MOVWF	r0x0d
	MOVLW	LOW(_display)
	MOVWF	r0x0c
	MOVLW	0x80
	MOVWF	r0x0e
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x05, W
	MOVWF	POSTDEC1
	MOVF	r0x04, W
	MOVWF	POSTDEC1
	MOVF	r0x07, W
	MOVWF	POSTDEC1
	MOVF	r0x06, W
	MOVWF	POSTDEC1
	MOVF	r0x09, W
	MOVWF	POSTDEC1
	MOVF	r0x08, W
	MOVWF	POSTDEC1
	MOVF	r0x0b, W
	MOVWF	POSTDEC1
	MOVF	r0x0a, W
	MOVWF	POSTDEC1
	MOVLW	UPPER(__str_12)
	MOVWF	POSTDEC1
	MOVLW	HIGH(__str_12)
	MOVWF	POSTDEC1
	MOVLW	LOW(__str_12)
	MOVWF	POSTDEC1
	MOVF	r0x0e, W
	MOVWF	POSTDEC1
	MOVF	r0x0d, W
	MOVWF	POSTDEC1
	MOVF	r0x0c, W
	MOVWF	POSTDEC1
	CALL	_sprintf
	MOVLW	0x12
	ADDWF	FSR1L, F
	BRA	_00194_DS_
_00191_DS_:
;	.line	294; reveil.c	thour, tmin, tsec, snooze, ahour_o, amin_o);
	MOVFF	_amin_o, r0x00
	CLRF	r0x01
	MOVFF	_ahour_o, r0x02
	CLRF	r0x03
	MOVFF	_snooze, r0x04
	CLRF	r0x05
	MOVFF	_tsec, r0x06
	CLRF	r0x07
	MOVFF	_tmin, r0x08
	CLRF	r0x09
	MOVFF	_thour, r0x0a
	CLRF	r0x0b
;	.line	293; reveil.c	sprintf(display, "    %02u:%02u:%02u    Snooze %u %02u:%02u ",
	MOVLW	HIGH(_display)
	MOVWF	r0x0d
	MOVLW	LOW(_display)
	MOVWF	r0x0c
	MOVLW	0x80
	MOVWF	r0x0e
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x05, W
	MOVWF	POSTDEC1
	MOVF	r0x04, W
	MOVWF	POSTDEC1
	MOVF	r0x07, W
	MOVWF	POSTDEC1
	MOVF	r0x06, W
	MOVWF	POSTDEC1
	MOVF	r0x09, W
	MOVWF	POSTDEC1
	MOVF	r0x08, W
	MOVWF	POSTDEC1
	MOVF	r0x0b, W
	MOVWF	POSTDEC1
	MOVF	r0x0a, W
	MOVWF	POSTDEC1
	MOVLW	UPPER(__str_13)
	MOVWF	POSTDEC1
	MOVLW	HIGH(__str_13)
	MOVWF	POSTDEC1
	MOVLW	LOW(__str_13)
	MOVWF	POSTDEC1
	MOVF	r0x0e, W
	MOVWF	POSTDEC1
	MOVF	r0x0d, W
	MOVWF	POSTDEC1
	MOVF	r0x0c, W
	MOVWF	POSTDEC1
	CALL	_sprintf
	MOVLW	0x12
	ADDWF	FSR1L, F
;	.line	295; reveil.c	break;
	BRA	_00194_DS_
_00193_DS_:
;	.line	297; reveil.c	sprintf(display, "**** ERROR ********* ERROR *****");
	MOVLW	HIGH(_display)
	MOVWF	r0x01
	MOVLW	LOW(_display)
	MOVWF	r0x00
	MOVLW	0x80
	MOVWF	r0x02
	MOVLW	UPPER(__str_14)
	MOVWF	POSTDEC1
	MOVLW	HIGH(__str_14)
	MOVWF	POSTDEC1
	MOVLW	LOW(__str_14)
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	_sprintf
	MOVLW	0x06
	ADDWF	FSR1L, F
_00194_DS_:
;	.line	300; reveil.c	DisplayString(0, display);
	MOVLW	HIGH(_display)
	MOVWF	r0x01
	MOVLW	LOW(_display)
	MOVWF	r0x00
	MOVLW	0x80
	MOVWF	r0x02
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	MOVLW	0x00
	MOVWF	POSTDEC1
	CALL	_DisplayString
	MOVLW	0x04
	ADDWF	FSR1L, F
	MOVFF	PREINC1, r0x0e
	MOVFF	PREINC1, r0x0d
	MOVFF	PREINC1, r0x0c
	MOVFF	PREINC1, r0x0b
	MOVFF	PREINC1, r0x0a
	MOVFF	PREINC1, r0x09
	MOVFF	PREINC1, r0x08
	MOVFF	PREINC1, r0x07
	MOVFF	PREINC1, r0x06
	MOVFF	PREINC1, r0x05
	MOVFF	PREINC1, r0x04
	MOVFF	PREINC1, r0x03
	MOVFF	PREINC1, r0x02
	MOVFF	PREINC1, r0x01
	MOVFF	PREINC1, r0x00
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_reveil__time	code
_time:
;	.line	200; reveil.c	void time(void)
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	MOVFF	r0x00, POSTDEC1
	MOVFF	r0x01, POSTDEC1
	MOVFF	r0x02, POSTDEC1
	MOVFF	r0x03, POSTDEC1
	MOVFF	r0x04, POSTDEC1
	MOVFF	r0x05, POSTDEC1
	MOVFF	r0x06, POSTDEC1
	MOVFF	r0x07, POSTDEC1
	BANKSEL	(_overflows + 3)
;	.line	203; reveil.c	sec = overflows/F;
	MOVF	(_overflows + 3), W, B
	MOVWF	POSTDEC1
	BANKSEL	(_overflows + 2)
	MOVF	(_overflows + 2), W, B
	MOVWF	POSTDEC1
	BANKSEL	(_overflows + 1)
	MOVF	(_overflows + 1), W, B
	MOVWF	POSTDEC1
	BANKSEL	_overflows
	MOVF	_overflows, W, B
	MOVWF	POSTDEC1
	CALL	___ulong2fs
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
	MOVFF	PRODH, r0x02
	MOVFF	FSR0L, r0x03
	MOVLW	0x04
	ADDWF	FSR1L, F
	MOVLW	0x42
	MOVWF	POSTDEC1
	MOVLW	0xbe
	MOVWF	POSTDEC1
	MOVLW	0xbb
	MOVWF	POSTDEC1
	MOVLW	0x64
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	___fsdiv
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
	MOVFF	PRODH, r0x02
	MOVFF	FSR0L, r0x03
	MOVLW	0x08
	ADDWF	FSR1L, F
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	___fs2ulong
	BANKSEL	_sec
	MOVWF	_sec, B
	MOVFF	PRODL, (_sec + 1)
	MOVFF	PRODH, (_sec + 2)
	MOVFF	FSR0L, (_sec + 3)
	MOVLW	0x04
	ADDWF	FSR1L, F
	BANKSEL	(_overflows + 3)
;	.line	204; reveil.c	decisec = ((10*overflows)/F);
	MOVF	(_overflows + 3), W, B
	MOVWF	POSTDEC1
	BANKSEL	(_overflows + 2)
	MOVF	(_overflows + 2), W, B
	MOVWF	POSTDEC1
	BANKSEL	(_overflows + 1)
	MOVF	(_overflows + 1), W, B
	MOVWF	POSTDEC1
	BANKSEL	_overflows
	MOVF	_overflows, W, B
	MOVWF	POSTDEC1
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	CALL	__mullong
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
	MOVFF	PRODH, r0x02
	MOVFF	FSR0L, r0x03
	MOVLW	0x08
	ADDWF	FSR1L, F
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	___ulong2fs
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
	MOVFF	PRODH, r0x02
	MOVFF	FSR0L, r0x03
	MOVLW	0x04
	ADDWF	FSR1L, F
	MOVLW	0x42
	MOVWF	POSTDEC1
	MOVLW	0xbe
	MOVWF	POSTDEC1
	MOVLW	0xbb
	MOVWF	POSTDEC1
	MOVLW	0x64
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	___fsdiv
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
	MOVFF	PRODH, r0x02
	MOVFF	FSR0L, r0x03
	MOVLW	0x08
	ADDWF	FSR1L, F
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	___fs2ulong
	BANKSEL	_decisec
	MOVWF	_decisec, B
	MOVFF	PRODL, (_decisec + 1)
	MOVFF	PRODH, (_decisec + 2)
	MOVFF	FSR0L, (_decisec + 3)
	MOVLW	0x04
	ADDWF	FSR1L, F
;	.line	205; reveil.c	ds = decisec%10;
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	BANKSEL	(_decisec + 3)
	MOVF	(_decisec + 3), W, B
	MOVWF	POSTDEC1
	BANKSEL	(_decisec + 2)
	MOVF	(_decisec + 2), W, B
	MOVWF	POSTDEC1
	BANKSEL	(_decisec + 1)
	MOVF	(_decisec + 1), W, B
	MOVWF	POSTDEC1
	BANKSEL	_decisec
	MOVF	_decisec, W, B
	MOVWF	POSTDEC1
	CALL	__modulong
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
	MOVFF	PRODH, r0x02
	MOVFF	FSR0L, r0x03
	MOVLW	0x08
	ADDWF	FSR1L, F
	MOVF	r0x00, W
	BANKSEL	_ds
	MOVWF	_ds, B
;	.line	206; reveil.c	h = (sec/3600)%24;
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x0e
	MOVWF	POSTDEC1
	MOVLW	0x10
	MOVWF	POSTDEC1
	BANKSEL	(_sec + 3)
	MOVF	(_sec + 3), W, B
	MOVWF	POSTDEC1
	BANKSEL	(_sec + 2)
	MOVF	(_sec + 2), W, B
	MOVWF	POSTDEC1
	BANKSEL	(_sec + 1)
	MOVF	(_sec + 1), W, B
	MOVWF	POSTDEC1
	BANKSEL	_sec
	MOVF	_sec, W, B
	MOVWF	POSTDEC1
	CALL	__divulong
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
	MOVFF	PRODH, r0x02
	MOVFF	FSR0L, r0x03
	MOVLW	0x08
	ADDWF	FSR1L, F
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x18
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__modulong
	MOVWF	r0x04
	MOVFF	PRODL, r0x05
	MOVFF	PRODH, r0x06
	MOVFF	FSR0L, r0x07
	MOVLW	0x08
	ADDWF	FSR1L, F
	MOVF	r0x04, W
	BANKSEL	_h
	MOVWF	_h, B
;	.line	207; reveil.c	m = (sec-(sec/3600)*3600)/60;
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x0e
	MOVWF	POSTDEC1
	MOVLW	0x10
	MOVWF	POSTDEC1
	CALL	__mullong
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
	MOVFF	PRODH, r0x02
	MOVFF	FSR0L, r0x03
	MOVLW	0x08
	ADDWF	FSR1L, F
	MOVF	r0x00, W
	BANKSEL	_sec
	SUBWF	_sec, W, B
	MOVWF	r0x00
	MOVF	r0x01, W
	BANKSEL	(_sec + 1)
	SUBWFB	(_sec + 1), W, B
	MOVWF	r0x01
	MOVF	r0x02, W
	BANKSEL	(_sec + 2)
	SUBWFB	(_sec + 2), W, B
	MOVWF	r0x02
	MOVF	r0x03, W
	BANKSEL	(_sec + 3)
	SUBWFB	(_sec + 3), W, B
	MOVWF	r0x03
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x3c
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__divulong
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
	MOVFF	PRODH, r0x02
	MOVFF	FSR0L, r0x03
	MOVLW	0x08
	ADDWF	FSR1L, F
	MOVF	r0x00, W
	BANKSEL	_m
	MOVWF	_m, B
	BANKSEL	_sec
;	.line	208; reveil.c	s = sec-h*3600-m*60;
	MOVF	_sec, W, B
	MOVWF	r0x00
; ;multiply lit val:0x10 by variable _h and store in r0x01
; ;Unrolled 8 X 8 multiplication
; ;FIXME: the function does not support result==WREG
	BANKSEL	_h
	MOVF	_h, W, B
	MULLW	0x10
	MOVFF	PRODL, r0x01
	MOVF	r0x01, W
	SUBWF	r0x00, F
; ;multiply lit val:0x3c by variable _m and store in r0x01
; ;Unrolled 8 X 8 multiplication
; ;FIXME: the function does not support result==WREG
	BANKSEL	_m
	MOVF	_m, W, B
	MULLW	0x3c
	MOVFF	PRODL, r0x01
	MOVF	r0x01, W
	SUBWF	r0x00, W
	BANKSEL	_s
	MOVWF	_s, B
	BANKSEL	_tsec
;	.line	211; reveil.c	if (tsec != s)
	MOVF	_tsec, W, B
	BANKSEL	_s
	XORWF	_s, W, B
	BZ	_00131_DS_
;	.line	212; reveil.c	tsec = s;
	MOVFF	_s, _tsec
_00131_DS_:
	BANKSEL	_tmin
;	.line	213; reveil.c	if (tmin != m)
	MOVF	_tmin, W, B
	BANKSEL	_m
	XORWF	_m, W, B
	BZ	_00133_DS_
;	.line	214; reveil.c	tmin = m;
	MOVFF	_m, _tmin
_00133_DS_:
	BANKSEL	_thour
;	.line	215; reveil.c	if (thour != h)
	MOVF	_thour, W, B
	BANKSEL	_h
	XORWF	_h, W, B
	BZ	_00135_DS_
;	.line	216; reveil.c	thour = h;
	MOVFF	_h, _thour
_00135_DS_:
	BANKSEL	_on
;	.line	219; reveil.c	if (!on && ds < 5) {
	MOVF	_on, W, B
	BNZ	_00139_DS_
	MOVLW	0x05
	BANKSEL	_ds
	SUBWF	_ds, W, B
	BC	_00139_DS_
;	.line	220; reveil.c	LATJbits.LATJ0 = 1;
	BSF	_LATJbits, 0
	BANKSEL	_whereami
;	.line	221; reveil.c	if (whereami == ALARM) {
	MOVF	_whereami, W, B
	XORLW	0x0a
	BNZ	_00137_DS_
;	.line	222; reveil.c	LATJbits.LATJ1 = 1;
	BSF	_LATJbits, 1
;	.line	223; reveil.c	LATJbits.LATJ2 = 1;
	BSF	_LATJbits, 2
_00137_DS_:
;	.line	225; reveil.c	on = 1;
	MOVLW	0x01
	BANKSEL	_on
	MOVWF	_on, B
_00139_DS_:
	BANKSEL	_on
;	.line	229; reveil.c	if (on && ds >= 5) {
	MOVF	_on, W, B
	BZ	_00146_DS_
	MOVLW	0x05
	BANKSEL	_ds
	SUBWF	_ds, W, B
	BNC	_00146_DS_
;	.line	230; reveil.c	LATJbits.LATJ0 = 0;
	BCF	_LATJbits, 0
	BANKSEL	_whereami
;	.line	231; reveil.c	if (whereami == ALARM) {
	MOVF	_whereami, W, B
	XORLW	0x0a
	BNZ	_00142_DS_
;	.line	232; reveil.c	LATJbits.LATJ1 = 0;
	BCF	_LATJbits, 1
;	.line	233; reveil.c	LATJbits.LATJ2 = 0;
	BCF	_LATJbits, 2
_00142_DS_:
	BANKSEL	_on
;	.line	235; reveil.c	on = 0;
	CLRF	_on, B
_00146_DS_:
	MOVFF	PREINC1, r0x07
	MOVFF	PREINC1, r0x06
	MOVFF	PREINC1, r0x05
	MOVFF	PREINC1, r0x04
	MOVFF	PREINC1, r0x03
	MOVFF	PREINC1, r0x02
	MOVFF	PREINC1, r0x01
	MOVFF	PREINC1, r0x00
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_reveil__low_isr	code
_low_isr:
;	.line	136; reveil.c	void low_isr(void) interrupt 2
	MOVFF	WREG, POSTDEC1
	MOVFF	STATUS, POSTDEC1
	MOVFF	BSR, POSTDEC1
	MOVFF	PRODL, POSTDEC1
	MOVFF	PRODH, POSTDEC1
	MOVFF	FSR0L, POSTDEC1
	MOVFF	FSR0H, POSTDEC1
	MOVFF	PCLATH, POSTDEC1
	MOVFF	PCLATU, POSTDEC1
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
;	.line	139; reveil.c	if (INTCON3bits.INT3F) {
	BTFSS	_INTCON3bits, 2
	BRA	_00115_DS_
;	.line	140; reveil.c	button1 = 1;
	MOVLW	0x01
	BANKSEL	_button1
	MOVWF	_button1, B
;	.line	141; reveil.c	INTCON3bits.INT3F = 0;
	BCF	_INTCON3bits, 2
	BRA	_00117_DS_
_00115_DS_:
;	.line	144; reveil.c	} else if (INTCON3bits.INT1F) {
	BTFSS	_INTCON3bits, 0
	BRA	_00117_DS_
;	.line	145; reveil.c	button2 = 1;
	MOVLW	0x01
	BANKSEL	_button2
	MOVWF	_button2, B
;	.line	146; reveil.c	INTCON3bits.INT1F = 0;
	BCF	_INTCON3bits, 0
_00117_DS_:
	MOVFF	PREINC1, FSR2L
	MOVFF	PREINC1, PCLATU
	MOVFF	PREINC1, PCLATH
	MOVFF	PREINC1, FSR0H
	MOVFF	PREINC1, FSR0L
	MOVFF	PREINC1, PRODH
	MOVFF	PREINC1, PRODL
	MOVFF	PREINC1, BSR
	MOVFF	PREINC1, STATUS
	MOVFF	PREINC1, WREG
	RETFIE	

; ; Starting pCode block
S_reveil__high_isr	code
_high_isr:
;	.line	126; reveil.c	void high_isr(void) interrupt 1
	MOVFF	WREG, POSTDEC1
	MOVFF	STATUS, POSTDEC1
	MOVFF	BSR, POSTDEC1
	MOVFF	PRODL, POSTDEC1
	MOVFF	PRODH, POSTDEC1
	MOVFF	FSR0L, POSTDEC1
	MOVFF	FSR0H, POSTDEC1
	MOVFF	PCLATH, POSTDEC1
	MOVFF	PCLATU, POSTDEC1
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
;	.line	129; reveil.c	if (INTCONbits.T0IF) {
	BTFSS	_INTCONbits, 2
	BRA	_00107_DS_
	BANKSEL	_overflows
;	.line	130; reveil.c	overflows++;
	INCF	_overflows, F, B
	BNC	_10348_DS_
	BANKSEL	(_overflows + 1)
	INCF	(_overflows + 1), F, B
_10348_DS_:
	BNC	_20349_DS_
	BANKSEL	(_overflows + 2)
	INCF	(_overflows + 2), F, B
_20349_DS_:
	BNC	_30350_DS_
	BANKSEL	(_overflows + 3)
	INCF	(_overflows + 3), F, B
_30350_DS_:
;	.line	131; reveil.c	INTCONbits.T0IF = 0;
	BCF	_INTCONbits, 2
_00107_DS_:
	MOVFF	PREINC1, FSR2L
	MOVFF	PREINC1, PCLATU
	MOVFF	PREINC1, PCLATH
	MOVFF	PREINC1, FSR0H
	MOVFF	PREINC1, FSR0L
	MOVFF	PREINC1, PRODH
	MOVFF	PREINC1, PRODL
	MOVFF	PREINC1, BSR
	MOVFF	PREINC1, STATUS
	MOVFF	PREINC1, WREG
	RETFIE	

; ; Starting pCode block
__str_0:
	DB	0x44, 0x6f, 0x20, 0x79, 0x6f, 0x75, 0x20, 0x77, 0x61, 0x6e, 0x74, 0x20
	DB	0x74, 0x6f, 0x20, 0x20, 0x73, 0x65, 0x74, 0x20, 0x74, 0x68, 0x65, 0x20
	DB	0x74, 0x69, 0x6d, 0x65, 0x20, 0x3f, 0x20, 0x20, 0x00
; ; Starting pCode block
__str_1:
	DB	0x20, 0x5b, 0x25, 0x30, 0x32, 0x75, 0x5d, 0x3a, 0x20, 0x25, 0x30, 0x32
	DB	0x75, 0x20, 0x3a, 0x20, 0x25, 0x30, 0x32, 0x75, 0x20, 0x20, 0x20, 0x20
	DB	0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20
	DB	0x20, 0x20, 0x00
; ; Starting pCode block
__str_2:
	DB	0x20, 0x20, 0x25, 0x30, 0x32, 0x75, 0x20, 0x3a, 0x5b, 0x25, 0x30, 0x32
	DB	0x75, 0x5d, 0x3a, 0x20, 0x25, 0x30, 0x32, 0x75, 0x20, 0x20, 0x20, 0x20
	DB	0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20
	DB	0x20, 0x20, 0x00
; ; Starting pCode block
__str_3:
	DB	0x20, 0x20, 0x25, 0x30, 0x32, 0x75, 0x20, 0x3a, 0x20, 0x25, 0x30, 0x32
	DB	0x75, 0x20, 0x3a, 0x5b, 0x25, 0x30, 0x32, 0x75, 0x5d, 0x20, 0x20, 0x20
	DB	0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20
	DB	0x20, 0x20, 0x00
; ; Starting pCode block
__str_4:
	DB	0x44, 0x6f, 0x20, 0x79, 0x6f, 0x75, 0x20, 0x77, 0x61, 0x6e, 0x74, 0x20
	DB	0x74, 0x6f, 0x20, 0x20, 0x73, 0x65, 0x74, 0x20, 0x74, 0x68, 0x65, 0x20
	DB	0x61, 0x6c, 0x61, 0x72, 0x6d, 0x20, 0x3f, 0x20, 0x00
; ; Starting pCode block
__str_5:
	DB	0x20, 0x20, 0x41, 0x6c, 0x61, 0x72, 0x6d, 0x20, 0x5b, 0x4f, 0x4e, 0x20
	DB	0x5d, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20
	DB	0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00
; ; Starting pCode block
__str_6:
	DB	0x20, 0x20, 0x41, 0x6c, 0x61, 0x72, 0x6d, 0x20, 0x5b, 0x4f, 0x46, 0x46
	DB	0x5d, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20
	DB	0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00
; ; Starting pCode block
__str_7:
	DB	0x20, 0x20, 0x20, 0x20, 0x41, 0x6c, 0x61, 0x72, 0x6d, 0x20, 0x61, 0x74
	DB	0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x5b, 0x25, 0x30, 0x32
	DB	0x75, 0x5d, 0x3a, 0x20, 0x25, 0x30, 0x32, 0x75, 0x20, 0x20, 0x20, 0x20
	DB	0x00
; ; Starting pCode block
__str_8:
	DB	0x20, 0x20, 0x20, 0x20, 0x41, 0x6c, 0x61, 0x72, 0x6d, 0x20, 0x61, 0x74
	DB	0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x25, 0x30, 0x32
	DB	0x75, 0x20, 0x3a, 0x5b, 0x25, 0x30, 0x32, 0x75, 0x5d, 0x20, 0x20, 0x20
	DB	0x00
; ; Starting pCode block
__str_9:
	DB	0x20, 0x20, 0x20, 0x20, 0x25, 0x30, 0x32, 0x75, 0x3a, 0x25, 0x30, 0x32
	DB	0x75, 0x3a, 0x25, 0x30, 0x32, 0x75, 0x20, 0x20, 0x20, 0x20, 0x41, 0x6c
	DB	0x61, 0x72, 0x6d, 0x20, 0x4f, 0x4e, 0x20, 0x20, 0x25, 0x30, 0x32, 0x75
	DB	0x3a, 0x25, 0x30, 0x32, 0x75, 0x20, 0x00
; ; Starting pCode block
__str_10:
	DB	0x20, 0x20, 0x20, 0x20, 0x25, 0x30, 0x32, 0x75, 0x3a, 0x25, 0x30, 0x32
	DB	0x75, 0x3a, 0x25, 0x30, 0x32, 0x75, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20
	DB	0x20, 0x41, 0x6c, 0x61, 0x72, 0x6d, 0x20, 0x20, 0x4f, 0x46, 0x46, 0x20
	DB	0x20, 0x20, 0x00
; ; Starting pCode block
__str_11:
	DB	0x20, 0x20, 0x20, 0x20, 0x25, 0x30, 0x32, 0x75, 0x3a, 0x25, 0x30, 0x32
	DB	0x75, 0x3a, 0x25, 0x30, 0x32, 0x75, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20
	DB	0x49, 0x20, 0x61, 0x6d, 0x20, 0x72, 0x69, 0x6e, 0x67, 0x69, 0x6e, 0x67
	DB	0x21, 0x20, 0x00
; ; Starting pCode block
__str_12:
	DB	0x20, 0x20, 0x20, 0x20, 0x25, 0x30, 0x32, 0x75, 0x3a, 0x25, 0x30, 0x32
	DB	0x75, 0x3a, 0x25, 0x30, 0x32, 0x75, 0x20, 0x20, 0x20, 0x20, 0x53, 0x6e
	DB	0x6f, 0x6f, 0x7a, 0x65, 0x20, 0x25, 0x75, 0x20, 0x20, 0x25, 0x30, 0x32
	DB	0x75, 0x3a, 0x25, 0x30, 0x32, 0x75, 0x20, 0x00
; ; Starting pCode block
__str_13:
	DB	0x20, 0x20, 0x20, 0x20, 0x25, 0x30, 0x32, 0x75, 0x3a, 0x25, 0x30, 0x32
	DB	0x75, 0x3a, 0x25, 0x30, 0x32, 0x75, 0x20, 0x20, 0x20, 0x20, 0x53, 0x6e
	DB	0x6f, 0x6f, 0x7a, 0x65, 0x20, 0x25, 0x75, 0x20, 0x25, 0x30, 0x32, 0x75
	DB	0x3a, 0x25, 0x30, 0x32, 0x75, 0x20, 0x00
; ; Starting pCode block
__str_14:
	DB	0x2a, 0x2a, 0x2a, 0x2a, 0x20, 0x45, 0x52, 0x52, 0x4f, 0x52, 0x20, 0x2a
	DB	0x2a, 0x2a, 0x2a, 0x2a, 0x2a, 0x2a, 0x2a, 0x2a, 0x20, 0x45, 0x52, 0x52
	DB	0x4f, 0x52, 0x20, 0x2a, 0x2a, 0x2a, 0x2a, 0x2a, 0x00


; Statistics:
; code size:	 4698 (0x125a) bytes ( 3.58%)
;           	 2349 (0x092d) words
; udata size:	   44 (0x002c) bytes ( 1.15%)
; access size:	   17 (0x0011) bytes


	end

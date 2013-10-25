;--------------------------------------------------------
; File Created by SDCC : free open source ANSI-C Compiler
; Version 2.9.4 #5595 (Oct 17 2013) (Mac OS X ppc)
; This file was generated Fri Oct 25 16:39:29 2013
;--------------------------------------------------------
; PIC16 port for the Microchip 16-bit core micros
;--------------------------------------------------------
	list	p=18f97j60

	radix dec

;--------------------------------------------------------
; public variables in this module
;--------------------------------------------------------
	global _inc_tsec
	global _inc_ahour
	global _inc_amin
	global _dec_ahour
	global _dec_amin
	global _time
	global _manageseconds
	global _button
	global _refresh_lcd
	global _DisplayString
	global _strlcpy
	global _chandelle
	global _thour
	global _tmin
	global _tsec
	global _alarm_set
	global _ahour
	global _amin
	global _ahour_o
	global _amin_o
	global _snooze
	global _stop_ringing
	global _whereami
	global _new_time
	global _halfsecond
	global _button1
	global _button2
	global _overflows
	global _pseudoseconds
	global _high_isr
	global _low_isr
	global _main

;--------------------------------------------------------
; extern variables in this module
;--------------------------------------------------------
	extern __gptrget1
	extern __gptrput1
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
	extern _strlen
	extern _LCDInit
	extern _LCDUpdate
	extern __divsint
	extern __modsint
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
_chandelle	db	0x01
_thour	db	0x00
_tmin	db	0x00
_tsec	db	0x00
_alarm_set	db	0x00
_ahour	db	0x00
_amin	db	0x00
_ahour_o	db	0x00
_amin_o	db	0x00
_snooze	db	0x00
_stop_ringing	db	0x00
_whereami	db	0x00
_new_time	db	0x00
_halfsecond	db	0x00
_button1	db	0x00
_button2	db	0x00
_overflows	db	0x00, 0x00
_pseudoseconds	db	0x00, 0x00


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
r0x11	res	1
r0x12	res	1
r0x13	res	1
r0x14	res	1
r0x15	res	1

udata_lena_0	udata
_refresh_lcd_display_1_1	res	32

;--------------------------------------------------------
; interrupt vector 
;--------------------------------------------------------

;--------------------------------------------------------
; global & static initialisations
;--------------------------------------------------------
; ; Starting pCode block for absolute section
; ;-----------------------------------------
S_lena_ivec_0x1_high_isr	code	0X000008
ivec_0x1_high_isr:
	GOTO	_high_isr

; ; Starting pCode block for absolute section
; ;-----------------------------------------
S_lena_ivec_0x2_low_isr	code	0X000018
ivec_0x2_low_isr:
	GOTO	_low_isr

; I code from now on!
; ; Starting pCode block
S_lena__main	code
_main:
;	.line	142; lena.c	RCONbits.IPEN       = 1; //enable interrupts priority levels
	BSF	_RCONbits, 7
;	.line	143; lena.c	INTCONbits.GIE      = 1; //enables all high-priority interrupts
	BSF	_INTCONbits, 7
;	.line	144; lena.c	INTCONbits.PEIE     = 1; //enables all low-priority peripheral interrupts
	BSF	_INTCONbits, 6
;	.line	147; lena.c	T0CONbits.TMR0ON    = 1; //enables Timer0
	BSF	_T0CONbits, 7
;	.line	148; lena.c	INTCONbits.TMR0IE   = 1; //enables the TMR0 overflow interrupt
	BSF	_INTCONbits, 5
;	.line	149; lena.c	INTCONbits.TMR0IF   = 0; //clear Timer0 overflow bit
	BCF	_INTCONbits, 2
;	.line	150; lena.c	INTCON2bits.TMR0IP  = 1; //high priority
	BSF	_INTCON2bits, 2
;	.line	151; lena.c	T0CONbits.T08BIT    = 0; //timer0 is configured as a 16-bit timer/counter
	BCF	_T0CONbits, 6
;	.line	152; lena.c	T0CONbits.T0CS      = 0; //internal instruction cycle clock (CLKO)
	BCF	_T0CONbits, 5
;	.line	153; lena.c	T0CONbits.PSA       = 1; //timer0 prescaler is not assigned
	BSF	_T0CONbits, 3
;	.line	154; lena.c	TMR0L = 0;    TMR0H = 0;
	CLRF	_TMR0L
	CLRF	_TMR0H
;	.line	157; lena.c	LED0_TRIS = 0; //configure 1st led pin as output (yellow)
	BCF	_TRISJbits, 0
;	.line	158; lena.c	LED1_TRIS = 0; //configure 2nd led pin as output (red)
	BCF	_TRISJbits, 1
;	.line	159; lena.c	LED2_TRIS = 0; //configure 3rd led pin as output (red)
	BCF	_TRISJbits, 2
;	.line	160; lena.c	LATJbits.LATJ0 = 0; // switch LED 1 off
	BCF	_LATJbits, 0
;	.line	161; lena.c	LATJbits.LATJ1 = 0; // switch LED 2 off
	BCF	_LATJbits, 1
;	.line	162; lena.c	LATJbits.LATJ2 = 0; // switch LED 3 off
	BCF	_LATJbits, 2
;	.line	165; lena.c	BUTTON0_TRIS        = 1; //configure 1st button as input
	BSF	_TRISBbits, 3
;	.line	166; lena.c	BUTTON1_TRIS        = 1; //configure 2nd button as input
	BSF	_TRISBbits, 1
;	.line	167; lena.c	INTCON3bits.INT3E   = 1; //enable INT3 interrupt (button 1)
	BSF	_INTCON3bits, 5
;	.line	168; lena.c	INTCON3bits.INT3F   = 0; //clear INT3 flag
	BCF	_INTCON3bits, 2
;	.line	169; lena.c	INTCON3bits.INT1E   = 1; //enable INT1 interrupt (button 2)
	BSF	_INTCON3bits, 3
;	.line	170; lena.c	INTCON3bits.INT1F   = 0; //clear INT1 flag
	BCF	_INTCON3bits, 0
;	.line	171; lena.c	INTCON3bits.INT1IP  = 0; //low priority
	BCF	_INTCON3bits, 6
;	.line	172; lena.c	INTCON2bits.INT3IP  = 0; //low priority
	BCF	_INTCON2bits, 1
;	.line	178; lena.c	LCDInit();
	CALL	_LCDInit
;	.line	179; lena.c	whereami = TIME_MENU;
	MOVLW	0x01
	BANKSEL	_whereami
	MOVWF	_whereami, B
	BANKSEL	_chandelle
;	.line	181; lena.c	chandelle++; // ##### BIZARRE ####
	INCF	_chandelle, F, B
;	.line	183; lena.c	T0CONbits.TMR0ON = 1; // start timer0
	BSF	_T0CONbits, 7
_00135_DS_:
;	.line	185; lena.c	time();
	CALL	_time
;	.line	186; lena.c	manageseconds();
	CALL	_manageseconds
;	.line	187; lena.c	button();
	CALL	_button
;	.line	188; lena.c	refresh_lcd();
	CALL	_refresh_lcd
	BRA	_00135_DS_
	RETURN	

; ; Starting pCode block
S_lena__strlcpy	code
_strlcpy:
;	.line	562; lena.c	strlcpy(char *dst, const char *src, size_t siz)
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
	MOVFF	r0x0f, POSTDEC1
	MOVFF	r0x10, POSTDEC1
	MOVFF	r0x11, POSTDEC1
	MOVFF	r0x12, POSTDEC1
	MOVFF	r0x13, POSTDEC1
	MOVFF	r0x14, POSTDEC1
	MOVFF	r0x15, POSTDEC1
	MOVLW	0x02
	MOVFF	PLUSW2, r0x00
	MOVLW	0x03
	MOVFF	PLUSW2, r0x01
	MOVLW	0x04
	MOVFF	PLUSW2, r0x02
	MOVLW	0x05
	MOVFF	PLUSW2, r0x03
	MOVLW	0x06
	MOVFF	PLUSW2, r0x04
	MOVLW	0x07
	MOVFF	PLUSW2, r0x05
	MOVLW	0x08
	MOVFF	PLUSW2, r0x06
	MOVLW	0x09
	MOVFF	PLUSW2, r0x07
;	.line	564; lena.c	char       *d = dst;
	MOVFF	r0x00, r0x08
	MOVFF	r0x01, r0x09
	MOVFF	r0x02, r0x0a
;	.line	565; lena.c	const char *s = src;
	MOVFF	r0x03, r0x0b
	MOVFF	r0x04, r0x0c
	MOVFF	r0x05, r0x0d
;	.line	566; lena.c	size_t      n = siz;
	MOVFF	r0x06, r0x0e
	MOVFF	r0x07, r0x0f
;	.line	569; lena.c	if (n != 0)
	MOVF	r0x06, W
	IORWF	r0x07, W
	BTFSC	STATUS, 2
	BRA	_00395_DS_
;	.line	571; lena.c	while (--n != 0)
	MOVFF	r0x03, r0x10
	MOVFF	r0x04, r0x11
	MOVFF	r0x05, r0x12
	MOVFF	r0x06, r0x13
	MOVFF	r0x07, r0x14
_00391_DS_:
	MOVLW	0xff
	ADDWF	r0x13, F
	BTFSS	STATUS, 0
	DECF	r0x14, F
	MOVF	r0x13, W
	IORWF	r0x14, W
	BZ	_00410_DS_
;	.line	573; lena.c	if ((*d++ = *s++) == '\0')
	MOVFF	r0x10, FSR0L
	MOVFF	r0x11, PRODL
	MOVF	r0x12, W
	CALL	__gptrget1
	MOVWF	r0x15
	INCF	r0x10, F
	BTFSC	STATUS, 0
	INCF	r0x11, F
	BTFSC	STATUS, 0
	INCF	r0x12, F
	MOVFF	r0x15, POSTDEC1
	MOVFF	r0x00, FSR0L
	MOVFF	r0x01, PRODL
	MOVF	r0x02, W
	CALL	__gptrput1
	INCF	r0x00, F
	BTFSC	STATUS, 0
	INCF	r0x01, F
	BTFSC	STATUS, 0
	INCF	r0x02, F
	MOVF	r0x15, W
	BNZ	_00391_DS_
_00410_DS_:
;	.line	574; lena.c	break;
	MOVFF	r0x10, r0x0b
	MOVFF	r0x11, r0x0c
	MOVFF	r0x12, r0x0d
	MOVFF	r0x00, r0x08
	MOVFF	r0x01, r0x09
	MOVFF	r0x02, r0x0a
	MOVFF	r0x13, r0x0e
	MOVFF	r0x14, r0x0f
_00395_DS_:
;	.line	579; lena.c	if (n == 0)
	MOVF	r0x0e, W
	IORWF	r0x0f, W
	BNZ	_00402_DS_
;	.line	581; lena.c	if (siz != 0)
	MOVF	r0x06, W
	IORWF	r0x07, W
	BZ	_00409_DS_
;	.line	582; lena.c	*d = '\0';          /* NUL-terminate dst */
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVFF	r0x08, FSR0L
	MOVFF	r0x09, PRODL
	MOVF	r0x0a, W
	CALL	__gptrput1
_00409_DS_:
;	.line	583; lena.c	while (*s++)
	MOVFF	r0x0b, r0x00
	MOVFF	r0x0c, r0x01
	MOVFF	r0x0d, r0x02
_00398_DS_:
	MOVFF	r0x00, FSR0L
	MOVFF	r0x01, PRODL
	MOVF	r0x02, W
	CALL	__gptrget1
	MOVWF	r0x06
	INCF	r0x00, F
	BTFSC	STATUS, 0
	INCF	r0x01, F
	BTFSC	STATUS, 0
	INCF	r0x02, F
	MOVF	r0x06, W
	BNZ	_00398_DS_
	MOVFF	r0x00, r0x0b
	MOVFF	r0x01, r0x0c
	MOVFF	r0x02, r0x0d
_00402_DS_:
;	.line	589; lena.c	return (s - src - 1);       /* count does not include NUL */
	MOVF	r0x03, W
	SUBWF	r0x0b, W
	MOVWF	r0x03
	MOVF	r0x04, W
	SUBWFB	r0x0c, W
	MOVWF	r0x04
	MOVLW	0xff
	ADDWF	r0x03, F
	BTFSS	STATUS, 0
	DECF	r0x04, F
	MOVFF	r0x04, PRODL
	MOVF	r0x03, W
	MOVFF	PREINC1, r0x15
	MOVFF	PREINC1, r0x14
	MOVFF	PREINC1, r0x13
	MOVFF	PREINC1, r0x12
	MOVFF	PREINC1, r0x11
	MOVFF	PREINC1, r0x10
	MOVFF	PREINC1, r0x0f
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
S_lena__DisplayString	code
_DisplayString:
;	.line	516; lena.c	void DisplayString(BYTE pos, char* text)
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
	MOVLW	0x02
	MOVFF	PLUSW2, r0x00
	MOVLW	0x03
	MOVFF	PLUSW2, r0x01
	MOVLW	0x04
	MOVFF	PLUSW2, r0x02
	MOVLW	0x05
	MOVFF	PLUSW2, r0x03
;	.line	518; lena.c	BYTE l= strlen(text)+1;
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	CALL	_strlen
	MOVWF	r0x04
	MOVFF	PRODL, r0x05
	MOVLW	0x03
	ADDWF	FSR1L, F
	INCF	r0x04, F
;	.line	519; lena.c	BYTE max= 32-pos;
	MOVF	r0x00, W
	SUBLW	0x20
	MOVWF	r0x05
;	.line	520; lena.c	strlcpy((char*)&LCDText[pos], text,(l<max)?l:max );
	CLRF	r0x06
	MOVLW	LOW(_LCDText)
	ADDWF	r0x00, F
	MOVLW	HIGH(_LCDText)
	ADDWFC	r0x06, F
	MOVF	r0x06, W
	MOVWF	r0x06
	MOVF	r0x00, W
	MOVWF	r0x00
	MOVLW	0x80
	MOVWF	r0x07
	MOVF	r0x05, W
	SUBWF	r0x04, W
	BNC	_00382_DS_
	MOVFF	r0x05, r0x04
_00382_DS_:
	CLRF	r0x05
	MOVF	r0x05, W
	MOVWF	POSTDEC1
	MOVF	r0x04, W
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x07, W
	MOVWF	POSTDEC1
	MOVF	r0x06, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	_strlcpy
	MOVLW	0x08
	ADDWF	FSR1L, F
;	.line	521; lena.c	LCDUpdate();
	CALL	_LCDUpdate
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
S_lena__refresh_lcd	code
_refresh_lcd:
;	.line	442; lena.c	void refresh_lcd(void)
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
;	.line	446; lena.c	switch (whereami) {
	MOVLW	0x01
	BANKSEL	_whereami
	SUBWF	_whereami, W, B
	BTFSS	STATUS, 0
	GOTO	_00362_DS_
	MOVLW	0x0c
	BANKSEL	_whereami
	SUBWF	_whereami, W, B
	BTFSC	STATUS, 0
	GOTO	_00362_DS_
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
	MOVLW	UPPER(_00373_DS_)
	MOVWF	PCLATU
	MOVLW	HIGH(_00373_DS_)
	MOVWF	PCLATH
	MOVLW	LOW(_00373_DS_)
	ADDWF	r0x0f, F
	MOVF	r0x10, W
	ADDWFC	PCLATH, F
	BTFSC	STATUS, 0
	INCF	PCLATU, F
	MOVF	r0x0f, W
	MOVFF	PREINC1, r0x10
	MOVFF	PREINC1, r0x0f
	MOVWF	PCL
_00373_DS_:
	GOTO	_00342_DS_
	GOTO	_00343_DS_
	GOTO	_00344_DS_
	GOTO	_00345_DS_
	GOTO	_00346_DS_
	GOTO	_00347_DS_
	GOTO	_00351_DS_
	GOTO	_00352_DS_
	GOTO	_00353_DS_
	GOTO	_00357_DS_
	GOTO	_00358_DS_
_00342_DS_:
;	.line	448; lena.c	sprintf(display, "Do you want to  set the time ?  ");
	MOVLW	HIGH(_refresh_lcd_display_1_1)
	MOVWF	r0x01
	MOVLW	LOW(_refresh_lcd_display_1_1)
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
;	.line	449; lena.c	break;
	GOTO	_00363_DS_
_00343_DS_:
;	.line	452; lena.c	thour, tmin, tsec);
	MOVFF	_tsec, r0x00
	CLRF	r0x01
	MOVFF	_tmin, r0x02
	CLRF	r0x03
	MOVFF	_thour, r0x04
	CLRF	r0x05
;	.line	451; lena.c	sprintf(display, " [%02u]: %02u : %02u                  ",
	MOVLW	HIGH(_refresh_lcd_display_1_1)
	MOVWF	r0x07
	MOVLW	LOW(_refresh_lcd_display_1_1)
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
;	.line	453; lena.c	break;
	GOTO	_00363_DS_
_00344_DS_:
;	.line	456; lena.c	thour, tmin, tsec);
	MOVFF	_tsec, r0x00
	CLRF	r0x01
	MOVFF	_tmin, r0x02
	CLRF	r0x03
	MOVFF	_thour, r0x04
	CLRF	r0x05
;	.line	455; lena.c	sprintf(display, "  %02u :[%02u]: %02u                  ",
	MOVLW	HIGH(_refresh_lcd_display_1_1)
	MOVWF	r0x07
	MOVLW	LOW(_refresh_lcd_display_1_1)
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
;	.line	457; lena.c	break;
	BRA	_00363_DS_
_00345_DS_:
;	.line	460; lena.c	thour, tmin, tsec);
	MOVFF	_tsec, r0x00
	CLRF	r0x01
	MOVFF	_tmin, r0x02
	CLRF	r0x03
	MOVFF	_thour, r0x04
	CLRF	r0x05
;	.line	459; lena.c	sprintf(display, "  %02u : %02u :[%02u]                 ",
	MOVLW	HIGH(_refresh_lcd_display_1_1)
	MOVWF	r0x07
	MOVLW	LOW(_refresh_lcd_display_1_1)
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
;	.line	461; lena.c	break;
	BRA	_00363_DS_
_00346_DS_:
;	.line	463; lena.c	sprintf(display, "Do you want to  set the alarm ? ");
	MOVLW	HIGH(_refresh_lcd_display_1_1)
	MOVWF	r0x01
	MOVLW	LOW(_refresh_lcd_display_1_1)
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
;	.line	464; lena.c	break;
	BRA	_00363_DS_
_00347_DS_:
	BANKSEL	_alarm_set
;	.line	466; lena.c	if (alarm_set) {
	MOVF	_alarm_set, W, B
	BZ	_00349_DS_
;	.line	467; lena.c	sprintf(display, "  Alarm [ON ]                   ");
	MOVLW	HIGH(_refresh_lcd_display_1_1)
	MOVWF	r0x01
	MOVLW	LOW(_refresh_lcd_display_1_1)
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
	BRA	_00363_DS_
_00349_DS_:
;	.line	469; lena.c	sprintf(display, "  Alarm [OFF]                   ");
	MOVLW	HIGH(_refresh_lcd_display_1_1)
	MOVWF	r0x01
	MOVLW	LOW(_refresh_lcd_display_1_1)
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
;	.line	471; lena.c	break;
	BRA	_00363_DS_
_00351_DS_:
;	.line	474; lena.c	ahour, amin);
	MOVFF	_amin, r0x00
	CLRF	r0x01
	MOVFF	_ahour, r0x02
	CLRF	r0x03
;	.line	473; lena.c	sprintf(display, "    Alarm at        [%02u]: %02u    ",
	MOVLW	HIGH(_refresh_lcd_display_1_1)
	MOVWF	r0x05
	MOVLW	LOW(_refresh_lcd_display_1_1)
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
;	.line	475; lena.c	break;
	BRA	_00363_DS_
_00352_DS_:
;	.line	478; lena.c	ahour, amin);
	MOVFF	_amin, r0x00
	CLRF	r0x01
	MOVFF	_ahour, r0x02
	CLRF	r0x03
;	.line	477; lena.c	sprintf(display, "    Alarm at         %02u :[%02u]   ",
	MOVLW	HIGH(_refresh_lcd_display_1_1)
	MOVWF	r0x05
	MOVLW	LOW(_refresh_lcd_display_1_1)
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
;	.line	479; lena.c	break;
	BRA	_00363_DS_
_00353_DS_:
	BANKSEL	_alarm_set
;	.line	481; lena.c	if (alarm_set) {
	MOVF	_alarm_set, W, B
	BZ	_00355_DS_
;	.line	483; lena.c	thour, tmin, tsec, ahour, amin);
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
;	.line	482; lena.c	sprintf(display, "    %02u:%02u:%02u    Alarm ON  %02u:%02u ",
	MOVLW	HIGH(_refresh_lcd_display_1_1)
	MOVWF	r0x0b
	MOVLW	LOW(_refresh_lcd_display_1_1)
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
	BRA	_00363_DS_
_00355_DS_:
;	.line	486; lena.c	thour, tmin, tsec);
	MOVFF	_tsec, r0x00
	CLRF	r0x01
	MOVFF	_tmin, r0x02
	CLRF	r0x03
	MOVFF	_thour, r0x04
	CLRF	r0x05
;	.line	485; lena.c	sprintf(display, "    %02u:%02u:%02u       Alarm  OFF   ",
	MOVLW	HIGH(_refresh_lcd_display_1_1)
	MOVWF	r0x07
	MOVLW	LOW(_refresh_lcd_display_1_1)
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
;	.line	488; lena.c	break;
	BRA	_00363_DS_
_00357_DS_:
;	.line	491; lena.c	thour, tmin, tsec); // ***blink***
	MOVFF	_tsec, r0x00
	CLRF	r0x01
	MOVFF	_tmin, r0x02
	CLRF	r0x03
	MOVFF	_thour, r0x04
	CLRF	r0x05
;	.line	490; lena.c	sprintf(display, "    %02u:%02u:%02u      I am ringing! ",
	MOVLW	HIGH(_refresh_lcd_display_1_1)
	MOVWF	r0x07
	MOVLW	LOW(_refresh_lcd_display_1_1)
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
;	.line	492; lena.c	break;
	BRA	_00363_DS_
_00358_DS_:
;	.line	494; lena.c	if (snooze < 10) {
	MOVLW	0x0a
	BANKSEL	_snooze
	SUBWF	_snooze, W, B
	BTFSC	STATUS, 0
	BRA	_00360_DS_
;	.line	496; lena.c	thour, tmin, tsec, snooze, ahour_o, amin_o);
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
;	.line	495; lena.c	sprintf(display, "    %02u:%02u:%02u    Snooze %u  %02u:%02u ",
	MOVLW	HIGH(_refresh_lcd_display_1_1)
	MOVWF	r0x0d
	MOVLW	LOW(_refresh_lcd_display_1_1)
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
	BRA	_00363_DS_
_00360_DS_:
;	.line	499; lena.c	thour, tmin, tsec, snooze, ahour_o, amin_o);
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
;	.line	498; lena.c	sprintf(display, "    %02u:%02u:%02u    Snooze %u %02u:%02u ",
	MOVLW	HIGH(_refresh_lcd_display_1_1)
	MOVWF	r0x0d
	MOVLW	LOW(_refresh_lcd_display_1_1)
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
;	.line	502; lena.c	break;
	BRA	_00363_DS_
_00362_DS_:
;	.line	504; lena.c	sprintf(display, "**** ERROR ********* ERROR *****");
	MOVLW	HIGH(_refresh_lcd_display_1_1)
	MOVWF	r0x01
	MOVLW	LOW(_refresh_lcd_display_1_1)
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
_00363_DS_:
;	.line	507; lena.c	DisplayString(0, display);
	MOVLW	HIGH(_refresh_lcd_display_1_1)
	MOVWF	r0x01
	MOVLW	LOW(_refresh_lcd_display_1_1)
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
S_lena__button	code
_button:
;	.line	313; lena.c	void button(void)
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	MOVFF	r0x00, POSTDEC1
	BANKSEL	_button1
;	.line	316; lena.c	if (button1) {
	MOVF	_button1, W, B
	BTFSC	STATUS, 2
	BRA	_00302_DS_
;	.line	317; lena.c	switch (whereami) {
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
	MOVFF	r0x01, POSTDEC1
	MOVFF	r0x02, POSTDEC1
	CLRF	r0x02
	RLCF	r0x00, W
	RLCF	r0x02, F
	RLCF	WREG, W
	RLCF	r0x02, F
	ANDLW	0xfc
	MOVWF	r0x01
	MOVLW	UPPER(_00321_DS_)
	MOVWF	PCLATU
	MOVLW	HIGH(_00321_DS_)
	MOVWF	PCLATH
	MOVLW	LOW(_00321_DS_)
	ADDWF	r0x01, F
	MOVF	r0x02, W
	ADDWFC	PCLATH, F
	BTFSC	STATUS, 0
	INCF	PCLATU, F
	MOVF	r0x01, W
	MOVFF	PREINC1, r0x02
	MOVFF	PREINC1, r0x01
	MOVWF	PCL
_00321_DS_:
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
;	.line	319; lena.c	whereami = ALARM_MENU;
	MOVLW	0x05
	BANKSEL	_whereami
	MOVWF	_whereami, B
;	.line	320; lena.c	break;
	BRA	_00267_DS_
_00256_DS_:
;	.line	322; lena.c	whereami = SET_MINUTE;
	MOVLW	0x03
	BANKSEL	_whereami
	MOVWF	_whereami, B
;	.line	323; lena.c	break;
	BRA	_00267_DS_
_00257_DS_:
;	.line	325; lena.c	whereami = SET_SECOND;
	MOVLW	0x04
	BANKSEL	_whereami
	MOVWF	_whereami, B
;	.line	326; lena.c	break;
	BRA	_00267_DS_
_00258_DS_:
;	.line	328; lena.c	whereami = ALARM_MENU;
	MOVLW	0x05
	BANKSEL	_whereami
	MOVWF	_whereami, B
;	.line	329; lena.c	break;
	BRA	_00267_DS_
_00259_DS_:
;	.line	331; lena.c	whereami = DISPLAY;
	MOVLW	0x09
	BANKSEL	_whereami
	MOVWF	_whereami, B
;	.line	332; lena.c	break;
	BRA	_00267_DS_
_00260_DS_:
;	.line	334; lena.c	whereami = SET_A_HOUR;
	MOVLW	0x07
	BANKSEL	_whereami
	MOVWF	_whereami, B
;	.line	335; lena.c	break;
	BRA	_00267_DS_
_00261_DS_:
;	.line	337; lena.c	whereami = SET_A_MIN;
	MOVLW	0x08
	BANKSEL	_whereami
	MOVWF	_whereami, B
;	.line	338; lena.c	break;
	BRA	_00267_DS_
_00262_DS_:
;	.line	340; lena.c	whereami = DISPLAY;
	MOVLW	0x09
	BANKSEL	_whereami
	MOVWF	_whereami, B
;	.line	341; lena.c	break;
	BRA	_00267_DS_
_00263_DS_:
;	.line	343; lena.c	whereami = TIME_MENU;
	MOVLW	0x01
	BANKSEL	_whereami
	MOVWF	_whereami, B
;	.line	344; lena.c	break;
	BRA	_00267_DS_
_00264_DS_:
;	.line	346; lena.c	stop_ringing = 1;
	MOVLW	0x01
	BANKSEL	_stop_ringing
	MOVWF	_stop_ringing, B
;	.line	347; lena.c	LATJbits.LATJ1 = 0; // switch LED 2 off
	BCF	_LATJbits, 1
;	.line	348; lena.c	LATJbits.LATJ2 = 0; // switch LED 3 off
	BCF	_LATJbits, 2
;	.line	349; lena.c	whereami = DISPLAY;
	MOVLW	0x09
	BANKSEL	_whereami
	MOVWF	_whereami, B
;	.line	350; lena.c	break;
	BRA	_00267_DS_
_00265_DS_:
;	.line	352; lena.c	amin = amin_o; // remet le réveil
	MOVFF	_amin_o, _amin
;	.line	353; lena.c	ahour = ahour_o;
	MOVFF	_ahour_o, _ahour
	BANKSEL	_snooze
;	.line	354; lena.c	snooze = 0;
	CLRF	_snooze, B
	BANKSEL	_stop_ringing
;	.line	355; lena.c	stop_ringing = 0; // le réveil doit sonner à nouveau 
	CLRF	_stop_ringing, B
;	.line	356; lena.c	LATJbits.LATJ1 = 0; // switch LED 2 off
	BCF	_LATJbits, 1
;	.line	357; lena.c	LATJbits.LATJ2 = 0; // switch LED 3 off
	BCF	_LATJbits, 2
;	.line	358; lena.c	whereami = DISPLAY;
	MOVLW	0x09
	BANKSEL	_whereami
	MOVWF	_whereami, B
_00267_DS_:
	BANKSEL	_button1
;	.line	363; lena.c	button1 = 0; // remet le flag du boutton 1 à 0
	CLRF	_button1, B
	BRA	_00304_DS_
_00302_DS_:
	BANKSEL	_button2
;	.line	366; lena.c	} else if (button2) {
	MOVF	_button2, W, B
	BTFSC	STATUS, 2
	BRA	_00304_DS_
;	.line	367; lena.c	switch (whereami) {
	MOVLW	0x01
	BANKSEL	_whereami
	SUBWF	_whereami, W, B
	BTFSS	STATUS, 0
	BRA	_00298_DS_
	MOVLW	0x0c
	BANKSEL	_whereami
	SUBWF	_whereami, W, B
	BTFSC	STATUS, 0
	BRA	_00298_DS_
	BANKSEL	_whereami
	DECF	_whereami, W, B
	MOVWF	r0x00
	MOVFF	r0x01, POSTDEC1
	MOVFF	r0x02, POSTDEC1
	CLRF	r0x02
	RLCF	r0x00, W
	RLCF	r0x02, F
	RLCF	WREG, W
	RLCF	r0x02, F
	ANDLW	0xfc
	MOVWF	r0x01
	MOVLW	UPPER(_00324_DS_)
	MOVWF	PCLATU
	MOVLW	HIGH(_00324_DS_)
	MOVWF	PCLATH
	MOVLW	LOW(_00324_DS_)
	ADDWF	r0x01, F
	MOVF	r0x02, W
	ADDWFC	PCLATH, F
	BTFSC	STATUS, 0
	INCF	PCLATU, F
	MOVF	r0x01, W
	MOVFF	PREINC1, r0x02
	MOVFF	PREINC1, r0x01
	MOVWF	PCL
_00324_DS_:
	GOTO	_00268_DS_
	GOTO	_00269_DS_
	GOTO	_00272_DS_
	GOTO	_00276_DS_
	GOTO	_00280_DS_
	GOTO	_00281_DS_
	GOTO	_00282_DS_
	GOTO	_00286_DS_
	GOTO	_00290_DS_
	GOTO	_00291_DS_
	GOTO	_00294_DS_
_00268_DS_:
;	.line	369; lena.c	whereami = SET_HOUR;
	MOVLW	0x02
	BANKSEL	_whereami
	MOVWF	_whereami, B
;	.line	370; lena.c	break;
	BRA	_00298_DS_
_00269_DS_:
	BANKSEL	_thour
;	.line	372; lena.c	thour++;
	INCF	_thour, F, B
	BANKSEL	_thour
;	.line	373; lena.c	if (thour == 24) {
	MOVF	_thour, W, B
	XORLW	0x18
	BZ	_00326_DS_
	BRA	_00298_DS_
_00326_DS_:
	BANKSEL	_thour
;	.line	374; lena.c	thour = 0;
	CLRF	_thour, B
;	.line	376; lena.c	break;
	BRA	_00298_DS_
_00272_DS_:
	BANKSEL	_tmin
;	.line	378; lena.c	if (tmin == 59) {
	MOVF	_tmin, W, B
	XORLW	0x3b
	BNZ	_00274_DS_
_00328_DS_:
	BANKSEL	_tmin
;	.line	379; lena.c	tmin = 0;
	CLRF	_tmin, B
	BRA	_00298_DS_
_00274_DS_:
	BANKSEL	_tmin
;	.line	381; lena.c	tmin++;
	INCF	_tmin, F, B
;	.line	383; lena.c	break;
	BRA	_00298_DS_
_00276_DS_:
	BANKSEL	_tsec
;	.line	385; lena.c	if (tsec == 59) {
	MOVF	_tsec, W, B
	XORLW	0x3b
	BNZ	_00278_DS_
_00330_DS_:
	BANKSEL	_tsec
;	.line	386; lena.c	tsec = 0;
	CLRF	_tsec, B
	BRA	_00298_DS_
_00278_DS_:
	BANKSEL	_tsec
;	.line	388; lena.c	tsec++;
	INCF	_tsec, F, B
;	.line	390; lena.c	break;
	BRA	_00298_DS_
_00280_DS_:
;	.line	392; lena.c	whereami = SET_ALARM;
	MOVLW	0x06
	BANKSEL	_whereami
	MOVWF	_whereami, B
;	.line	393; lena.c	break;
	BRA	_00298_DS_
_00281_DS_:
;	.line	395; lena.c	alarm_set ^= 1;
	MOVLW	0x01
	BANKSEL	_alarm_set
	XORWF	_alarm_set, F, B
;	.line	396; lena.c	break;
	BRA	_00298_DS_
_00282_DS_:
	BANKSEL	_ahour
;	.line	398; lena.c	if (ahour == 23) {
	MOVF	_ahour, W, B
	XORLW	0x17
	BNZ	_00284_DS_
_00333_DS_:
	BANKSEL	_ahour
;	.line	399; lena.c	ahour = 0;
	CLRF	_ahour, B
	BRA	_00285_DS_
_00284_DS_:
	BANKSEL	_ahour
;	.line	401; lena.c	ahour++;
	INCF	_ahour, F, B
_00285_DS_:
;	.line	403; lena.c	ahour_o = ahour;
	MOVFF	_ahour, _ahour_o
;	.line	404; lena.c	break;
	BRA	_00298_DS_
_00286_DS_:
	BANKSEL	_amin
;	.line	406; lena.c	if (amin == 59) {
	MOVF	_amin, W, B
	XORLW	0x3b
	BNZ	_00288_DS_
_00335_DS_:
	BANKSEL	_amin
;	.line	407; lena.c	amin = 0;
	CLRF	_amin, B
	BRA	_00289_DS_
_00288_DS_:
	BANKSEL	_amin
;	.line	409; lena.c	amin++;
	INCF	_amin, F, B
_00289_DS_:
;	.line	411; lena.c	amin_o = amin;
	MOVFF	_amin, _amin_o
;	.line	412; lena.c	break;
	BRA	_00298_DS_
_00290_DS_:
;	.line	415; lena.c	break;
	BRA	_00298_DS_
_00291_DS_:
;	.line	418; lena.c	if (snooze < SNOOZE_MAX) {
	MOVLW	0x0c
	BANKSEL	_snooze
	SUBWF	_snooze, W, B
	BC	_00298_DS_
;	.line	419; lena.c	inc_amin(SNOOZE_MINUTE); // modifie le réveil
	MOVLW	0x05
	MOVWF	POSTDEC1
	CALL	_inc_amin
	INCF	FSR1L, F
	BANKSEL	_snooze
;	.line	420; lena.c	snooze++; // augmente le compteur de snooze
	INCF	_snooze, F, B
;	.line	421; lena.c	LATJbits.LATJ1 = 0; // switch LED 2 off
	BCF	_LATJbits, 1
;	.line	422; lena.c	LATJbits.LATJ2 = 0; // switch LED 3 off
	BCF	_LATJbits, 2
;	.line	423; lena.c	whereami = SNOOZE;
	MOVLW	0x0b
	BANKSEL	_whereami
	MOVWF	_whereami, B
;	.line	426; lena.c	break;
	BRA	_00298_DS_
_00294_DS_:
;	.line	429; lena.c	if (snooze < SNOOZE_MAX) {
	MOVLW	0x0c
	BANKSEL	_snooze
	SUBWF	_snooze, W, B
	BC	_00298_DS_
;	.line	430; lena.c	inc_amin(SNOOZE_MINUTE); // modifie le réveil
	MOVLW	0x05
	MOVWF	POSTDEC1
	CALL	_inc_amin
	INCF	FSR1L, F
	BANKSEL	_snooze
;	.line	431; lena.c	snooze++; // augmente le compteur de snooze
	INCF	_snooze, F, B
_00298_DS_:
	BANKSEL	_button2
;	.line	438; lena.c	button2 = 0; // remet le flag du boutton 2 à 0
	CLRF	_button2, B
_00304_DS_:
	MOVFF	PREINC1, r0x00
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_lena__manageseconds	code
_manageseconds:
;	.line	305; lena.c	void manageseconds() {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	BANKSEL	(_pseudoseconds + 1)
;	.line	306; lena.c	if (pseudoseconds >= PER) {
	MOVF	(_pseudoseconds + 1), W, B
	ADDLW	0x80
	ADDLW	0x80
	BNZ	_00250_DS_
	MOVLW	0x05
	BANKSEL	_pseudoseconds
	SUBWF	_pseudoseconds, W, B
_00250_DS_:
	BNC	_00247_DS_
;	.line	307; lena.c	overflows = overflows - CUT;
	MOVLW	0xfe
	BANKSEL	_overflows
	ADDWF	_overflows, F, B
	BC	_10389_DS_
	BANKSEL	(_overflows + 1)
	DECF	(_overflows + 1), F, B
_10389_DS_:
;	.line	308; lena.c	pseudoseconds = pseudoseconds - PER;
	MOVLW	0xfb
	BANKSEL	_pseudoseconds
	ADDWF	_pseudoseconds, F, B
	BC	_20390_DS_
	BANKSEL	(_pseudoseconds + 1)
	DECF	(_pseudoseconds + 1), F, B
_20390_DS_:
_00247_DS_:
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_lena__time	code
_time:
;	.line	257; lena.c	void time(void)
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	MOVFF	r0x00, POSTDEC1
	BANKSEL	_halfsecond
;	.line	260; lena.c	if (halfsecond) {
	MOVF	_halfsecond, W, B
	BZ	_00186_DS_
;	.line	261; lena.c	LED0_IO ^= 1;
	CLRF	r0x00
	BTFSC	_LATJbits, 0
	INCF	r0x00, F
	MOVLW	0x01
	XORWF	r0x00, F
	MOVF	r0x00, W
	ANDLW	0x01
	MOVWF	PRODH
	MOVF	_LATJbits, W
	ANDLW	0xfe
	IORWF	PRODH, W
	MOVWF	_LATJbits
	BANKSEL	_halfsecond
;	.line	262; lena.c	halfsecond = 0;
	CLRF	_halfsecond, B
_00186_DS_:
	BANKSEL	_new_time
;	.line	266; lena.c	if (new_time) {
	MOVF	_new_time, W, B
	BZ	_00188_DS_
	BANKSEL	_new_time
;	.line	267; lena.c	inc_tsec(new_time);
	MOVF	_new_time, W, B
	MOVWF	POSTDEC1
	CALL	_inc_tsec
	INCF	FSR1L, F
	BANKSEL	_new_time
;	.line	268; lena.c	new_time = 0;
	CLRF	_new_time, B
;	.line	270; lena.c	LED0_IO ^= 1;
	CLRF	r0x00
	BTFSC	_LATJbits, 0
	INCF	r0x00, F
	MOVLW	0x01
	XORWF	r0x00, F
	MOVF	r0x00, W
	ANDLW	0x01
	MOVWF	PRODH
	MOVF	_LATJbits, W
	ANDLW	0xfe
	IORWF	PRODH, W
	MOVWF	_LATJbits
_00188_DS_:
	BANKSEL	_thour
;	.line	274; lena.c	if ((thour == ahour) && (tmin == amin) && alarm_set) {
	MOVF	_thour, W, B
	BANKSEL	_ahour
	XORWF	_ahour, W, B
	BZ	_00226_DS_
	BRA	_00209_DS_
_00226_DS_:
	BANKSEL	_tmin
	MOVF	_tmin, W, B
	BANKSEL	_amin
	XORWF	_amin, W, B
	BZ	_00228_DS_
	BRA	_00209_DS_
_00228_DS_:
	BANKSEL	_alarm_set
	MOVF	_alarm_set, W, B
	BTFSC	STATUS, 2
	BRA	_00209_DS_
;	.line	277; lena.c	if ((tsec < 31) && (stop_ringing == 0)) {
	MOVLW	0x1f
	BANKSEL	_tsec
	SUBWF	_tsec, W, B
	BC	_00202_DS_
	BANKSEL	_stop_ringing
	MOVF	_stop_ringing, W, B
	BNZ	_00202_DS_
	BANKSEL	_whereami
;	.line	278; lena.c	if ((whereami == DISPLAY) || (whereami == SNOOZE)) {
	MOVF	_whereami, W, B
	XORLW	0x09
	BZ	_00189_DS_
_00231_DS_:
	BANKSEL	_whereami
	MOVF	_whereami, W, B
	XORLW	0x0b
	BNZ	_00190_DS_
_00189_DS_:
;	.line	279; lena.c	whereami = ALARM;
	MOVLW	0x0a
	BANKSEL	_whereami
	MOVWF	_whereami, B
_00190_DS_:
	BANKSEL	_whereami
;	.line	281; lena.c	if (whereami == ALARM) {
	MOVF	_whereami, W, B
	XORLW	0x0a
	BZ	_00235_DS_
	BRA	_00209_DS_
_00235_DS_:
;	.line	282; lena.c	LED1_IO ^= 1; //change state of red leds
	CLRF	r0x00
	BTFSC	_LATJbits, 1
	INCF	r0x00, F
	MOVLW	0x01
	XORWF	r0x00, F
	MOVF	r0x00, W
	ANDLW	0x01
	RLNCF	WREG, W
	MOVWF	PRODH
	MOVF	_LATJbits, W
	ANDLW	0xfd
	IORWF	PRODH, W
	MOVWF	_LATJbits
;	.line	283; lena.c	LED2_IO ^= 1; //change state of red leds
	CLRF	r0x00
	BTFSC	_LATJbits, 2
	INCF	r0x00, F
	MOVLW	0x01
	XORWF	r0x00, F
	MOVF	r0x00, W
	ANDLW	0x01
	RLNCF	WREG, W
	RLNCF	WREG, W
	MOVWF	PRODH
	MOVF	_LATJbits, W
	ANDLW	0xfb
	IORWF	PRODH, W
	MOVWF	_LATJbits
	BRA	_00209_DS_
_00202_DS_:
;	.line	286; lena.c	} else if (tsec > 30) {
	MOVLW	0x1f
	BANKSEL	_tsec
	SUBWF	_tsec, W, B
	BNC	_00209_DS_
	BANKSEL	_stop_ringing
;	.line	287; lena.c	stop_ringing = 0; // remet à 0 si l'alarme a été éteinte à la main
	CLRF	_stop_ringing, B
;	.line	288; lena.c	LATJbits.LATJ1 = 0; // switch LED 2 off
	BCF	_LATJbits, 1
;	.line	289; lena.c	LATJbits.LATJ2 = 0; // switch LED 3 off
	BCF	_LATJbits, 2
	BANKSEL	_snooze
;	.line	291; lena.c	if (snooze) {
	MOVF	_snooze, W, B
	BZ	_00197_DS_
;	.line	292; lena.c	ahour = ahour_o; // remet le réveil
	MOVFF	_ahour_o, _ahour
;	.line	293; lena.c	amin = amin_o;
	MOVFF	_amin_o, _amin
	BANKSEL	_snooze
;	.line	294; lena.c	snooze = 0;
	CLRF	_snooze, B
;	.line	295; lena.c	whereami = DISPLAY;
	MOVLW	0x09
	BANKSEL	_whereami
	MOVWF	_whereami, B
	BRA	_00209_DS_
_00197_DS_:
	BANKSEL	_whereami
;	.line	296; lena.c	} else if (whereami == ALARM) { // si l'alarme sonnait toujours, on
	MOVF	_whereami, W, B
	XORLW	0x0a
	BNZ	_00209_DS_
;	.line	297; lena.c	whereami = DISPLAY;         // revient à l'affichage de l'heure
	MOVLW	0x09
	BANKSEL	_whereami
	MOVWF	_whereami, B
_00209_DS_:
	MOVFF	PREINC1, r0x00
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_lena__dec_amin	code
_dec_amin:
;	.line	245; lena.c	void dec_amin(BYTE val)
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	MOVFF	r0x00, POSTDEC1
	MOVFF	r0x01, POSTDEC1
	MOVLW	0x02
	MOVFF	PLUSW2, r0x00
;	.line	248; lena.c	if (val > amin) {
	MOVF	r0x00, W
	BANKSEL	_amin
	SUBWF	_amin, W, B
	BC	_00175_DS_
;	.line	249; lena.c	dec_ahour(1);
	MOVLW	0x01
	MOVWF	POSTDEC1
	CALL	_dec_ahour
	INCF	FSR1L, F
;	.line	250; lena.c	amin = (60+amin) - val;
	MOVLW	0x3c
	BANKSEL	_amin
	ADDWF	_amin, W, B
	MOVWF	r0x01
	MOVF	r0x00, W
	SUBWF	r0x01, W
	BANKSEL	_amin
	MOVWF	_amin, B
	BRA	_00177_DS_
_00175_DS_:
;	.line	252; lena.c	amin  = amin - val;
	MOVF	r0x00, W
	BANKSEL	_amin
	SUBWF	_amin, F, B
_00177_DS_:
	MOVFF	PREINC1, r0x01
	MOVFF	PREINC1, r0x00
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_lena__dec_ahour	code
_dec_ahour:
;	.line	233; lena.c	void dec_ahour(BYTE val)
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	MOVFF	r0x00, POSTDEC1
	MOVFF	r0x01, POSTDEC1
	MOVLW	0x02
	MOVFF	PLUSW2, r0x00
;	.line	236; lena.c	if (val > ahour) {
	MOVF	r0x00, W
	BANKSEL	_ahour
	SUBWF	_ahour, W, B
	BC	_00164_DS_
;	.line	237; lena.c	ahour = (24+ahour) - val;
	MOVLW	0x18
	BANKSEL	_ahour
	ADDWF	_ahour, W, B
	MOVWF	r0x01
	MOVF	r0x00, W
	SUBWF	r0x01, W
	BANKSEL	_ahour
	MOVWF	_ahour, B
	BRA	_00166_DS_
_00164_DS_:
;	.line	239; lena.c	ahour = ahour - val;
	MOVF	r0x00, W
	BANKSEL	_ahour
	SUBWF	_ahour, F, B
_00166_DS_:
	MOVFF	PREINC1, r0x01
	MOVFF	PREINC1, r0x00
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_lena__inc_amin	code
_inc_amin:
;	.line	220; lena.c	void inc_amin(BYTE val)
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	MOVFF	r0x00, POSTDEC1
	MOVFF	r0x01, POSTDEC1
	MOVFF	r0x02, POSTDEC1
	MOVFF	r0x03, POSTDEC1
	MOVLW	0x02
	MOVFF	PLUSW2, r0x00
;	.line	224; lena.c	mod_amin = (amin + val) / 60;
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
;	.line	225; lena.c	if (mod_amin) {
	MOVF	r0x01, W
	BZ	_00157_DS_
;	.line	226; lena.c	inc_ahour(mod_amin);
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	CALL	_inc_ahour
	INCF	FSR1L, F
_00157_DS_:
;	.line	229; lena.c	amin = (amin + val) % 60;
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
S_lena__inc_ahour	code
_inc_ahour:
;	.line	214; lena.c	void inc_ahour(BYTE val)
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	MOVFF	r0x00, POSTDEC1
	MOVFF	r0x01, POSTDEC1
	MOVFF	r0x02, POSTDEC1
	MOVFF	r0x03, POSTDEC1
	MOVLW	0x02
	MOVFF	PLUSW2, r0x00
;	.line	216; lena.c	ahour = (ahour + val) % 24;
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
S_lena__inc_tsec	code
_inc_tsec:
;	.line	194; lena.c	void inc_tsec(BYTE val)
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
	MOVLW	0x02
	MOVFF	PLUSW2, r0x00
;	.line	199; lena.c	mod_tsec = (tsec + val) / 60;
	MOVFF	_tsec, r0x01
	CLRF	r0x02
	CLRF	r0x03
	MOVF	r0x00, W
	ADDWF	r0x01, W
	MOVWF	r0x00
	MOVF	r0x03, W
	ADDWFC	r0x02, W
	MOVWF	r0x01
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x3c
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__divsint
	MOVWF	r0x02
	MOVFF	PRODL, r0x03
	MOVLW	0x04
	ADDWF	FSR1L, F
;	.line	200; lena.c	if (mod_tsec) {
	MOVF	r0x02, W
	BTFSC	STATUS, 2
	BRA	_00145_DS_
;	.line	202; lena.c	mod_tmin = (tmin + mod_tsec) / 60;
	MOVFF	_tmin, r0x03
	CLRF	r0x04
	CLRF	r0x05
	MOVF	r0x02, W
	ADDWF	r0x03, W
	MOVWF	r0x02
	MOVF	r0x05, W
	ADDWFC	r0x04, W
	MOVWF	r0x03
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x3c
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	CALL	__divsint
	MOVWF	r0x04
	MOVFF	PRODL, r0x05
	MOVLW	0x04
	ADDWF	FSR1L, F
;	.line	203; lena.c	if (mod_tmin) {
	MOVF	r0x04, W
	BZ	_00143_DS_
;	.line	204; lena.c	thour = (thour + mod_tmin) % 24;
	MOVFF	_thour, r0x05
	CLRF	r0x06
	CLRF	r0x07
	MOVF	r0x04, W
	ADDWF	r0x05, F
	MOVF	r0x07, W
	ADDWFC	r0x06, F
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x18
	MOVWF	POSTDEC1
	MOVF	r0x06, W
	MOVWF	POSTDEC1
	MOVF	r0x05, W
	MOVWF	POSTDEC1
	CALL	__modsint
	MOVWF	r0x04
	MOVFF	PRODL, r0x05
	MOVLW	0x04
	ADDWF	FSR1L, F
	MOVF	r0x04, W
	BANKSEL	_thour
	MOVWF	_thour, B
_00143_DS_:
;	.line	207; lena.c	tmin = (tmin + mod_tsec) % 60;
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x3c
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	CALL	__modsint
	MOVWF	r0x02
	MOVFF	PRODL, r0x03
	MOVLW	0x04
	ADDWF	FSR1L, F
	MOVF	r0x02, W
	BANKSEL	_tmin
	MOVWF	_tmin, B
_00145_DS_:
;	.line	210; lena.c	tsec = (tsec + val) % 60;
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x3c
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__modsint
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
	MOVLW	0x04
	ADDWF	FSR1L, F
	MOVF	r0x00, W
	BANKSEL	_tsec
	MOVWF	_tsec, B
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
S_lena__low_isr	code
_low_isr:
;	.line	124; lena.c	void low_isr (void) interrupt 2
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
;	.line	127; lena.c	if(INTCON3bits.INT3F) {
	BTFSS	_INTCON3bits, 2
	BRA	_00127_DS_
;	.line	128; lena.c	button1 = 1;
	MOVLW	0x01
	BANKSEL	_button1
	MOVWF	_button1, B
;	.line	129; lena.c	INTCON3bits.INT3F = 0;   //clear INT1 flag
	BCF	_INTCON3bits, 2
	BRA	_00129_DS_
_00127_DS_:
;	.line	132; lena.c	} else if(INTCON3bits.INT1F) {
	BTFSS	_INTCON3bits, 0
	BRA	_00129_DS_
;	.line	133; lena.c	button2 = 1;
	MOVLW	0x01
	BANKSEL	_button2
	MOVWF	_button2, B
;	.line	134; lena.c	INTCON3bits.INT1F = 0;
	BCF	_INTCON3bits, 0
_00129_DS_:
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
S_lena__high_isr	code
_high_isr:
;	.line	103; lena.c	void high_isr (void) interrupt 1
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
;	.line	106; lena.c	if (INTCONbits.T0IF) {
	BTFSS	_INTCONbits, 2
	BRA	_00111_DS_
	BANKSEL	_overflows
;	.line	107; lena.c	overflows++;
	INCF	_overflows, F, B
	BNC	_30391_DS_
	BANKSEL	(_overflows + 1)
	INCF	(_overflows + 1), F, B
_30391_DS_:
	BANKSEL	_overflows
;	.line	109; lena.c	if (overflows == OPS/2) {
	MOVF	_overflows, W, B
	XORLW	0x2f
	BNZ	_00116_DS_
	BANKSEL	(_overflows + 1)
	MOVF	(_overflows + 1), W, B
	BZ	_00117_DS_
_00116_DS_:
	BRA	_00106_DS_
_00117_DS_:
;	.line	110; lena.c	halfsecond = 1;
	MOVLW	0x01
	BANKSEL	_halfsecond
	MOVWF	_halfsecond, B
_00106_DS_:
	BANKSEL	_overflows
;	.line	113; lena.c	if (overflows == OPS) {
	MOVF	_overflows, W, B
	XORLW	0x5f
	BNZ	_00118_DS_
	BANKSEL	(_overflows + 1)
	MOVF	(_overflows + 1), W, B
	BZ	_00119_DS_
_00118_DS_:
	BRA	_00108_DS_
_00119_DS_:
	BANKSEL	_pseudoseconds
;	.line	114; lena.c	pseudoseconds++;
	INCF	_pseudoseconds, F, B
	BNC	_40392_DS_
	BANKSEL	(_pseudoseconds + 1)
	INCF	(_pseudoseconds + 1), F, B
_40392_DS_:
	BANKSEL	_new_time
;	.line	115; lena.c	new_time++;
	INCF	_new_time, F, B
	BANKSEL	_overflows
;	.line	116; lena.c	overflows = 0;
	CLRF	_overflows, B
	BANKSEL	(_overflows + 1)
	CLRF	(_overflows + 1), B
_00108_DS_:
;	.line	118; lena.c	INTCONbits.T0IF = 0;
	BCF	_INTCONbits, 2
_00111_DS_:
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
; code size:	 4398 (0x112e) bytes ( 3.36%)
;           	 2199 (0x0897) words
; udata size:	   32 (0x0020) bytes ( 0.83%)
; access size:	   22 (0x0016) bytes


	end

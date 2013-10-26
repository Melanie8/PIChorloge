/******************************************************************************
 
 FICHIER : findfreq.c
 ------------------------------------------------------------------------------
 AUTEURS : Lena Peschke et Mélanie Sedda
 ------------------------------------------------------------------------------
 OBJECTIF : Trouver la véritable fréquence de notre pic
 ------------------------------------------------------------------------------
 STRATEGIE :
 
 Nous avons configuré le timer1 pour qu'il overflow après 2 secondes.
 L'idée de base est alors de mesurer le nombre de fois que le timer0
 overflow pendant ce laps de temps. Nous estimerons alors que :
 
    (nombre d'instructions par seconde) = overflows * 2^16 / 2 et
    f = 4 * (nombre d'instructions par seconde)
 
 Afin d'avoir une erreur relative la plus petite possible nous
    - n'avons pas utilisé de prescaler sur le timer0,
    - avons effectué notre mesure sur plus de 2 secondes.
 
 En effet,
    f_exacte = 4 * (overflows * 2^16 + value) / N
    où  - value est la valeur qui se trouve dans le timer0 pile au moment de l'overflow du timer1
        - N est le temps de notre mesure en secondes
 
    f_mesuree = 4 * (overflows * 2^16) / N
    (erreur relative) = |f_exacte - f_mesuree| / f_exacte
                      = value / (overflows * 2^16 + value)
 
 Notre erreur relative sera donc plus petite lorsque overflows est plus grand.
 C'est pourquoi utiliser un prescaler sur le timer0 serait contreproductif
 (cela diviserait le nombre d'overflows par la valeur du prescaler) et
 augmenter le temps de mesure est intéressant (cela augmentera le nombre
 d'overflows).
 
 Nous avons pris le parti de ne pas mesurer la valeur se trouvant dans
 le timer0 après l'overflow du timer1 parce que cette valeur est négligeable.
 L'erreur relative maximale en ne la mesurant pas est en effet
 <= 1 / (overflows + 1).
 De plus, on ne trouverait de toute façon pas précisément la valeur qui se
 trouvait dans le timer0 pile au moment de l'overflow.
 
 Nous avons effectué 3 mesures sur un laps de temps de 10 minutes et avons
 trouvé qu'en moyenne
 le nombre d'overflows/s        = 95,366
 le nombre d'incrémentation/s   = 95,366 * 2^16      = 6249906,176
 la fréquence du PIC            = 6249906,176 * 4    = 24999624 Hz
 ce qui représente un écart de 0.0015% par rapport à la fréquence de 25 MHz
 annoncée dans la documentation du PIC.
 
 *****************************************************************************/

#define __18F97J60
#define __SDCC__
#define THIS_INCLUDES_THE_MAIN_FUNCTION
#include "Include/HardwareProfile.h"

#include <string.h>
#include <stdlib.h>

#include "Include/LCDBlocking.h"
#include "Include/TCPIP_Stack/Delay.h"

void DisplayString(BYTE pos, char *text);
size_t strlcpy(char *dst, const char *src, size_t siz);

/* strings pour le LCD */
CHAR s[16];
CHAR t[16];

/* stockage de la valeur dans timer0 */
UINT low;
UINT high;
UINT value;

/* nombre d'overflows du timer0 */
UINT overflows = 0;

/* nombre de secondes qu'on veut laisser passer (doit être un nombre pair)
 * et de secondes déjà passées */
UINT N = 600;
UINT seconds = 0;


/* routine d'interruption */
void timer_overflow() interrupt 1 {
    
    // si le timer0 overflow, on incrémente notre compteur
    if(INTCONbits.TMR0IF){
        overflows ++;
		INTCONbits.TMR0IF = 0;
	}
    
    // si le timer1 overflow, on incrémente de 2 le nombre de secondes passées
	if(PIR1bits.TMR1IF){
        seconds = seconds+2;
        
        // si on a atteint le nombre de secondes voulues, on affiche overflows
        if (seconds == N) {
            sprintf(t, "%u", overflows);
            DisplayString (0, t);
            
            // on stoppe les timers
            T1CONbits.TMR1ON = 0;
            T0CONbits.TMR0ON = 0;
        }
        
        PIR1bits.TMR1IF = 0;
	}
}

void main(void) {
    
    /* bits d'interruption */
    RCONbits.IPEN   = 1; //enable priority levels on interrupts
    INTCONbits.GIE  = 1; //enables all high-priority interrupts
    INTCONbits.PEIE = 0; //disables all low-priority peripheral interrupts
    
    /* LCD */
	LCDInit();
    
    /* timer0 */
    T0CONbits.TMR0ON    = 1; //enables Timer0
    INTCONbits.TMR0IE   = 1; //enables the TMR0 overflow interrupt
    INTCONbits.TMR0IF   = 0; //clear Timer0 overflow bit
    INTCON2bits.TMR0IP  = 1; //high priority
    T0CONbits.T08BIT    = 0; //timer0 is configured as a 16-bit timer/counter
	T0CONbits.T0CS      = 0; //internal instruction cycle clock (CLKO)
	T0CONbits.PSA       = 1; //timer0 prescaler is not assigned
	TMR0L = 0;    TMR0H = 0;
    
    /* timer1 */
    T1CONbits.TMR1ON    = 1; //enables Timer1
    PIE1bits.TMR1IE    	= 1; //enables Timer1 interrupt
    PIR1bits.TMR1IF    	= 0; //clear Timer1 overflow bit
	IPR1bits.TMR1IP		= 1; //high priority
	T1CONbits.T1RUN     = 0; //device clock is derived from another source
	T1CONbits.TMR1CS    = 1; //external clock from RC0/T1OSO/T13CKI pin
                             //(on the rising edge)
    
	T1CONbits.T1OSCEN 	= 1; //timer1 oscillator is enabled
	T1CONbits.T1CKPS1 	= 0; //disable prescaling
	T1CONbits.T1CKPS0 	= 0; //disable prescaling
    TMR1L = 0;    TMR1H = 0; // -> /!\ interruptions toutes les 2 secondes
    
    /* départ! */
    DisplayString(0,"C'est parti!");
    T1CONbits.TMR1ON = 1; //enable timer1
	T0CONbits.TMR0ON = 1; //enable timer0
    
}

#if defined(__SDCC__)
/*************************************************
 Function DisplayString:
 Writes an IP address to string to the LCD display
 starting at pos
 *************************************************/
void DisplayString(BYTE pos, char* text)
{
    BYTE l= strlen(text)+1;
    BYTE max= 32-pos;
    strlcpy((char*)&LCDText[pos], text,(l<max)?l:max );
    LCDUpdate();
    
}
#endif

/*-------------------------------------------------------------------------
 *
 * strlcpy.c
 *    strncpy done right
 *
 * This file was taken from OpenBSD and is used on platforms that don't
 * provide strlcpy().  The OpenBSD copyright terms follow.
 *-------------------------------------------------------------------------
 */

/*  $OpenBSD: strlcpy.c,v 1.11 2006/05/05 15:27:38 millert Exp $    */

/*
 * Copyright (c) 1998 Todd C. Miller <Todd.Miller@courtesan.com>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

/*
 * Copy src to string dst of size siz.  At most siz-1 characters
 * will be copied.  Always NUL terminates (unless siz == 0).
 * Returns strlen(src); if retval >= siz, truncation occurred.
 * Function creation history:  http://www.gratisoft.us/todd/papers/strlcpy.html
 */
size_t
strlcpy(char *dst, const char *src, size_t siz)
{
    char       *d = dst;
    const char *s = src;
    size_t      n = siz;
    
    /* Copy as many bytes as will fit */
    if (n != 0)
    {
        while (--n != 0)
        {
            if ((*d++ = *s++) == '\0')
                break;
        }
    }
    
    /* Not enough room in dst, add NUL and traverse rest of src */
    if (n == 0)
    {
        if (siz != 0)
            *d = '\0';          /* NUL-terminate dst */
        while (*s++)
            ;
    }
    
    return (s - src - 1);       /* count does not include NUL */
}

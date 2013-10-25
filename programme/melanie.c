/*H*********************************************************************H*
 *
 * FILENAME :       reveil.c
 *
 * DESCRIPTION :
 *
 * NOTES :
 *
 * AUTHOR :         Lena Peschke
 *                  Mélanie Sedda
 *
 * DATE :           Oct. 27st 2013
 *
 *H*********************************************************************H*/

#define __18F97J60
#define __SDCC__
#define THIS_INCLUDES_THE_MAIN_FUNCTION
#include "Include/HardwareProfile.h"

#include <string.h>
#include <stdlib.h>

#include "Include/LCDBlocking.h"
#include "Include/TCPIP_Stack/Delay.h"

/* states */
#define TIME_MENU 1
#define SET_HOUR 2
#define SET_MINUTE 3
#define SET_SECOND 4
#define ALARM_MENU 5
#define SET_ALARM 6
#define SET_A_HOUR 7
#define SET_A_MIN 8
#define DISPLAY 9
#define ALARM 10
#define SNOOZE 11

#define SNOOZE_MIN 5

/* OPS est le nombre d'overflows du timer0 en 1 seconde, que nous avons mesuré
 * dans findfreq. Pour être plus précis, toutes les PER pseudosecondes (càd après
 * PER*OPS overflows), il faudra retirer CUT nombre d'overflows
 */
#define OPS 95
#define PER 5
#define CUT 2

void time(void);
void manageseconds(void);
void led(void);
void button(void);
void refresh_lcd(void);
void snooze_alarm(void);
void DisplayString(BYTE pos, char* text);
void DisplayWORD(BYTE pos, WORD w);
size_t strlcpy(char *dst, const char *src, size_t siz);

BYTE chandelle = 1;

/* l'heure actuelle */
BYTE thour = 0;
BYTE tmin = 0;
BYTE tsec = 0;

/* l'heure de l'alarme */
BYTE alarm_set = 0; // dis si l'alarme est mise
BYTE ahour = 0;
BYTE amin = 0;

/* l'état de l'alarme */
BYTE snooze = 0; // compte le nombre de snooze
BYTE stop_ringing = 0; // dis si on a eteint le reveil

/* l'état du réveil */
BYTE whereami = 0;

/* les boutons */
BYTE button1 = 0;
BYTE button2 = 0;

/* seconde et demi-seconde */
BYTE new_time = 0; // compteur de nouvelles secondes écoulées, utile pour modifier l'heure (sera remis à 0 dès que tsec sera incrémenté de new_time)
BYTE halfsecond = 0; // flag demi-seconde


/* variables utiles pour que la seconde soit précise */
int overflows = 0; // nombre d'overflows du timer0
int pseudoseconds = 0; // compteur du nombre de secondes écoulées présumées sur base de OPS, utile pour savoir quand on devra décrémenter overflows de CUT pour retrouver notre précision d'une seconde (cela se fera quand pseudoseconds vaudra PER et il sera alors remis à 0)


/* Interruption de priorité haute : liée au temps */
void high_isr (void) interrupt 1
{
    // après chaque overflow, on incrémente overflows
    if (INTCONbits.T0IF) {
        overflows++;
        // quand on est au demi-overflow, on met le flag halfsecond à 1 (juste utile pour les LED)
        if (overflows == OPS/2) {
            halfsecond = 1;
        }
        // quand on overflow, on incrémente nos compteurs de nombres de secondes (celui lié à la modification de l'heure actuelle et celui lié au réglage précis de la seconde) et on remets le nombre d'overflows à 0
        if (overflows == OPS) {
            pseudoseconds++;
            new_time++;
            overflows = 0;
        }
        INTCONbits.T0IF = 0;
    }
}

/* Interruption de priorité basse : liée aux boutons */
void low_isr (void) interrupt 2
{
    // button 1
    if(INTCON3bits.INT3F) {
        button1 = 1;
        INTCON3bits.INT3F = 0;   //clear INT1 flag
        
        // button 2
    } else if(INTCON3bits.INT1F) {
        button2 = 1;
        INTCON3bits.INT1F = 0;
    }
}


void main() {
    /* bits d'interruption */
    RCONbits.IPEN       = 1; //enable interrupts priority levels
    INTCONbits.GIE      = 1; //enables all high-priority interrupts
    INTCONbits.PEIE     = 1; //enables all low-priority peripheral interrupts
    
    /* timer0 */
    T0CONbits.TMR0ON    = 1; //enables Timer0
    INTCONbits.TMR0IE   = 1; //enables the TMR0 overflow interrupt
    INTCONbits.TMR0IF   = 0; //clear Timer0 overflow bit
    INTCON2bits.TMR0IP  = 1; //high priority
    T0CONbits.T08BIT    = 0; //timer0 is configured as a 16-bit timer/counter
	T0CONbits.T0CS      = 0; //internal instruction cycle clock (CLKO)
    T0CONbits.PSA       = 1; //timer0 prescaler is not assigned
	TMR0L = 0;    TMR0H = 0;
    
    /* LED */
    LED0_TRIS = 0; //configure 1st led pin as output (yellow)
    LED1_TRIS = 0; //configure 2nd led pin as output (red)
    LED2_TRIS = 0; //configure 3rd led pin as output (red)
    
    /* buttons */
    BUTTON0_TRIS        = 1; //configure 1st button as input
    BUTTON1_TRIS        = 1; //configure 2nd button as input
    INTCON3bits.INT3E   = 1; //enable INT3 interrupt (button 1)
    INTCON3bits.INT3F   = 0; //clear INT3 flag
    INTCON3bits.INT1E   = 1; //enable INT1 interrupt (button 2)
    INTCON3bits.INT1F   = 0; //clear INT1 flag
    INTCON3bits.INT1IP  = 0; //low priority
    INTCON2bits.INT3IP  = 0; //low priority
    INTCON2bits.INTEDG1 = 0; //INT1 interrupts on falling edge ?????
    INTCON2bits.INTEDG3 = 0; //INT3 interrupts on falling edge ?????
    
    //////////////////////////////////////////////
    
    LCDInit();
    whereami = TIME_MENU;
    
    chandelle++; // ##### BIZARRE ####
    
    T0CONbits.TMR0ON = 1; // start timer0
    while (1) {
        time();
        manageseconds();
        led();
        button();
        refresh_lcd();
    }
}

void inc_thour(BYTE val) {
    thour = (thour + val) % 24;
}

void inc_tmin(BYTE val) {
    BYTE mod_tmin;
    
    mod_tmin = (tmin + val) / 60;
    if (mod_tmin) {
        inc_thour(mod_tmin);
    }
    tmin = (tmin + val) % 60;
}

void inc_tsec(BYTE val) {
    BYTE mod_tsec;
    
    mod_tsec = (tsec + val) / 60;
    if (mod_tsec) {
        inc_tmin(mod_tsec);
    }
    tsec = (tsec + val) % 60;
    
}

void time(void) {
    if (new_time) {
        LATJbits.LATJ0^=1; // toggle LED
        inc_tsec(new_time);
        new_time = 0;
    }
    
    if (alarm_set && (thour == ahour) && (tmin == amin)) {
        if ((tsec < 31) && (stop_ringing == 0) && (whereami == DISPLAY)) { // l'alarme ne perturbe pas le menu ? ##
            whereami = ALARM;
        } else if (tsec > 30) {
            stop_ringing = 0;
            if (whereami == ALARM) {
                whereami = DISPLAY;
            }
        }
    }
}

/* fonction qui gère la correction pour que la seconde soit précise */
void manageseconds() {
    if (pseudoseconds >= PER) {
        overflows = overflows - CUT;
        pseudoseconds = pseudoseconds - PER;
    }
}

/* //gros pate
 if (alarm_set && (thour == ahour) && (tmin == amin)) {
 if (!start_ringing) {
 start_ringing = 1;
 whereami = ALARM;
 }
 if (tsec == 30) {
 stop_ringing = 1;
 whereami = DISPLAY;
 }
 }
 if (start_ringing && !stop_ringing) {
 LED0_IO ^= 1;
 }
 if (alarm_set && (tmin == amin+1)) {
 start_ringing = 0;
 stop_ringing = 0;
 }
 //jusqu'ici */

void led(void) {
    
    //    if ((whereami != ALARM) && (whereami != SNOOZE)) {
    //        //LED0_IO ^= 1;
    //        // activer une led!
    //    }
    // on fait d'office clignoter la LED jaune avec une période de 1s
    if (halfsecond) {
        LED0_IO ^= 1;
        halfsecond = 0;
    }
    if (new_time)
        LED0_IO ^= 1;
    
    if (whereami == ALARM) {
        LED1_IO ^= 1; //change state of red leds
        LED2_IO ^= 1; //change state of red leds
        if (halfsecond) {
            LED1_IO ^= 1; //change state of red leds
            LED2_IO ^= 1; //change state of red leds
            halfsecond = 0;
        }
    }
}

// refaire avec des modulos
/*void time() {
 if (new_time) {
 if (tsec == 59) {
 tsec = 0;
 if (tmin == 59) {
 tmin = 0;
 if (thour == 23) {
 thour = 0;
 } else {
 thour++;
 }
 } else {
 tmin++;
 }
 } else {
 tsec++;
 }
 // gros pate
 if (alarm_set && (thour == ahour) && (tmin == amin)) {
 if (!start_ringing) {
 start_ringing = 1;
 whereami = ALARM;
 }
 if (tsec == 30) {
 stop_ringing = 1;
 whereami = DISPLAY;
 }
 }
 if (start_ringing && !stop_ringing) {
 LED0_IO ^= 1;
 }
 if (alarm_set && (tmin == amin+1)) {
 start_ringing = 0;
 stop_ringing = 0;
 }
 // jusqu'ici
 }
 }*/



void button(void) {
    
    if (button1) {
        switch (whereami) {
            case TIME_MENU:
                whereami = ALARM_MENU;
                break;
            case SET_HOUR:
                whereami = SET_MINUTE;
                break;
            case SET_MINUTE:
                whereami = SET_SECOND;
                break;
            case SET_SECOND:
                whereami = ALARM_MENU;
                break;
            case ALARM_MENU:
                whereami = DISPLAY;
                break;
            case SET_ALARM:
                whereami = SET_A_HOUR;
                break;
            case SET_A_HOUR:
                whereami = SET_A_MIN;
                break;
            case SET_A_MIN:
                whereami = DISPLAY;
                break;
            case DISPLAY:
                whereami = TIME_MENU;
                break;
            case ALARM:
                stop_ringing = 1;
                whereami = DISPLAY;
                break;
            case SNOOZE:
                stop_ringing = 1;
                // ++++++ a revoir d'urgence! +++++
                // amin = (amin - (snooze * SNOOZE_MIN));
                snooze = 0;
                whereami = DISPLAY;
                break;
            default:
                break;
        }
        button1 = 0;
        
    } else if (button2) {
        switch (whereami) {
            case TIME_MENU:
                whereami = SET_HOUR;
                break;
            case SET_HOUR:
                thour++;
                if (thour == 24) {
                    thour = 0;
                }
                break;
            case SET_MINUTE:
                if (tmin == 59) {
                    tmin = 0;
                } else {
                    tmin++;
                }
                break;
            case SET_SECOND:
                if (tsec == 59) {
                    tsec = 0;
                } else {
                    tsec++;
                }
                break;
            case ALARM_MENU:
                whereami = SET_ALARM;
                break;
            case SET_ALARM:
                alarm_set ^= 1;
                break;
            case SET_A_HOUR:
                if (ahour == 23) {
                    ahour = 0;
                } else {
                    ahour++;
                }
                break;
            case SET_A_MIN:
                if (amin == 59) {
                    amin = 0;
                } else {
                    amin++;
                }
                break;
            case DISPLAY:
                /* nothing to be done */
                break;
            case ALARM:
                whereami = SNOOZE;
                break;
            case SNOOZE:
                snooze_alarm(); // ****** ajouter une limite et decider de ce qui se passe *****
                break;
            default:
                break;
        }
        button2 = 0;
    }
}

void refresh_lcd(void) {
    
    CHAR display[32];
    
    switch (whereami) {
        case TIME_MENU:
            sprintf(display, "Do you want to  set the time ?  ");
            break;
        case SET_HOUR:
            sprintf(display, " [%02u]: %02u : %02u                  ",
                    thour, tmin, tsec); // ***blink***
            break;
        case SET_MINUTE:
            sprintf(display, "  %02u :[%02u]: %02u                  ",
                    thour, tmin, tsec); // ***blink***
            break;
        case SET_SECOND:
            sprintf(display, "  %02u : %02u :[%02u]                 ",
                    thour, tmin, tsec); // ***blink***
            break;
        case ALARM_MENU:
            sprintf(display, "Do you want to  set the alarm ? ");
            break;
        case SET_ALARM:
            if (alarm_set) {
                sprintf(display, "  Alarm [ON ]                   "); // ***blink***
            } else {
                sprintf(display, "  Alarm [OFF]                   "); // ***blink***
            }
            break;
        case SET_A_HOUR:
            sprintf(display, "    Alarm at        [%02u]: %02u    ",
                    ahour, amin); // ***blink***
            break;
        case SET_A_MIN:
            sprintf(display, "    Alarm at         %02u :[%02u]   ",
                    ahour, amin); // ***blink***
            break;
        case DISPLAY:
            if (alarm_set) {
                sprintf(display, "    %02u:%02u:%02u    Alarm ON  %02u:%02u ",
                        thour, tmin, tsec, ahour, amin);
            } else {
                sprintf(display, "    %02u:%02u:%02u       Alarm  OFF   ",
                        thour, tmin, tsec);
            }
            break;
        case ALARM:
            sprintf(display, "    %02u:%02u:%02u      I am ringing! ",
                    thour, tmin, tsec); // ***blink***
            break;
            // ****** A DISCUTER *******
        case SNOOZE:
            sprintf(display, "    %02u:%02u:%02u     Snooze nr. %u   ",
                    thour, tmin, tsec, snooze); // ***blink***
            break;
        default:
            sprintf(display, "**** ERROR ********* ERROR *****");
            break;
    }
    DisplayString(0, display);
}

void snooze_alarm(void) {
    if ((amin = (amin + SNOOZE_MIN) % 60) != 0) {
        ahour++;
    }
    snooze++;
}

/*************************************************
 Function DisplayWORD:
 writes a WORD in hexa on the position indicated by
 pos.
 - pos=0 -> 1st line of the LCD
 - pos=16 -> 2nd line of the LCD
 
 __SDCC__ only: for debugging
 *************************************************/
#if defined(__SDCC__)
void DisplayWORD(BYTE pos, WORD w) //WORD is a 16 bits unsigned
{
    BYTE WDigit[6]; //enough for a  number < 65636: 5 digits + \0
    BYTE j;
    BYTE LCDPos=0;  //write on first line of LCD
    unsigned radix=10; //type expected by sdcc's ultoa()
    
    LCDPos=pos;
    ultoa(w, WDigit, radix);
    for(j = 0; j < strlen((char*)WDigit); j++)
    {
        LCDText[LCDPos++] = WDigit[j];
    }
    if(LCDPos < 32u)
        LCDText[LCDPos] = 0;
    LCDUpdate();
}
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
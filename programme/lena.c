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
 * DATE :           Oct. 21st 2013
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

// states
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

#define SNOOZE_MINUTE 5
#define SNOOZE_MAX 12

/* OPS est le nombre d'overflows du timer0 en 1 seconde, que nous avons mesuré
 * dans findfreq. Pour être plus précis, toutes les PER pseudosecondes (càd
 * après PER*OPS overflows), il faudra retirer CUT nombre d'overflows
 */
#define OPS 95
#define PER 5
#define CUT 2

void inc_tsec(BYTE val);
void inc_ahour(BYTE val);
void inc_amin(BYTE val);
void dec_ahour(BYTE val);
void dec_amin(BYTE val);
void manageseconds(void);

void time(void);
void button(void);
void refresh_lcd(void);

void DisplayString(BYTE pos, char* text);
void DisplayWORD(BYTE pos, WORD w);
size_t strlcpy(char *dst, const char *src, size_t siz);

BYTE chandelle = 1; // ##

/* l'heure actuelle */
BYTE thour = 0;
BYTE tmin = 0;
BYTE tsec = 0;

/* l'heure de l'alarme */
BYTE alarm_set = 0; // indique si l'alarme est mise
BYTE ahour = 0;
BYTE amin = 0;

/* l'heure originelle de l'alarme */
BYTE ahour_o = 0;
BYTE amin_o = 0;

/* l'état de l'alarme */
BYTE snooze = 0; // compteur du nombre de snooze
BYTE stop_ringing = 0; // indique si on a eteint le réveil

/* l'état du réveil */
BYTE whereami = 0;

/* seconde et demi-seconde */
BYTE new_time = 0; // compteur de secondes
BYTE halfsecond = 0; // flag demi-seconde

/* les boutons */
BYTE button1 = 0; // flag pour le boutton 1
BYTE button2 = 0; // flag pour le bouton 2

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
    // Button 1
    if(INTCON3bits.INT3F) {
        button1 = 1;
        INTCON3bits.INT3F = 0;   //clear INT1 flag
        
        // Button 2
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
    LATJbits.LATJ0 = 0; // switch LED 1 off
    LATJbits.LATJ1 = 0; // switch LED 2 off
    LATJbits.LATJ2 = 0; // switch LED 3 off
    
    /* buttons */
    BUTTON0_TRIS        = 1; //configure 1st button as input
    BUTTON1_TRIS        = 1; //configure 2nd button as input
    INTCON3bits.INT3E   = 1; //enable INT3 interrupt (button 1)
    INTCON3bits.INT3F   = 0; //clear INT3 flag
    INTCON3bits.INT1E   = 1; //enable INT1 interrupt (button 2)
    INTCON3bits.INT1F   = 0; //clear INT1 flag
    INTCON3bits.INT1IP  = 0; //low priority
    INTCON2bits.INT3IP  = 0; //low priority
    INTCON2bits.INTEDG1 = 1; //INT1 interrupts on falling edge ?????
    INTCON2bits.INTEDG3 = 1; //INT3 interrupts on falling edge ?????
    
    //////////////////////////////////////////////
    
    LCDInit();
    whereami = TIME_MENU;
    
    chandelle++; // ##### BIZARRE ####
    
    T0CONbits.TMR0ON = 1; // start timer0
    while (1) {
        time();
        manageseconds();
        button();
        refresh_lcd();
    }
}


/* Fonction qui incrémente les secondes de l'horloge */
void inc_tsec(BYTE val)
{
    BYTE mod_tsec;
    BYTE mod_tmin;
    
    mod_tsec = (tsec + val) / 60;
    if (mod_tsec) {
        
        mod_tmin = (tmin + mod_tsec) / 60;
        if (mod_tmin) {
            thour = (thour + mod_tmin) % 24;
        }
        
        tmin = (tmin + mod_tsec) % 60;
    }
    
    tsec = (tsec + val) % 60;
}

/* Fonction qui incrémente les heures du réveil */
void inc_ahour(BYTE val)
{
    ahour = (ahour + val) % 24;
}

/* Fonction qui incrémente les minutes du réveil */
void inc_amin(BYTE val)
{
    // vérifie s'il faut incrémenter les heures
    BYTE mod_amin;
    mod_amin = (amin + val) / 60;
    if (mod_amin) {
        inc_ahour(mod_amin);
    }
    
    amin = (amin + val) % 60;
}

/* Fonction qui décrémente les heures de l'alarme */
void dec_ahour(BYTE val)
{
    // vérifie s'il faut faire une boucle sur les heures
    if (val > ahour) {
        ahour = (24+ahour) - val;
    } else {
        ahour = ahour - val;
    }

}

/* Fonction qui décrémente les minutes de l'alarme */
void dec_amin(BYTE val)
{
    // vérifie s'il faut décrémenter les heures
    if (val > amin) {
        dec_ahour(1);
        amin = (60+amin) - val;
    } else {
        amin  = amin - val;
    }
}

/* Fonction qui met l'heure à jour et lance le réveil */
void time(void)
{
    // on fait d'office clignoter la LED jaune avec une période de 1s
    if (halfsecond) {
        LED0_IO ^= 1;
        halfsecond = 0;
    }
    
    // vérifie si min. 1 seconde est passée
    if (new_time) {
        inc_tsec(new_time);
        new_time = 0;
        
        LED0_IO ^= 1;
    }
    
    // vérifie si l'heure de réveil est atteinte et si l'alarme est mise
    if ((thour == ahour) && (tmin == amin) && alarm_set) {
        // vérifie que l'alarme n'est pas encore finie, qu'elle n'a pas encore
        // été arrêtée et qu'on ne se trouve pas dans le menu
        if ((tsec < 31) && (stop_ringing == 0)) {
            if ((whereami == DISPLAY) || (whereami == SNOOZE)) {
                whereami = ALARM;
            }
            if (whereami == ALARM) {
                LED1_IO ^= 1; //change state of red leds
                LED2_IO ^= 1; //change state of red leds
            }
        // éteint l'alarme au bout de 30 secondes 
        } else if (tsec > 30) {
            stop_ringing = 0; // remet à 0 si l'alarme a été éteinte à la main
            LATJbits.LATJ1 = 0; // switch LED 2 off
            LATJbits.LATJ2 = 0; // switch LED 3 off
            
            if (snooze) {
                ahour = ahour_o; // remet le réveil
                amin = amin_o;
                snooze = 0;
                whereami = DISPLAY;
            } else if (whereami == ALARM) { // si l'alarme sonnait toujours, on
                whereami = DISPLAY;         // revient à l'affichage de l'heure
                                            // normal
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

/* Fonction qui gère l'actionnement des boutons */
void button(void)
{    
    // gère le bouton MENU/NEXT/STOP
    if (button1) {
        switch (whereami) {
            case TIME_MENU: // NEXT
                whereami = ALARM_MENU;
                break;
            case SET_HOUR: // NEXT
                whereami = SET_MINUTE;
                break;
            case SET_MINUTE: // NEXT
                whereami = SET_SECOND;
                break;
            case SET_SECOND: // NEXT
                whereami = ALARM_MENU;
                break;
            case ALARM_MENU: // NEXT
                whereami = DISPLAY;
                break;
            case SET_ALARM: // NEXT
                whereami = SET_A_HOUR;
                break;
            case SET_A_HOUR: // NEXT
                whereami = SET_A_MIN;
                break;
            case SET_A_MIN: // NEXT
                whereami = DISPLAY;
                break;
            case DISPLAY: // MENU
                whereami = TIME_MENU;
                break;
            case ALARM: // STOP
                stop_ringing = 1;
                LATJbits.LATJ1 = 0; // switch LED 2 off
                LATJbits.LATJ2 = 0; // switch LED 3 off
                whereami = DISPLAY;
                break;
            case SNOOZE: // STOP // à vérifier ##
                amin = amin_o; // remet le réveil
                ahour = ahour_o;
                snooze = 0;
                stop_ringing = 0; // le réveil doit sonner à nouveau 
                LATJbits.LATJ1 = 0; // switch LED 2 off
                LATJbits.LATJ2 = 0; // switch LED 3 off
                whereami = DISPLAY;
                break;
            default:
                break;
        }
        button1 = 0; // remet le flag du boutton 1 à 0
    
    // gère le bouton SELECT/ADD/SNOOZE
    } else if (button2) {
        switch (whereami) {
            case TIME_MENU: // SELECT
                whereami = SET_HOUR;
                break;
            case SET_HOUR: // ADD
                thour++;
                if (thour == 24) {
                    thour = 0;
                }
                break;
            case SET_MINUTE: // ADD
                if (tmin == 59) {
                    tmin = 0;
                } else {
                    tmin++;
                }
                break;
            case SET_SECOND: // ADD
                if (tsec == 59) {
                    tsec = 0;
                } else {
                    tsec++;
                }
                break;
            case ALARM_MENU: // SELECT
                whereami = SET_ALARM;
                break;
            case SET_ALARM: // ADD
                alarm_set ^= 1;
                break;
            case SET_A_HOUR: // ADD
                if (ahour == 23) {
                    ahour = 0;
                } else {
                    ahour++;
                }
                ahour_o = ahour;
                break;
            case SET_A_MIN: // ADD
                if (amin == 59) {
                    amin = 0;
                } else {
                    amin++;
                }
                amin_o = amin;
                break;
            case DISPLAY: // ---
                // rien à faire
                break;
            case ALARM: // SNOOZE
                // snooze si la limite n'est pas encore atteinte...
                if (snooze < SNOOZE_MAX) {
                    inc_amin(SNOOZE_MINUTE); // modifie le réveil
                    snooze++; // augmente le compteur de snooze
                    LATJbits.LATJ1 = 0; // switch LED 2 off
                    LATJbits.LATJ2 = 0; // switch LED 3 off
                    whereami = SNOOZE;
                }
                // ...sinon, ne fait rien
                break;
            case SNOOZE: // SNOOZE
                // snooze si la limite n'est pas encore atteinte...
                if (snooze < SNOOZE_MAX) {
                    inc_amin(SNOOZE_MINUTE); // modifie le réveil
                    snooze++; // augmente le compteur de snooze
                }
                // ...sinon, ne fait rien
                break;
            default:
                break;
        }
        button2 = 0; // remet le flag du boutton 2 à 0
    }
}

void refresh_lcd(void)
{
    CHAR display[32]; // buffer pour l'affichage
    
    switch (whereami) {
        case TIME_MENU:
            sprintf(display, "Do you want to  set the time ?  ");
            break;
        case SET_HOUR:
            sprintf(display, " [%02u]: %02u : %02u                  ",
                    thour, tmin, tsec);
            break;
        case SET_MINUTE:
            sprintf(display, "  %02u :[%02u]: %02u                  ",
                    thour, tmin, tsec);
            break;
        case SET_SECOND:
            sprintf(display, "  %02u : %02u :[%02u]                 ",
                    thour, tmin, tsec);
            break;
        case ALARM_MENU:
            sprintf(display, "Do you want to  set the alarm ? ");
            break;
        case SET_ALARM:
            if (alarm_set) {
                sprintf(display, "  Alarm [ON ]                   ");
            } else {
                sprintf(display, "  Alarm [OFF]                   ");
            }
            break;
        case SET_A_HOUR:
            sprintf(display, "    Alarm at        [%02u]: %02u    ",
                    ahour, amin);
            break;
        case SET_A_MIN:
            sprintf(display, "    Alarm at         %02u :[%02u]   ",
                    ahour, amin);
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
        case SNOOZE:
            if (snooze < 10) {
                sprintf(display, "    %02u:%02u:%02u    Snooze %u  %02u:%02u ",
                        thour, tmin, tsec, snooze, ahour_o, amin_o);
            } else {
                sprintf(display, "    %02u:%02u:%02u    Snooze %u %02u:%02u ",
                        thour, tmin, tsec, snooze, ahour_o, amin_o);
            }
            
            break;
        default:
            sprintf(display, "**** ERROR ********* ERROR *****");
            break;
    }
    DisplayString(0, display);
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

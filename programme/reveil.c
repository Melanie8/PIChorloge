/******************************************************************************
 
 FICHIER        reveil.c
 ------------------------------------------------------------------------------
 AUTEURS        Lena Peschke et Mélanie Sedda, G43
 ------------------------------------------------------------------------------
 VERSION        26 octobre 2013
 ------------------------------------------------------------------------------
 OBJECTIF       Implémenter un réveil pour le pic
 ------------------------------------------------------------------------------
 STRATEGIE
 
 Notre réveil se base sur le timer0 pour mesurer le temps qui passe.
 Connaissant la fréquence de celui-ci, il compte le nombre d'overflows et
 vérifie si une seconde est passée pour mettre à jour l'heure de l'horloge.
 ++++ EXPLIQUER LE COMPTAGE DES TICKS ++++
 
 Il y a deux routines d'interruptions à des niveaux de priorités différents :
    - le niveau 1 génère une interruption dès que le timer0 overflow et ne
      fait que compter le nombre d'overflows qui se sont produits;
    - le niveau 2 génère une interruption lorsqu'on appuie sur un bouton et
      modifie le flag du bouton correspondant.
 
 La fonction main vérifie en boucle
    - si le temps mesuré a changé, càd s'il y a eu assez d'overflows pour
      ajouter une seconde;
    - s'il faut changer l'affichage à l'écran, ce qui arrive lorsque le temps
      a changé, lorsqu'on a appuyé sur un bouton et lorsque l'alarme se
      déclenche (càd lorsque l'état du réveil indiqué dans la variable
      'whereami' a changé);
    - s'il faut déclencher ou arrêter l'alarme;
    - s'il faut changer d'état ou la valeur d'une variabel suite à
      l'actionnement d'un bouton.
 
 *****************************************************************************/

#define __18F97J60
#define __SDCC__
#define THIS_INCLUDES_THE_MAIN_FUNCTION
#include "Include/HardwareProfile.h"

#include <string.h>
#include <stdlib.h>

#include "Include/LCDBlocking.h"
#include "Include/TCPIP_Stack/Delay.h"

/* états du réveil */
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

/* données */
#define SNOOZE_MINUTE 5 // délai entre 2 répétitions du réveil
#define SNOOZE_MAX 12 // nombre maximum de répétitions

#define F 95.365555556 // nombre d'overflows du timer0 en 1 seconde

/* déclarations des fonctions */
void time(void);
void refresh_lcd(void);
void alarm(void);
void button(void);

void inc_ahour(BYTE val);
void inc_amin(BYTE val);

void DisplayString(BYTE pos, char* text);
void DisplayWORD(BYTE pos, WORD w);
size_t strlcpy(char *dst, const char *src, size_t siz);

//BYTE chandelle;

/* les boutons */
BYTE button1 = 0; // flag pour le boutton 1
BYTE button2 = 0; // flag pour le bouton 2

/* variables pour le calcul de l'heure actuelle */
QWORD overflows = 0; // nombre d'overflows du timer0 depuis le début
QWORD sec; // nombre de secondes écoulées depuis le début
QWORD decisec; // nombre de dixièmes de seconde écoulés depuis le début
BYTE ds; // decisec%10, utile pour pouvoir éteindre les LEDS +- toutes
         // les demi-secondes
BYTE h;
BYTE m;
BYTE s;

/* la dernière heure affichée */
BYTE thour = 0;
BYTE tmin = 0;
BYTE tsec = 0;

/* l'heure de l'alarme (si on snooze, elle change) */
BYTE ahour = 0;
BYTE amin = 0;

/* l'heure originelle de l'alarme (si on snooze, elle ne change pas) */
BYTE ahour_o = 0;
BYTE amin_o = 0;

/* l'état de l'alarme */
BYTE alarm_set = 0; // indique si l'alarme est mise
BYTE snooze = 0; // compteur du nombre de snooze
BYTE stop_ringing = 0; // indique si on a eteint le réveil

/* l'état du réveil */
BYTE whereami = 0;

/* état de la led jaune */
BYTE on = 0;

/* buffer pour l'affichage sur le LCD */
CHAR display[32];


/* Interruption de priorité haute : liée au temps */
void high_isr (void) interrupt 1
{
    // incrémente juste overflows
    if (INTCONbits.T0IF) {
        overflows++;
        INTCONbits.T0IF = 0;
    }
    
}

/* Interruption de priorité basse : liée aux boutons */
void low_isr (void) interrupt 2
{
    // bouton 1
    if(INTCON3bits.INT3F) {
        button1 = 1;
        INTCON3bits.INT3F = 0;
        
    // bouton 2
    } else if(INTCON3bits.INT1F) {
        button2 = 1;
        INTCON3bits.INT1F = 0;
    }
}

/* Fonction main */
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
    
    LCDInit();
    whereami = TIME_MENU;
    
    //chandelle++; // ##### BIZARRE ####
    
    T0CONbits.TMR0ON = 1; // start timer0
    
    while (1) {
        time();
        refresh_lcd();
        alarm();
        button();
    }
}

/* Fonction qui met à jour l'heure et gère les LED */
void time(void)
{
    // calcul de la nouvelle heure
    sec = overflows/F;
    decisec = ((10*overflows)/F);
    ds = decisec%10;
    h = (sec/3600)%24;
    m = (sec-(sec/3600)*3600)/60;
    s = sec-h*3600-m*60;
    
    // si l'heure a changé par rapport à la dernière calculée, on la change
    if (tsec != s)
        tsec = s;
    if (tmin != m)
        tmin = m;
    if (thour != h)
        thour = h;
    
    // allumage des LED, dès qu'on peut dans la première demi-seconde
    if (!on && ds < 5) {
        LATJbits.LATJ0 = 1;
        if (whereami == ALARM) {
            LATJbits.LATJ1 = 1;
            LATJbits.LATJ2 = 1;
        }
        on = 1;
    }
    
    // extinction des LED, dès qu'on peut dans la deuxième demi-seconde
    if (on && ds >= 5) {
        LATJbits.LATJ0 = 0;
        if (whereami == ALARM) {
            LATJbits.LATJ1 = 0;
            LATJbits.LATJ2 = 0;
        }
        on = 0;
    }
}

/* Fonction qui gère l'affichage */
void refresh_lcd(void)
{
    // vérifie dans quel état se trouve le réveil
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
                    thour, tmin, tsec);
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


/* Fonction qui gère le réveil */
void alarm(void)
{
    // vérifie si l'heure de réveil est atteinte et si l'alarme est mise
    if ((thour == ahour) && (tmin == amin) && alarm_set) {
        // vérifie que l'alarme n'est pas encore finie, qu'elle n'a pas encore
        // été arrêtée et qu'on ne se trouve pas dans le menu
        if ((tsec < 31) && (stop_ringing == 0)) {
            if ((whereami == DISPLAY) || (whereami == SNOOZE)) {
                whereami = ALARM;
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

/* Fonction qui gère l'actionnement des boutons */
void button(void)
{
    
    BYTE inchour = thour;
    BYTE incmin = tmin;
    BYTE incsec = tsec;
    
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
            case SNOOZE: // STOP
                stop_ringing = 1; // le réveil ne doit plus sonner
                amin = amin_o; // remet le réveil à l'heure d'origine
                ahour = ahour_o;
                snooze = 0;
                LATJbits.LATJ1 = 0; // switch LED 2 off
                LATJbits.LATJ2 = 0; // switch LED 3 off
                whereami = DISPLAY;
                break;
            default:
                break;
        }
        button1 = 0; // remet le flag du bouton 1 à 0
    
    // gère le bouton SELECT/ADD/SNOOZE
    } else if (button2) {
        switch (whereami) {
            case TIME_MENU: // SELECT
                whereami = SET_HOUR;
                break;
            case SET_HOUR: // ADD
                if (thour==23)
                    overflows -= 59*F*3600;
                else
                    overflows += F*3600;
                break;
            case SET_MINUTE: // ADD
                if (tmin==59)
                    overflows -= 59*F*60;
                else
                    overflows += F*60;
                break;
            case SET_SECOND: // ADD
                if (tsec==59)
                    overflows -= 59*F;
                else
                    overflows += F;
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
        button2 = 0; // remet le flag du bouton 2 à 0
    }
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

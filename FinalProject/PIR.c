/*
 * PIR.c
 *
 *  Created on: Oct 26, 2017
 *      Author: Dan Bagby
 */

#include "driverlib.h"
#include "msp.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "PIR.h"

void alarmOK(void);
void alarmNOK(void);

void PIR(void)
{
    while(1)
    {
        if(P3->IN&0x01)
            alarmOK();
        else
            alarmNOK();
        sysDelay(100);
    }
}

void alarmOK(void)
{
    P2->OUT = 0x01;
  ST7735_FillScreen(RED);     //Red Screen
}

void alarmNOK(void)
{
    P2->OUT = 0x02;
    ST7735_FillScreen(GREEN);   //Green Screen
}


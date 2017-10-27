/*
 * EnterPasscode.c
 *
 *  Created on: Oct 21, 2017
 *      Author: Dan Bagby
 *      This file controls the passcode change and entry display and code entry
 */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "EnterPasscode.h"

uint8_t button_value;
int key[4] = {0},passcode[4] = {0}, noCode=1, successFlag = 1;

void getPasscode(int key[]){
    int enterFlag = 1;

    while(enterFlag)
    {
      button_value=keypad_getkey();
      if(button_value!=0)
      {
          if(button_value != 12 && button_value != 10){

              if(button_value == 11){
                  button_value = 0;
              }

              key[0] = key[1];
              key[1] = key[2];
              key[2] = key[3];
              key[3] = button_value;
          }


          if(button_value == 12){
              noCode=0;
              enterFlag = 0;
          }
      }
      sysDelay(100);
    }
}

int checkPasscode(int key[], int passcode[]){
    int j;
    int passFlag = 1;
    for(j = 0 ; j<4 ; j++){
        if(key[j] != passcode[j]){
            passFlag = 0;
        }
    }
    return passFlag;
}

void changePasscode(void)
{
    // uint8_t red, green, blue;
       Output_Clear();
       ST7735_DrawCharS(35, 49, 'E', ST7735_Color565(255, 255, 0), 0, 2);
       ST7735_DrawCharS(47, 49, 'n', ST7735_Color565(255, 255, 0), 0, 2);
       ST7735_DrawCharS(59, 49, 't', ST7735_Color565(255, 255, 0), 0, 2);
       ST7735_DrawCharS(71, 49, 'e', ST7735_Color565(255, 255, 0), 0, 2);
       ST7735_DrawCharS(83, 49, 'r', ST7735_Color565(255, 255, 0), 0, 2);
       ST7735_DrawCharS(47, 65, 'N', ST7735_Color565(255, 255, 0), 0, 2);
       ST7735_DrawCharS(59, 65, 'e', ST7735_Color565(255, 255, 0), 0, 2);
       ST7735_DrawCharS(71, 65, 'w', ST7735_Color565(255, 255, 0), 0, 2);
       ST7735_DrawCharS(41, 81, 'C', ST7735_Color565(255, 255, 0), 0, 2);
       ST7735_DrawCharS(53, 81, 'o', ST7735_Color565(255, 255, 0), 0, 2);
       ST7735_DrawCharS(65, 81, 'd', ST7735_Color565(255, 255, 0), 0, 2);
       ST7735_DrawCharS(77, 81, 'e', ST7735_Color565(255, 255, 0), 0, 2);
while(noCode){
getPasscode(passcode);
}
}
void enterPasscode(void){

    while(1){
        Output_Clear();
        ST7735_DrawCharS(35, 49, 'E', ST7735_Color565(255, 255, 0), 0, 2);
        ST7735_DrawCharS(47, 49, 'n', ST7735_Color565(255, 255, 0), 0, 2);
        ST7735_DrawCharS(59, 49, 't', ST7735_Color565(255, 255, 0), 0, 2);
        ST7735_DrawCharS(71, 49, 'e', ST7735_Color565(255, 255, 0), 0, 2);
        ST7735_DrawCharS(83, 49, 'r', ST7735_Color565(255, 255, 0), 0, 2);
        ST7735_DrawCharS(41, 65, 'C', ST7735_Color565(255, 255, 0), 0, 2);
        ST7735_DrawCharS(53, 65, 'o', ST7735_Color565(255, 255, 0), 0, 2);
        ST7735_DrawCharS(65, 65, 'd', ST7735_Color565(255, 255, 0), 0, 2);
        ST7735_DrawCharS(77, 65, 'e', ST7735_Color565(255, 255, 0), 0, 2);
    getPasscode(key);

    int correctFlag = checkPasscode(key,passcode);

    if(correctFlag){
        ST7735_FillScreen(0);
        ST7735_SetCursor(0, 0);
        printf("\nCorrect key entered!");
        /******************************
         *
         * Do something!!!!!!!!!!!!
         *
         ******************************/
        successFlag = 0;
    }

    if(!correctFlag){
        ST7735_FillScreen(0);
        ST7735_SetCursor(0, 0);
        printf("\nNot correct.\n Try again");
    }}
}

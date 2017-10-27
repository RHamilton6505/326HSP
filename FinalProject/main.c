
/* --COPYRIGHT--,BSD
 * Copyright (c) 2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/

#define TIMER32_1                        (TIMER32_BASE)


/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

/* Statics */
static volatile uint16_t curADCResult;
static volatile float normalizedADCRes = 3;
int flag = 1;
int alarmState = 2;
int buzzCount = 0;


int main(void)
{
    MAP_WDT_A_holdTimer();
    sysTimerInit();
    ADCInit();
    T32Init();
    mainInit();


    while(1){
    }
    
}

void mainInit(){
    //BUZZER 2.7
    P2->DIR |=  BIT0 | BIT1 | BIT2;


    //SCREEN 7.7
    P7->DIR |= BIT7 | BIT6;
    P7->SEL0 |= BIT7 | BIT6;
    P7->SEL1 &= ~BIT7 | BIT6;

}

void ADC14_IRQHandler(void)
{
    uint64_t status = MAP_ADC14_getEnabledInterruptStatus();
    MAP_ADC14_clearInterruptFlag(status);
    if (ADC_INT0 & status)
    {
        curADCResult = MAP_ADC14_getResult(ADC_MEM0);
        normalizedADCRes = (curADCResult * 3.3)/16384 ;
        flag = 1;
    }
}

void T32_INT1_IRQHandler(void){
    fflush(stdout);
    printf("T32 works\n");


    int duty = (int) ((normalizedADCRes-.80)*50);
    int dutyBuzz = 0;

    MAP_ADC14_toggleConversionTrigger();

    if(alarmState == 0){
        P2OUT &= ~(BIT0 | BIT2);
        P2OUT ^= BIT1;
    }

    if(alarmState == 1){

        P2OUT &= ~(BIT1 |BIT2);
        dutyBuzz = buzzCount*100;
        P2OUT ^= BIT(0);
        buzzCount++;
        if(buzzCount == 2) buzzCount = 0;


    }

    if(alarmState == 2){
        P2OUT &= ~(BIT1 |BIT2);
        P2OUT = BIT0;
        buzzCount++;
        dutyBuzz = buzzCount*25+50;
        if(buzzCount == 2) buzzCount = 0;
    }

    LED_PWM_SCREEN_BUZZER(duty,dutyBuzz);

    Timer32_clearInterruptFlag(TIMER32_0_BASE);
    Timer32_setCount(TIMER32_0_BASE,20000);
}


void LED_PWM_SCREEN_BUZZER(int dutyCycleScreen, int dutyCycleBuzzer){
    // Output is on P7.7
    TIMER_A1->CCR[0] = 100 - 1;
    TIMER_A1-> CCTL[1] = TIMER_A_CCTLN_OUTMOD_7;
    TIMER_A1->CCR[1] = dutyCycleScreen;
    TIMER_A1-> CCTL[2] = TIMER_A_CCTLN_OUTMOD_7;
    TIMER_A1->CCR[2] = dutyCycleBuzzer;

    TIMER_A1-> CTL = TIMER_A_CTL_SSEL__SMCLK | TIMER_A_CTL_MC__UP | TIMER_A_CTL_CLR;
}


void sysTimerInit(){
    SysTick -> CTRL = 0;            // disable SysTick during setup
    SysTick -> LOAD = 0x00FFFFFF;   // maximum reload value
    SysTick -> VAL = 0;             // any write to current value clears it
    SysTick -> CTRL = 0x00000005;   // enable SysTick, CPU clk, no interrupts
}

void sysDelay(uint16_t mSecs){
    SysTick -> LOAD = 3000*mSecs;
    SysTick -> VAL = 0;
    while((SysTick -> CTRL & 0x00010000) == 0);
}

void ADCInit(){
    curADCResult = 0;
    
    /* Setting Flash wait state */
    MAP_FlashCtl_setWaitState(FLASH_BANK0, 2);
    MAP_FlashCtl_setWaitState(FLASH_BANK1, 2);
    
    /* Enabling the FPU for floating point operation */
    MAP_FPU_enableModule();
    MAP_FPU_enableLazyStacking();
    
    //![Single Sample Mode Configure]
    /* Initializing ADC (MCLK/1/4) */
    MAP_ADC14_enableModule();
    MAP_ADC14_initModule(ADC_CLOCKSOURCE_MCLK, ADC_PREDIVIDER_1, ADC_DIVIDER_4,
                         0);
    
    /* Configuring GPIOs (5.5 A0) */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5, GPIO_PIN5,
                                                   GPIO_TERTIARY_MODULE_FUNCTION);
    
    /* Configuring ADC Memory */
    MAP_ADC14_configureSingleSampleMode(ADC_MEM0, true);
    MAP_ADC14_configureConversionMemory(ADC_MEM0, ADC_VREFPOS_AVCC_VREFNEG_VSS,
                                        ADC_INPUT_A0, false);
    
    /* Configuring Sample Timer */
    MAP_ADC14_enableSampleTimer(ADC_MANUAL_ITERATION);
    
    /* Enabling/Toggling Conversion */
    MAP_ADC14_enableConversion();
    MAP_ADC14_toggleConversionTrigger();
    //![Single Sample Mode Configure]
    
    /* Enabling interrupts */
    MAP_ADC14_enableInterrupt(ADC_INT0);
    MAP_Interrupt_enableInterrupt(INT_ADC14);
    MAP_Interrupt_enableMaster();
    
}

void T32Init(){
    MAP_Timer32_initModule(TIMER32_BASE, TIMER32_PRESCALER_256, TIMER32_32BIT,
            TIMER32_PERIODIC_MODE);
    MAP_Interrupt_enableInterrupt(INT_T32_INT1);
    MAP_Interrupt_enableMaster();
    MAP_Timer32_setCount(TIMER32_BASE,20000);
    MAP_Timer32_enableInterrupt(TIMER32_BASE);
    MAP_Timer32_startTimer(TIMER32_BASE, true);

}



void playPeriod(int T){
    P2->OUT = BIT7;                    // Blink P1.0 LED
    sysDelay(10);
    P2->OUT = ~BIT7;
    sysDelay(T-10);
}

/* note   frequency     period  timeHigh
* c            261 Hz          3830    1915
* d            294 Hz          3400    1700
* e            329 Hz          3038    1519
* f            349 Hz          2864    1432
* g            392 Hz          2550    1275
* a            440 Hz          2272    1136
* b            493 Hz          2028    1014
* C            523 Hz          1912    956
*
* http://www.arduino.cc/en/Tutorial/Melody
*/
void playNote(int note){

    if(note == 0) playPeriod(4540); //A
    if(note == 1) playPeriod(4290); //A#
    if(note == 2) playPeriod(4050); //B
    if(note == 3) playPeriod(3831); //C
    if(note == 4) playPeriod(3610); //C#
    if(note == 5) playPeriod(3400); //D
    if(note == 6) playPeriod(3210); //D#
    if(note == 7) playPeriod(3038); //E
    if(note == 8) playPeriod(2864); //F
    if(note == 9) playPeriod(2700); //F#
    if(note == 10) playPeriod(2550); //G
    if(note == 11) playPeriod(2410); //G#
}

void playBlank(int T){
    P2->OUT = ~BIT7;
    sysDelay(T);
}

void note(int note){
    int i = 0;
    for(i ; i<100+note ; i++)
        playNote(note);
    playBlank(7000000);

}

void quarterRest(){
    int i = 0;
    for(i ; i<100+note ; i++)
        playBlank(300);
}



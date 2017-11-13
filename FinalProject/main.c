
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
#include <Splash.h>
#include <ST7735.h>

#define COLUMN1 0x10
#define COLUMN2 0x20
#define COLUMN3 0x40
#define RED     0x001F
#define GREEN   0x07E0
#define BLACK   0x0000
#define BLUE    0xF800
#define CYAN    0xFFE0
#define MAGENTA 0xF81F
#define YELLOW  0x07FF
#define WHITE   0xFFFF
#define MENU_NUMBER 7
#define SLAVE_ADDRESS 0b1101000



/* Statics */
uint8_t keypad_getkey(void);
int debounce(void);
void release(void);
int pressed(int row, int col);
void sysTimerInit(void);
void sysDelay(uint16_t mSecs);
void sysDelaySec(uint8_t secs);
void Clock_Init48MHz(void);
void getPasscode(int key[]);
int checkPasscode(int key[], int passcode[]);
void changePasscode(void);
int enterPasscode(void);
int detect(void);
void alarmOK(void);
void alarmNOK(void);
int menuDisplay(void);
int menu(void);
void mainScreen(int status);
void setUpI2C(void);
void RTCMultibyteWrite(void);
void RTCMultibyteRead(void);
void setTime(void);
int getRTCButton(void);
void armed(void);
void disarm(void);
void getTime(void);
void armSystem(void);
void bcdToDecTime(void);
int armStat = 0, selection;
uint8_t code;
int key[4] = {0},passcode[4] = {1,1,1,1}, noCode=1, successFlag = 1;
int keyVal, selected = 0;
int scrColor=0, button_value=0, passcodeEntry=0, alarmStatus, detected=0;
uint8_t bcdSecond = 0, bcdMinute = 0, bcdHour = 0, bcdDay = 0, bcdDate = 0, bcdMonth = 0, bcdYear = 0, P4IntFlag=0, PoundFlag=0, RTC_registers[7];
int armCode = 0, alarmFlag = 0, disarmFlag = 0;
timeFlag = 1;

const eUSCI_I2C_MasterConfig i2cConfig =
{
 EUSCI_B_I2C_CLOCKSOURCE_SMCLK,
 48000000,
 EUSCI_B_I2C_SET_DATA_RATE_400KBPS,
 0,
 EUSCI_B_I2C_NO_AUTO_STOP
};

static volatile uint16_t curADCResult;
static volatile float normalizedADCRes = 0;
int flag = 1;
int alarmState;
int buzzCount = 0;
int RTCFlag = 0;


int main(void)
{

    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P3, GPIO_PIN0);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2);
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4, GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2 + GPIO_PIN3);
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P5, GPIO_PIN0 + GPIO_PIN1);
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6, GPIO_PIN4 + GPIO_PIN5, GPIO_PRIMARY_MODULE_FUNCTION);
    P2->OUT &= ~0xFF;

    Clock_Init48MHz();
    ST7735_InitR(INITR_REDTAB);
    MAP_WDT_A_holdTimer();
    sysTimerInit();
    splash();
    setUpI2C();




    ADCInit();
    T32Init();
    mainInit();

    alarmState = 0;






    while(1)
    {
        mainScreen(armStat);
        selection=menuDisplay();
        switch(selection)
        {
        case 1:
            armSystem();
            break;
            /*case 2:
            unlockDoor();
            break;*/
        case 3:
            setTime();
            break;
        case 4:
            changePasscode();
            break;
            /*case 5:
            alarmHistory();
            break;
        case 6:
            armHistory();
            break;*/
        default:
            printf("Invalid Selection");
        }
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

    timeFlag = 1;

    //    getTime();

    //    ST7735_SetCursor(0,0);
    //
    //    printf("test");

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
    Timer32_setCount(TIMER32_0_BASE,187500);
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
    MAP_Timer32_setCount(TIMER32_BASE,187500);
    MAP_Timer32_enableInterrupt(TIMER32_BASE);
    MAP_Timer32_startTimer(TIMER32_BASE, true);

}

uint8_t keypad_getkey(void)
{
    /* assumes port 4 bits 0-3 are connected to rows
     *  bits 4,5,6 are connected to columns*/

    int row = 0x0F, col, button;
    const char column_select[] = {COLUMN3, COLUMN2, COLUMN1};                // one column is active
    // Activates one column at a time, read the input to see which column
    for (col = 0; col < 3; col++)
    {
        P4->DIR &= ~0xF0; // disable all columns
        P4->DIR |= column_select[col]; // enable one column at a time
        P4->OUT &= ~column_select[col]; // drive the active column low
        sysDelay(10); // wait for signal to settle
        row = debounce();
        if (row != 0x0F)
        {
            release();
            break; // if one of the input is low,
        }
        P4->OUT |= column_select[col]; // drive the active column high
    }
    P4->OUT |= 0xF0; // drive all columns high before disable
    P4->DIR &= ~0xF0; // disable all columns

    if (col == 3) return 0; // if we get here, no key is pressed

    // gets here when one of the columns has key pressed,
    // check which row it is
    button=pressed(row, col);
    return button;
}

int debounce()
{
    int i, j=0xFF;
    for(i=0;i<10;i++)
    {
        j = j & P4->IN & 0x0F; // read all rows
        sysDelay(5);
    }
    return j;
}

void release()
{
    int button_pressed=1;
    while(button_pressed)
    {
        int button = 0xFF;
        button = button & P4->IN & 0x0F; // read all rows
        if(button ==0x0F)
        {
            ST7735_SetTextColor(WHITE);
            printf("*");
            button_pressed = 0;
        }
        else sysDelay(30);
    }
}

int pressed(int row, int col)
{
    if ((row != 0x07)&(row != 0x0B)&(row != 0x0D)&(row != 0x0E))
    {
        printf("\nERROR: Multiple row buttons pressed\n");
        return 0;
    }
    if (row == 0x07) return col + 1; // key in row 0
    if (row == 0x0B) return 3 + col + 1; // key in row 1
    if (row == 0x0D) return 6 + col + 1; // key in row 2
    if (row == 0x0E) return 9 + col + 1; // key in row 3

}

void sysTimerInit(void)
{
    SysTick -> CTRL = 0;            // disable SysTick during setup
    SysTick -> LOAD = 0x00FFFFFF;   // maximum reload value
    SysTick -> VAL = 0;             // any write to current value clears it
    SysTick -> CTRL = 0x00000005;   // enable SysTick, CPU clk, no interrupts
}

void sysDelay(uint16_t mSecs)
{
    SysTick -> LOAD = 48000*mSecs;  // with the master clock at 48MHz, the sysTick will overflow at a value of 350 or more
    SysTick -> VAL = 0;
    while((SysTick -> CTRL & 0x00010000) == 0);
}

void sysDelaySec(uint8_t secs)
{
    int i;
    for (i=0;i<(3*secs);i++)
    {
        sysDelay(333);
    }
}
void Clock_Init48MHz(void)
{
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_PJ, GPIO_PIN3 | GPIO_PIN2, GPIO_PRIMARY_MODULE_FUNCTION);
    CS_setExternalClockSourceFrequency(32000, 48000000);
    MAP_PCM_setCoreVoltageLevel(PCM_VCORE1);
    MAP_FlashCtl_setWaitState(FLASH_BANK0, 2);
    MAP_FlashCtl_setWaitState(FLASH_BANK1, 2);
    CS_startHFXT(false);
    MAP_CS_initClockSignal(CS_MCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_1);
}

void getPasscode(int key[]){
    int enterFlag = 1;

    while(enterFlag)
    {
        code=keypad_getkey();
        if(code!=0)
        {
            if(code != 12 && code != 10){

                if(code == 11){
                    code = 0;
                }
                key[0] = key[1];
                key[1] = key[2];
                key[2] = key[3];
                key[3] = code;
            }
            if(code == 12){
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
    //ST7735_FillScreen(0);
    ST7735_DrawCharS(35, 49, 'E', WHITE, 0, 2);
    ST7735_DrawCharS(47, 49, 'n', WHITE, 0, 2);
    ST7735_DrawCharS(59, 49, 't', WHITE, 0, 2);
    ST7735_DrawCharS(71, 49, 'e', WHITE, 0, 2);
    ST7735_DrawCharS(83, 49, 'r', WHITE, 0, 2);
    ST7735_DrawCharS(47, 65, 'N', WHITE, 0, 2);
    ST7735_DrawCharS(59, 65, 'e', WHITE, 0, 2);
    ST7735_DrawCharS(71, 65, 'w', WHITE, 0, 2);
    ST7735_DrawCharS(41, 81, 'C', WHITE, 0, 2);
    ST7735_DrawCharS(53, 81, 'o', WHITE, 0, 2);
    ST7735_DrawCharS(65, 81, 'd', WHITE, 0, 2);
    ST7735_DrawCharS(77, 81, 'e', WHITE, 0, 2);
    getPasscode(passcode);
}
int enterPasscode(void){

    while(1)
    {
        Output_Clear();
        ST7735_DrawCharS(35, 49, 'E', WHITE, 0, 2);
        ST7735_DrawCharS(47, 49, 'n', WHITE, 0, 2);
        ST7735_DrawCharS(59, 49, 't', WHITE, 0, 2);
        ST7735_DrawCharS(71, 49, 'e', WHITE, 0, 2);
        ST7735_DrawCharS(83, 49, 'r', WHITE, 0, 2);
        ST7735_DrawCharS(41, 65, 'C', WHITE, 0, 2);
        ST7735_DrawCharS(53, 65, 'o', WHITE, 0, 2);
        ST7735_DrawCharS(65, 65, 'd', WHITE, 0, 2);
        ST7735_DrawCharS(77, 65, 'e', WHITE, 0, 2);
        ST7735_SetCursor(0,0);
        getPasscode(key);

        int correctFlag = checkPasscode(key,passcode);

        if(correctFlag){
            ST7735_FillScreen(0);
            ST7735_SetCursor(0, 0);
            sysDelay(300);
            return 1;
        }

        if(!correctFlag){
            ST7735_FillScreen(0);
            ST7735_SetTextColor(WHITE);
            ST7735_SetCursor(0, 0);
            printf("\nNot correct.\n Try again");
            sysDelay(350);
        }
    }
}

int menuDisplay(void)
{
    char *s ="1 Arm System";
    char *s2 ="2 Unlock Door";
    char *s3 ="3 Set Time/ Date";
    char *s4 ="4 Change Passcode";
    char *s5 ="5 Alarm History";
    char *s6 ="6 Arm/ Disarm History";
    Output_Clear();
    ST7735_FillScreen(WHITE);
    unlock();
    if(timeFlag == 1){
        getTime();
        timeFlag = 0;
    }

    ST7735_DrawString(0, 1, s, BLACK);
    ST7735_DrawString(0, 2, s2, BLACK);
    ST7735_DrawString(0, 3, s3, BLACK);
    ST7735_DrawString(0, 4, s4, BLACK);
    ST7735_DrawString(0, 5, s5, BLACK);
    ST7735_DrawString(0, 6, s6, BLACK);
    selected=menu();
    return selected;
}

int menu(void)
{
    while(1)
    {
        getTime();
        keyVal=keypad_getkey();
        if(keyVal<=MENU_NUMBER&&keyVal>0) return keyVal;
    }
}

void mainScreen(int status)
{
    Output_Clear();
    ST7735_FillScreen(0);
    getTime();
    unlock();
    ST7735_FillRect(27,40,74,80,GREEN);
    ST7735_DrawCharS(35, 49, 'A', BLACK, GREEN, 2);
    ST7735_DrawCharS(47, 49, 'l', BLACK, GREEN, 2);
    ST7735_DrawCharS(59, 49, 'a', BLACK, GREEN, 2);
    ST7735_DrawCharS(71, 49, 'r', BLACK, GREEN, 2);
    ST7735_DrawCharS(83, 49, 'm', BLACK, GREEN, 2);
    ST7735_DrawCharS(29, 65, 'S', BLACK, GREEN, 2);
    ST7735_DrawCharS(41, 65, 'y', BLACK, GREEN, 2);
    ST7735_DrawCharS(53, 65, 's', BLACK, GREEN, 2);
    ST7735_DrawCharS(65, 65, 't', BLACK, GREEN, 2);
    ST7735_DrawCharS(77, 65, 'e', BLACK, GREEN, 2);
    ST7735_DrawCharS(89, 65, 'm', BLACK, GREEN, 2);

    ST7735_SetCursor(1,0);
    char *s ="Press any key";
    ST7735_DrawString(0, 1, s, WHITE);
    while(!button_value)
    {
        button_value=keypad_getkey();
        if(timeFlag == 1){
            getTime();
            timeFlag = 0;
        }
    }
    button_value=0;
}

void setUpI2C(void)
{
    // Initializing I2C Master (see description in Driver Lib for
    MAP_I2C_initMaster(EUSCI_B1_BASE, &i2cConfig); // proper configuration options)
    MAP_I2C_setSlaveAddress(EUSCI_B1_BASE, SLAVE_ADDRESS);// Specify slave address
    MAP_I2C_setMode(EUSCI_B1_BASE, EUSCI_B_I2C_TRANSMIT_MODE);// Set Master in transmit mode
    MAP_I2C_enableModule(EUSCI_B1_BASE);// Enable I2C Module to start operations
}

void RTCMultibyteWrite(void)
{
    MAP_I2C_setMode(EUSCI_B1_BASE, EUSCI_B_I2C_TRANSMIT_MODE);
    while (MAP_I2C_isBusBusy(EUSCI_B1_BASE));// Wait for bus release, ready to write
    MAP_I2C_masterSendMultiByteStart(EUSCI_B1_BASE,0);// set pointer to beginning of RTC registers
    MAP_I2C_masterSendMultiByteNext(EUSCI_B1_BASE,bcdSecond);// and write to seconds register
    MAP_I2C_masterSendMultiByteNext(EUSCI_B1_BASE, bcdMinute);// write to minutes register
    MAP_I2C_masterSendMultiByteNext(EUSCI_B1_BASE, bcdHour);// write to hours register
    MAP_I2C_masterSendMultiByteNext(EUSCI_B1_BASE, bcdDay);// write to day register
    MAP_I2C_masterSendMultiByteNext(EUSCI_B1_BASE, bcdDate); // write to date register
    MAP_I2C_masterSendMultiByteNext(EUSCI_B1_BASE, bcdMonth);// write to months register
    MAP_I2C_masterSendMultiByteFinish(EUSCI_B1_BASE, bcdYear);// write to year register and send stop
}

void RTCMultibyteRead(void)
{
    MAP_I2C_setMode(EUSCI_B1_BASE, EUSCI_B_I2C_TRANSMIT_MODE);// Set Master in transmit mode
    while (MAP_I2C_isBusBusy(EUSCI_B1_BASE));// Wait for bus release, ready to write
    MAP_I2C_masterSendSingleByte(EUSCI_B1_BASE,0);// set pointer to beginning of RTC registers
    while (MAP_I2C_isBusBusy(EUSCI_B1_BASE));// Wait for bus release
    MAP_I2C_setMode(EUSCI_B1_BASE, EUSCI_B_I2C_RECEIVE_MODE);// Set Master in receive mode
    while (MAP_I2C_isBusBusy(EUSCI_B1_BASE)); // Wait for bus release, ready to receive
    // read from RTC registers (pointer auto increments after each read)
    RTC_registers[0]=MAP_I2C_masterReceiveSingleByte(EUSCI_B1_BASE);
    RTC_registers[1]=MAP_I2C_masterReceiveSingleByte(EUSCI_B1_BASE);
    RTC_registers[2]=MAP_I2C_masterReceiveSingleByte(EUSCI_B1_BASE);
    RTC_registers[3]=MAP_I2C_masterReceiveSingleByte(EUSCI_B1_BASE);
    RTC_registers[4]=MAP_I2C_masterReceiveSingleByte(EUSCI_B1_BASE);
    RTC_registers[5]=MAP_I2C_masterReceiveSingleByte(EUSCI_B1_BASE);
    RTC_registers[6]=MAP_I2C_masterReceiveSingleByte(EUSCI_B1_BASE);
}

/**********************************
 * TimerFunctions
 **********************************/
void getTime()
{
    RTCMultibyteRead();
    ST7735_SetCursor(0,0);
    bcdToDecTime();
    ST7735_OutUDec(bcdHour);
    printf(":");
    ST7735_OutUDec(bcdMinute);
    printf(":");
    ST7735_OutUDec(bcdSecond);
    printf("  ");
    ST7735_OutUDec(bcdMonth);
    printf("/");
    ST7735_OutUDec(bcdDate);
    printf("/");
    ST7735_OutUDec(bcdYear);
}

void setTime()
{
    bcdSecond = 0; bcdMinute = 0; bcdHour = 0; bcdDay = 0; bcdDate = 0; bcdMonth = 0; bcdYear = 0;

    fflush(stdout);

    Output_Clear();
    ST7735_SetCursor(0,0);

    while(!bcdHour)
    {
        printf("\nEnter the hour \nfollowed by #. \n(1-12)\n");
        bcdHour|=getRTCButton();
    }

    Output_Clear();
    ST7735_SetCursor(0,0);


    while(!bcdMinute)
    {
        printf("\nEnter the minute \nfollowed by #. \n(1-60)\n");
        bcdMinute|=getRTCButton();
    }

    Output_Clear();
    ST7735_SetCursor(0,0);


    while(!bcdSecond)
    {
        printf("Enter the second \nfollowed by #. \n(1-60)\n");
        bcdSecond=0x40|getRTCButton();
    }

    Output_Clear();
    ST7735_SetCursor(0,0);

    while(!bcdDay)
    {
        printf("Enter the day \nfollowed by #. \n(1-7)\n");
        printf("1=Mon, 2=Tues, 3=Wed ...\n");
        bcdDay=getRTCButton();
    }

    Output_Clear();
    ST7735_SetCursor(0,0);

    while(!bcdMonth)
    {
        printf("Enter the month \nfollowed by #. \n(1-12)\n");
        bcdMonth=getRTCButton();
    }

    Output_Clear();
    ST7735_SetCursor(0,0);


    while(!bcdDate)
    {
        printf("Enter the date \nfollowed by #. \n(1-31)\n");
        bcdDate=getRTCButton();
    }

    Output_Clear();
    ST7735_SetCursor(0,0);


    while(!bcdYear)
    {
        printf("Enter the year \nfollowed by #. \n(20xx)\n");
        bcdYear=getRTCButton();
    }
    RTCMultibyteWrite();
}

int getRTCButton()
{
    uint8_t bcdVal = 0;
    int button=0;
    do
    {
        button=0;
        button=keypad_getkey();
        if(button!=0&&button!=10&&button!=12)
        {
            if(button==11) button=0;
            bcdVal=bcdVal<<4;
            bcdVal|=button;
        }
    }
    while(button!=12);
    return bcdVal;
}

void bcdToDecTime(void)
{
    bcdSecond=((0x70 & RTC_registers[0])>>4) * 10 + (0x0F & RTC_registers[0]);
    bcdMinute=((0x70 & RTC_registers[1])>>4) * 10 + (0x0F & RTC_registers[1]);
    bcdHour=((0x10 & RTC_registers[2])>>4) * 10 + (0x0F & RTC_registers[2]);
    bcdDay=(0x0F & RTC_registers[3]);
    bcdDate=((0x30 & RTC_registers[4])>>4) * 10 + (0x0F & RTC_registers[4]);
    bcdMonth=((0x10 & RTC_registers[5])>>4) * 10 + (0x0F & RTC_registers[5]);
    bcdYear=((0xF0 & RTC_registers[6])>>4) * 10 + (0x0F & RTC_registers[6]);
}

void armSystem()
{

    while(!armCode)
    {
        armCode=enterPasscode();
    }
    alarmState = 1;
    armed();
}

void armed(void)
{
    Output_Clear();

    ST7735_FillScreen(RED);
    lock();
    if(timeFlag == 1){
        getTime();
        timeFlag = 0;
    }
    ST7735_DrawCharS(29, 49, 'S', BLACK, RED, 2);
    ST7735_DrawCharS(41, 49, 'y', BLACK, RED, 2);
    ST7735_DrawCharS(53, 49, 's', BLACK, RED, 2);
    ST7735_DrawCharS(65, 49, 't', BLACK, RED, 2);
    ST7735_DrawCharS(77, 49, 'e', BLACK, RED, 2);
    ST7735_DrawCharS(89, 49, 'm', BLACK, RED, 2);
    ST7735_DrawCharS(35, 65, 'A', BLACK, RED, 2);
    ST7735_DrawCharS(47, 65, 'r', BLACK, RED, 2);
    ST7735_DrawCharS(59, 65, 'm', BLACK, RED, 2);
    ST7735_DrawCharS(71, 65, 'e', BLACK, RED, 2);
    ST7735_DrawCharS(83, 65, 'd', BLACK, RED, 2);
    while(P3->IN&0x01 && P5->IN&0x01 && P5->IN&0x02){
        getTime();
    }
    alarm();
}

void alarm(void)
{
    int disarmFlag = 0;

    alarmState = 2;

    Output_Clear();
    ST7735_FillScreen(0);
    if(timeFlag == 1){
        getTime();
        timeFlag = 0;
    }
    lock();
    ST7735_DrawCharS(35, 33, 'E', YELLOW, RED, 2);
    ST7735_DrawCharS(47, 33, 'n', YELLOW, RED, 2);
    ST7735_DrawCharS(59, 33, 't', YELLOW, RED, 2);
    ST7735_DrawCharS(71, 33, 'e', YELLOW, RED, 2);
    ST7735_DrawCharS(83, 33, 'r', YELLOW, RED, 2);
    ST7735_DrawCharS(29, 49, 'D', YELLOW, RED, 2);
    ST7735_DrawCharS(41, 49, 'i', YELLOW, RED, 2);
    ST7735_DrawCharS(53, 49, 's', YELLOW, RED, 2);
    ST7735_DrawCharS(65, 49, 'a', YELLOW, RED, 2);
    ST7735_DrawCharS(77, 49, 'r', YELLOW, RED, 2);
    ST7735_DrawCharS(89, 49, 'm', YELLOW, RED, 2);
    ST7735_DrawCharS(41, 65, 'C', YELLOW, RED, 2);
    ST7735_DrawCharS(53, 65, 'o', YELLOW, RED, 2);
    ST7735_DrawCharS(65, 65, 'd', YELLOW, RED, 2);
    ST7735_DrawCharS(77, 65, 'e', YELLOW, RED, 2);
    while(!disarmFlag)
    {
        disarmFlag=enterPasscode();
    }
    disarmFlag = 0;
    alarmState = 0;
}






#include <stdint.h>
#include <stdbool.h>
#include "15348.h"
#include "timer.h"
#include <math.h>
#include "serial.h"

// LCD commands

#define clear_display     0x01
#define returnHome        0x02
#define moveCursorRight   0x06
#define moveCursorLeft    0x08
#define shiftDisplayRight 0x1C
#define shiftDisplayLeft  0x18
#define cursorBlink       0x0F
#define cursorOff         0x0C
#define cursorOn          0x0E
#define Function_set_4bit 0x28
#define Function_set_8bit 0x38
#define Entry_mode        0x06
#define Function_8_bit    0x32
#define Set5x7FontSize    0x20
#define FirstRow          0x80


void PLLInit()
{
    SYSCTL_RCC2_R |= 0x80000000;
    SYSCTL_RCC2_R |= 0x00000800;
    SYSCTL_RCC_R = (SYSCTL_RCC_R & ~0x000007C0) + 0x00000540;
    SYSCTL_RCC2_R &= ~0x00000070;
    SYSCTL_RCC2_R &= ~0x00002000;
    SYSCTL_RCC2_R |= 0x40000000;
    SYSCTL_RCC2_R = (SYSCTL_RCC2_R & ~0x1FC00000) + (4 << 22);
    while ((SYSCTL_RIS_R & 0x00000040) == 0)
    {
    };
    SYSCTL_RCC2_R &= ~0x00000800;
}

void PortB_Init(void)
{
    volatile unsigned long delay;
    SYSCTL_RCGC2_R |= 0x00000002;     // 1) B clock
    delay = SYSCTL_RCGC2_R; // reading register adds a delay, which we might need
    GPIO_PORTB_LOCK_R = 0x4C4F434B;   // 2) unlock PortB
    GPIO_PORTB_CR_R = 0xFF;           // 3) allow changes to PB
    GPIO_PORTB_AMSEL_R = 0x00;        // 4) disable analog function
    GPIO_PORTB_AFSEL_R = 0x00;        // 5) no alternate function
    GPIO_PORTB_PCTL_R = 0x00000000;   // 6) GPIO clear bit PCTL
    GPIO_PORTB_PUR_R = 0x00;          // 7) disable pull up resistors
    GPIO_PORTB_DEN_R = 0xFF;          // 8) enable digital pins PB
    GPIO_PORTB_DIR_R = 0xFF;          // 9) PB output

    GPIO_PORTB_DATA_R = 0x0;
}

// LCD Helper Functions

// https://microcontrollerslab.com/16x2-lcd-interfacing-with-tm4c123-tiva-launchpad-keil-uvision/

/* Micro seconds delay function */
void delay_us(int n)
{
    int i,j;
    for(i=0;i<n;i++)
        for(j=0;j<3;j++) {}
}

void writeNibble(unsigned char data, unsigned char control){

    // take four upper bytes of data
    data &= 0xF0;

    // write data and set control (RS - cmd or data)
    GPIO_PORTB_DATA_R = data | control;

    // enable pins
    GPIO_PORTB_DATA_R = data | control | 0x04;

    delay_us(0);

    // disable pins - pulsed enable
    GPIO_PORTB_DATA_R = data | control;

    // clear data
    GPIO_PORTB_DATA_R = 0;

}

void sendCommand(unsigned char command){
    // send upper and lower 4 bits one after the other
    writeNibble(command & 0xF0, 0x0);
    writeNibble(command << 4, 0x0);

    if (command < 4){
        SysTick_Wait1ms(2);
    }
    else {
        delay_us(40);
    }
}

void writeChar(unsigned char data){
    writeNibble(data & 0xF0, 0x01);
    writeNibble(data << 4, 0x01);

    delay_us(40);

}

void writeString(char *str)
{
    int i;
    for(i=0; str[i]!=0; i++)
    {
        delay_us(40);
        writeChar(str[i]);

    }
}

void LCDInit(void)
{
    sendCommand(Set5x7FontSize);
    sendCommand(Function_set_4bit);
    sendCommand(moveCursorRight);
    sendCommand(clear_display);
    sendCommand(cursorBlink);
}

void SysTickInterruptInit()
{
    NVIC_ST_CTRL_R = 0;
    NVIC_ST_RELOAD_R = 800000; // exactly 10ms
    NVIC_ST_CURRENT_R = 0;
    NVIC_ST_CTRL_R = 0x00000007;
}

// ADC initialization

volatile unsigned long delay;
void ADC_Init(void){

    // PORT E initalization

    SYSCTL_RCGCGPIO_R |= 0x10; // activate clock for PORT E
    GPIO_PORTE_DIR_R &= ~0x3F; // make PE5-0 input
    GPIO_PORTE_AFSEL_R |= 0x3F; // enable alternate fun on PE5-0
    GPIO_PORTE_DEN_R &= ~0x3F; // disable digital I/O on PE5-0
    GPIO_PORTE_AMSEL_R |= 0x3F; // enable analog fun on PE5-0

    // configuring timer

    SYSCTL_RCGCTIMER_R |= 0x01; // activate timer0
    TIMER0_CTL_R = 0x0; // disable timer during setup
    TIMER0_CTL_R |= 0x20; // enable timer0A trigger to ADC
    TIMER0_CFG_R = 0; // 32-bit timer mode
    TIMER0_TAMR_R = 0x02; // configure for periodic mode
    TIMER0_TAPR_R = 0; // pre-scale value
    TIMER0_TAILR_R = 79999; // 8000 cycles in 1ms
    TIMER0_IMR_R = 0x0; // disable all interrupts
    TIMER0_CTL_R |= 0x01; // enable timer0A

    // configuring ADC0

    SYSCTL_RCGCADC_R |= 0x01; // activate ADC0

    delay = SYSCTL_RCGCADC_R;
    delay = SYSCTL_RCGCADC_R;
    delay = SYSCTL_RCGCADC_R;
    delay = SYSCTL_RCGCADC_R;

    ADC0_PC_R = 0x01; // sample rate 125k
    ADC0_SSPRI_R = 0x3210;

    // configuring sequencers

    ADC0_ACTSS_R &= ~0x01; // disable SS0
    ADC0_ACTSS_R &= ~0x02; // disable SS1
    ADC0_ACTSS_R &= ~0x04; // disable SS2

    ADC0_EMUX_R = (ADC0_EMUX_R&0xFFFFF000) | 0x555; // timer triggers for all sequencers

    // set channels and flags

    ADC0_SSMUX0_R = 0x23; // channels
    ADC0_SSCTL0_R = 0x60; // set flag

    ADC0_SSMUX1_R = 0x01; // channels
    ADC0_SSCTL1_R = 0x60; // set flag

    ADC0_SSMUX2_R = 0x8; // channels
    ADC0_SSCTL2_R = 0x6; // set flag

    ADC0_IM_R |= 0x01; // enable SS0 interrupts
    ADC0_IM_R |= 0x02; // enable SS1 interrupts
    ADC0_IM_R |= 0x04; // enable SS2 interrupts

    ADC0_ACTSS_R |= 0x01; // enable SS0
    ADC0_ACTSS_R |= 0x02; // enable SS1
    ADC0_ACTSS_R |= 0x04; // enable SS2

    // enabling interrupts

    // for SS0 and SS1

    NVIC_PRI3_R = (NVIC_PRI3_R&0xFF00FFFF) | 0x40200000; // ADC0 priority 1 (SS0) and 2 (SS1)
    NVIC_EN0_R = 0x0;
    NVIC_EN0_R |= 1 << 14; // SS0
    NVIC_EN0_R |= 1 << 15; // SS1

    // for SS2

    NVIC_PRI4_R = (NVIC_PRI4_R&0xFFFFFF00) | 0x00000060; // ADC0 priority 3 (SS2)
    NVIC_EN0_R |= 1 << 16;


}

uint32_t sensor1 = 0;
uint32_t sensor2 = 0;
uint32_t sensor3 = 0;
uint32_t sensor4 = 0;
uint32_t sensor5 = 0;

uint32_t sensor1Arr[20];
uint32_t sensor2Arr[20];
uint32_t sensor3Arr[20];
uint32_t sensor4Arr[20];
uint32_t sensor5Arr[20];

int sensor1Idx = 0;
int sensor2Idx = 0;
int sensor3Idx = 0;
int sensor4Idx = 0;
int sensor5Idx = 0;

uint64_t fifoStatusPRE = 0;
uint64_t fifoStatusPOST = 0;

void ADC0Seq0_Handler (void){
    ADC0_ISC_R |= 0x01; // ack interrupt

    // read samples
    sensor1 = ADC0_SSFIFO0_R&0xFFF;
    sensor2 = ADC0_SSFIFO0_R&0xFFF;

    // add to running avg list

    sensor1Arr[sensor1Idx] = sensor1;
    sensor2Arr[sensor2Idx] = sensor2;

    sensor1Idx++;
    sensor2Idx++;

    if (sensor1Idx == 20) sensor1Idx = 0;
    if (sensor2Idx == 20) sensor2Idx = 0;
}

void ADC0Seq1_Handler (void){
    ADC0_ISC_R |= 0x02; // ack interrupt

    // read samples
    sensor3 = ADC0_SSFIFO1_R&0xFFF;
    sensor4 = ADC0_SSFIFO1_R&0xFFF; // 4 SENSORS NOT WORKING

    // add to running avg list

    sensor3Arr[sensor3Idx] = sensor3;
    sensor2Arr[sensor4Idx] = sensor4;

    sensor3Idx++;
    sensor4Idx++;

    if (sensor3Idx == 20) sensor3Idx = 0;
    if (sensor4Idx == 20) sensor4Idx = 0;
}

void ADC0Seq2_Handler (void){
    ADC0_ISC_R |= 0x04; // ack interrupt

    // read samples
    sensor5 = ADC0_SSFIFO2_R&0xFFF;

    // add to running avg list

    sensor5Arr[sensor5Idx] = sensor5;

    sensor5Idx++;

    if (sensor5Idx == 20) sensor5Idx = 0;


}

uint64_t calculateRunningAvg (uint64_t *dataArr){
    int runningAvg = 0;

    int i;
    for (i = 0; i < 20; i ++){
        runningAvg = runningAvg + dataArr[i];
    }

    runningAvg = runningAvg/20.0;

    return runningAvg;
}

char convertASLtoChar(void){

    return 'a';

}

// updating values on LCD

unsigned long timer_val = 0;

void SysTick_Handler(void)
{
    timer_val ++;

    if (timer_val % 100){
        sendCommand(clear_display);
        sendCommand(FirstRow);
        writeChar(convertASLtoChar());
    }
}

/**
 * main.c
 */
int main(void)
{
    PLLInit();
    SystickInit();
    SetupSerial();

    ADC_Init();

    PortB_Init();
    LCDInit();
    SysTickInterruptInit();

    sendCommand(clear_display);
    sendCommand(FirstRow);

    while(1){}
}

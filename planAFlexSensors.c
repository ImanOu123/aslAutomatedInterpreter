#include <stdint.h>
#include <stdbool.h>
#include "15348.h"
#include "timer.h"
#include <math.h>
#include "serial.h"

// LCD commands

#define clear_display 0x01
#define returnHome 0x02
#define moveCursorRight 0x06
#define moveCursorLeft 0x08
#define shiftDisplayRight 0x1C
#define shiftDisplayLeft 0x18
#define cursorBlink 0x0F
#define cursorOff 0x0C
#define cursorOn 0x0E
#define Function_set_4bit 0x28
#define Function_set_8bit 0x38
#define Entry_mode 0x06
#define Function_8_bit 0x32
#define Set5x7FontSize 0x20
#define FirstRow 0x80

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
    SYSCTL_RCGC2_R |= 0x00000002;   // 1) B clock
    delay = SYSCTL_RCGC2_R;         // reading register adds a delay, which we might need
    GPIO_PORTB_LOCK_R = 0x4C4F434B; // 2) unlock PortB
    GPIO_PORTB_CR_R = 0xFF;         // 3) allow changes to PB
    GPIO_PORTB_AMSEL_R = 0x00;      // 4) disable analog function
    GPIO_PORTB_AFSEL_R = 0x00;      // 5) no alternate function
    GPIO_PORTB_PCTL_R = 0x00000000; // 6) GPIO clear bit PCTL
    GPIO_PORTB_PUR_R = 0x00;        // 7) disable pull up resistors
    GPIO_PORTB_DEN_R = 0xFF;        // 8) enable digital pins PB
    GPIO_PORTB_DIR_R = 0xFF;        // 9) PB output

    GPIO_PORTB_DATA_R = 0x0;
}

// ADC initialization

volatile unsigned long delay;
void ADC_Init(void)
{
    SYSCTL_RCGCADC_R |= 0x01;   // activate ADC0
    SYSCTL_RCGCGPIO_R |= 0x10;  // activate clock for PORT E
    SYSCTL_RCGCTIMER_R |= 0x01; // activate timer0

    GPIO_PORTE_DIR_R &= ~0x3F;  // make PE5-0 input
    GPIO_PORTE_AFSEL_R |= 0x3F; // enable alternate fun on PE5-0
    GPIO_PORTE_DEN_R &= ~0x3F;  // disable digital I/O on PE5-0
    GPIO_PORTE_AMSEL_R |= 0x3F; // enable analog fun on PE5-0

    // extra time to stabilize

    delay = SYSCTL_RCGCADC_R;
    delay = SYSCTL_RCGCADC_R;
    delay = SYSCTL_RCGCADC_R;
    delay = SYSCTL_RCGCADC_R;

    // configuring ADC

    ADC0_PC_R = 0x01;
    ADC0_SSPRI_R = 0x3210;

    // configuring timer

    TIMER0_CTL_R = 0x0;     // disable timer during setup
    TIMER0_CTL_R |= 0x20;   // enable timer0A trigger to ADC
    TIMER0_CFG_R = 0;       // 32-bit timer mode
    TIMER0_TAMR_R = 0x02;   // configure for periodic mode
    TIMER0_TAPR_R = 0;      // pre-scale value
    TIMER0_TAILR_R = 79999; // 8000 cycles in 1ms
    TIMER0_IMR_R = 0x0;     // disable all interrupts
    TIMER0_CTL_R |= 0x01;   // enable timer0A

    // configuring ADC - more

    ADC0_ACTSS_R &= ~0x01;                             // disable SS0
    ADC0_EMUX_R = (ADC0_EMUX_R & 0xFFFFFFF0) + 0x0005; // timer trigger

    // ADC0_SSMUX0_R = 0x00090123; // channels to sample from
    // ADC0_SSCTL0_R = 0x00060000; // no TS0 & D0, yes IE0 & ENDO

    ADC0_SSMUX0_R = 0x123; // channels
    ADC0_SSCTL0_R = 0x600; // set flag (ENDS AT 3RD SAMPLE)

    ADC0_IM_R |= 0x01;    // enable SS0 interrupts
    ADC0_ACTSS_R |= 0x01; // enable SS0

    // enabling interrupts

    NVIC_PRI3_R = (NVIC_PRI3_R & 0xFF00FFFF) | 0x00400000; // ADC0 priority 2
    NVIC_EN0_R = 1 << 14;
}

uint32_t sensor1 = 0;
uint32_t sensor2 = 0;
uint32_t sensor3 = 0;
uint32_t sensor4 = 0;
uint32_t sensor5 = 0;

uint64_t fifoStatusPRE = 0;
uint64_t fifoStatusPOST = 0;

void ADC0Seq0_Handler(void)
{
    ADC0_ISC_R = 0x01; // ack interrupt

    fifoStatusPRE = ADC0_SSFSTAT0_R;

    // read samples

    sensor1 = ADC0_SSFIFO0_R & 0xFFF;
    sensor2 = ADC0_SSFIFO0_R & 0xFFF;
    sensor3 = ADC0_SSFIFO0_R & 0xFFF;

    // FIFO status

    fifoStatusPOST = ADC0_SSFSTAT0_R;

    //    sensor4 = ADC0_SSFIFO0_R&0xFFF;
    //    sensor5 = ADC0_SSFIFO0_R&0xFFF;
}

// LCD Helper Functions

// https://microcontrollerslab.com/16x2-lcd-interfacing-with-tm4c123-tiva-launchpad-keil-uvision/

/* Micro seconds delay function */
void delay_us(int n)
{
    int i, j;
    for (i = 0; i < n; i++)
        for (j = 0; j < 3; j++)
        {
        }
}

void writeNibble(unsigned char data, unsigned char control)
{

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

void sendCommand(unsigned char command)
{
    // send upper and lower 4 bits one after the other
    writeNibble(command & 0xF0, 0x0);
    writeNibble(command << 4, 0x0);

    if (command < 4)
    {
        SysTick_Wait1ms(2);
    }
    else
    {
        delay_us(40);
    }
}

void writeChar(unsigned char data)
{
    writeNibble(data & 0xF0, 0x01);
    writeNibble(data << 4, 0x01);

    delay_us(40);
}

void writeString(char *str)
{
    int i;
    for (i = 0; str[i] != 0; i++)
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

/**
 * main.c
 */
int main(void)
{
    PLLInit();
    SystickInit();
    SetupSerial();

    ADC_Init();

    //    PortB_Init();
    //    LCDInit();
    //
    //    sendCommand(clear_display);
    //    sendCommand(FirstRow);
    //    writeString("down");

    while (1)
    {
    }
}

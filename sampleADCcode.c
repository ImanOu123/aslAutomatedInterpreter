#include <stdint.h>
#include <stdbool.h>
#include "15348.h"
#include "serial.h"
#include <math.h>

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

// ADC initialization

volatile unsigned long delay;
void ADC_Init(void){
    SYSCTL_RCGCADC_R |= 0x01; // activate ADC0
    SYSCTL_RCGCGPIO_R |= 0x10; // activate clock for PORT E
    SYSCTL_RCGCTIMER_R |= 0x01; // activate timer0

    GPIO_PORTE_DIR_R &= ~0x1F; // make PE0-4 input
    GPIO_PORTE_AFSEL_R |= 0x1F; // enable alternate fun on PE0-4
    GPIO_PORTE_DEN_R &= ~0x1F; // disable digital I/O on PE0-4
    GPIO_PORTE_AMSEL_R |= 0x1F; // enable analog fun on PE0-4

    // extra time to stabilize

    delay = SYSCTL_RCGCADC_R;
    delay = SYSCTL_RCGCADC_R;
    delay = SYSCTL_RCGCADC_R;
    delay = SYSCTL_RCGCADC_R;

    // configuring ADC

    ADC0_PC_R = 0x01;   // 125k sampling rate
    ADC0_SSPRI_R = 0x3210;  // Sequencer priorities

    // configuring timer

    TIMER0_CTL_R = 0x0; // disable timer during setup
    TIMER0_CTL_R |= 0x20; // enable timer0A trigger to ADC
    TIMER0_CFG_R = 0; // 32-bit timer mode
    TIMER0_TAMR_R = 0x02; // configure for periodic mode
    TIMER0_TAPR_R = 0; // pre-scale value
    TIMER0_TAILR_R = 0x4C4B400; // 1 second
    TIMER0_IMR_R = 0x0; // disable all interrupts
    TIMER0_CTL_R |= 0x01; // enable timer0A

    // configuring ADC - more

    ADC0_ACTSS_R &= ~0x01; // disable SS0
    ADC0_EMUX_R = (ADC0_EMUX_R&0xFFFFFFF0) + 0x0005; // Seq0 timer trigger

    ADC0_SSMUX0_R = 0x93210; // channels.  Sample AIN0 (PE3), then AIN1 (PE2), then AIN2 (PE1), then AIN3 (PE0), then AIN9 (PE4)
    ADC0_SSCTL0_R = 0x60000; // set flag (ENDS AT 5th SAMPLE)

    ADC0_IM_R |= 0x01; // enable SS0 interrupts
    ADC0_ACTSS_R |= 0x01; // enable SS0

    // enabling interrupts

    NVIC_PRI3_R = (NVIC_PRI3_R&0xFF00FFFF) | 0x00400000; // ADC0 priority 2
    NVIC_EN0_R = 1 << 14;
}

uint32_t sensor1 = 0;
uint32_t sensor2 = 0;
uint32_t sensor3 = 0;
uint32_t sensor4 = 0;
uint32_t sensor5 = 0;

uint64_t fifoStatusPRE = 0;
uint64_t fifoStatusPOST = 0;

void ADC0Seq0_Handler (void){
    ADC0_ISC_R = 0x01; // ack interrupt

    sensor1 = ADC0_SSFIFO0_R&0xFFF;
    sensor2 = ADC0_SSFIFO0_R&0xFFF;
    sensor3 = ADC0_SSFIFO0_R&0xFFF;
    sensor4 = ADC0_SSFIFO0_R&0xFFF;
    sensor5 = ADC0_SSFIFO0_R&0xFFF;

    SerialWriteInt(sensor1);
    SerialWriteInt(sensor2);
    SerialWriteInt(sensor3);
    SerialWriteInt(sensor4);
    SerialWriteInt(sensor5);
    SerialWriteLine("---");
}

/**
 * main.c
 */
int main(void)
{
    PLLInit();
    SetupSerial();

    ADC_Init();

    SerialWriteLine("ADC Go!");

    while(1){}
}

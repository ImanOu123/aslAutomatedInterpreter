#include <stdint.h>
#include <stdbool.h>
#include "15348.h"
#include "serial.h"
#include "timer.h"
#include <math.h>
#include "finalProjectConstants.h"

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
    TIMER0_TAILR_R = 0x2625A00; // 500 ms
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

// I2C initialization

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

void PortF_Init(void)
{
    volatile unsigned long delay;
    SYSCTL_RCGC2_R |= 0x00000020;     // 1) F clock
    delay = SYSCTL_RCGC2_R; // reading register adds a delay, which we might need
    GPIO_PORTF_LOCK_R = 0x4C4F434B;   // 2) unlock PortF PF0
    GPIO_PORTF_CR_R = 0x1F;           // 3) allow changes to PF4-0
    GPIO_PORTF_AMSEL_R = 0x00;        // 4) disable analog function
    GPIO_PORTF_AFSEL_R = 0x00;        // 5) no alternate function
    GPIO_PORTF_PCTL_R = 0x00000000;   // 6) GPIO clear bit PCTL
    GPIO_PORTF_PUR_R = 0x11;          // 7) enable pullup resistors on PF4,PF0
    GPIO_PORTF_DEN_R = 0x1F;          // 8) enable digital pins PF4-PF0
    GPIO_PORTF_DIR_R = 0x0E;          // 9) PF4,PF0 input, PF3,PF2,PF1 output
}

// WAIT functions

void delay_us(int n)
{
    int i,j;
    for(i=0;i<n;i++)
        for(j=0;j<3;j++) {}
}

void Delay(unsigned long counter)
{
    unsigned long i = 0;

    for(i=0; i< counter*10000; i++);
}

// LCD helper functions - https://microcontrollerslab.com/16x2-lcd-interfacing-with-tm4c123-tiva-launchpad-keil-uvision/

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
        SerialWrite("1");
        SysTick_Wait1ms(2);
        SerialWrite("2");
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

// LCD initialization

void LCDInit(void)
{
    sendCommand(Set5x7FontSize);
    sendCommand(Function_set_4bit);
    sendCommand(moveCursorRight);

    SerialWrite("hi2");
    sendCommand(clear_display);

    SerialWrite("hi3");
    sendCommand(cursorBlink);
}

// SysTick Interrupt initialization

void SysTickInterruptInit()
{
    NVIC_ST_CTRL_R = 0;
    NVIC_ST_RELOAD_R = 80000; // exactly 1ms
    NVIC_ST_CURRENT_R = 0;
    NVIC_ST_CTRL_R = 0x00000007;
}

unsigned long timer_val1 = 0;
unsigned long timer_val2 = 0;

void SysTick_Handler(void)
{

    timer_val1 = timer_val1 + 1;
    timer_val2 = timer_val2 + 1;

}

uint32_t sensor1 = 0;
uint32_t sensor2 = 0;
uint32_t sensor3 = 0;
uint32_t sensor4 = 0;
uint32_t sensor5 = 0;

uint32_t filteredSensor1 = 0;
uint32_t filteredSensor2 = 0;
uint32_t filteredSensor3 = 0;
uint32_t filteredSensor4 = 0;
uint32_t filteredSensor5 = 0;

unsigned long interruptCount = 0;

char currLetter = '\0';
char dispLetter = '\0';

// variables to construct word

bool button1Status = 0;
bool button2Status = 0;
bool dispWord = 0;

bool listEq(char **list1, char **list2, int n){
    int i;
    for (i = 0; i < n; i++) if (list1[i] != list2[i]) return 0;
    return 1;
}

// each sensor has 3 states: BENT, MID or UNBENT - for non-complex letters

char *fingStatus[5] = {"UNBENT", "UNBENT", "UNBENT", "UNBENT", "UNBENT"};

void getFingStatus(void){

    // sensor 1

    if (filteredSensor1 <= 800) fingStatus[0] = "UNBENT";
    else if (800 < filteredSensor1 && filteredSensor1 <= 900) fingStatus[0] = "MID";
    else if (900 < filteredSensor1) fingStatus[0] = "BENT";

    // sensor 2

    if (filteredSensor2 < 100) fingStatus[1] = "UNBENT";
    else if (150 < filteredSensor2) fingStatus[1] = "BENT";

    // sensor 3

    if (filteredSensor3 <= 800) fingStatus[2] = "UNBENT";
    else if (800 < filteredSensor3) fingStatus[2] = "BENT";

    // sensor 4

    if (filteredSensor4 <= 100) fingStatus[3] = "UNBENT";
    else if (100 < filteredSensor4) fingStatus[3] = "BENT";

    // sensor 5

    if (filteredSensor5 <= 100) fingStatus[4] = "UNBENT";
    else if (100 < filteredSensor5) fingStatus[4] = "BENT";

}

// de-bouncing using state machine from demo code

// We have four states

#define WAITING 0x00
#define PRESSED 0x01
#define DEBOUNCE_RELEASE  0x02
#define RELEASED 0x03

struct State
{
    unsigned char next[2]; // Next state based on 1-bit input (switch value)
    unsigned char out;     // Whether or not we "output"/take action in this take.  (0 or non-zero)
    unsigned int time;     // min time to stay in this state (intervals of 1 ms)
};

struct State FSM[4] = {
        { { WAITING, PRESSED }, 0x00, 0 }, // State WAITING
        { { DEBOUNCE_RELEASE, PRESSED }, 0x00, 7 }, // State PRESSED
        { { RELEASED, RELEASED }, 0x00, 3 }, // State DEBOUNCE_RELEASE
        { { WAITING, PRESSED }, 0x01, 0 }  // State RELEASED
};

unsigned char curState1 = WAITING;
unsigned char curState2 = WAITING;

bool getButtonStatus(int buttonNum){

    // get button one status

    if (buttonNum == 1){
        // Check if we've been in the state long enough to move on if needed

        if (timer_val1 >= FSM[curState1].time) {

            // Check SW1 status

            int inp = (GPIO_PORTF_DATA_R & 0x10)>>4;
            inp ^= 0x01;

            // Choose the next state

            if (curState1 != FSM[curState1].next[inp]) {

                // If we are going to change states, then reset the counter

                timer_val1 = 0;
            }
            curState1 = FSM[curState1].next[inp];
        }
        return FSM[curState1].out;
    }

    // get button two status

    if (timer_val2 >= FSM[curState2].time) {

        // Check SW2 status
        int inp = (GPIO_PORTF_DATA_R & 0x01);
        inp ^= 0x01;

        // Choose the next state

        if (curState2 != FSM[curState2].next[inp]) {

            // If we are going to change states, then reset the counter

            timer_val2 = 0;
        }
        curState2 = FSM[curState2].next[inp];
    }
    return FSM[curState2].out;

}

char getCurrASLLetter(void){

    // A
    char *A[5] = {"UNBENT", "BENT", "BENT", "BENT", "BENT"};
    if (listEq(fingStatus, A, 5)) return 'a';



    // B
    char *B1[5] = {"MID", "UNBENT", "UNBENT", "UNBENT", "UNBENT"};
    char *B2[5] = {"BENT", "UNBENT", "UNBENT", "UNBENT", "UNBENT"};
    if (listEq(fingStatus, B1, 5) || listEq(fingStatus, B2, 5)) return 'b';

    // C


    // D

    char *D1[5] = {"MID", "UNBENT", "BENT", "UNBENT", "UNBENT"};
    char *D2[5] = {"UNBENT", "UNBENT", "BENT", "UNBENT", "UNBENT"};
    if (listEq(fingStatus, D1, 5) || listEq(fingStatus, D2, 5)) return 'd';

    // E

    // F

    char *F1[5] = {"MID", "BENT", "UNBENT", "UNBENT", "UNBENT"};
    char *F2[5] = {"BENT", "BENT", "UNBENT", "UNBENT", "UNBENT"};
    if (listEq(fingStatus, F1, 5) || listEq(fingStatus, F2, 5)) return 'f';

    // G

    // H

    // I

    char *I1[5] = {"MID", "BENT", "BENT", "BENT", "UNBENT"};
    char *I2[5] = {"BENT", "BENT", "BENT", "BENT", "UNBENT"};
    if (listEq(fingStatus, I1, 5) || listEq(fingStatus, I2, 5)) return 'i';

    // J

    // K

    char *K[5] = {"UNBENT", "UNBENT", "UNBENT", "BENT", "BENT"};
    if (listEq(fingStatus, K, 5)) return 'k';

    // L

    char *L[5] = {"UNBENT", "UNBENT", "BENT", "BENT", "BENT"};
    if (listEq(fingStatus, L, 5)) return 'l';

    // M

    // N

    // O

    // P

    // Q

    // R

    char *R1[5] = {"BENT", "UNBENT", "UNBENT", "BENT", "BENT"};
    char *R2[5] = {"MID", "UNBENT", "UNBENT", "BENT", "BENT"};
    if (listEq(fingStatus, R1, 5) || listEq(fingStatus, R2, 5)) return 'r';

    // S

    // T

    // U

    // V

    // W

    char *W1[5] = {"MID", "UNBENT", "UNBENT", "UNBENT", "BENT"};
    char *W2[5] = {"BENT", "UNBENT", "UNBENT", "UNBENT", "BENT"};
    if (listEq(fingStatus, W1, 5) || listEq(fingStatus, W2, 5)) return 'w';

    // X

    // Y

    char *Y[5] = {"UNBENT", "BENT", "BENT", "BENT", "UNBENT"};
    if (listEq(fingStatus, Y, 5)) return 'y';

    // Z

    return dispLetter;
}

uint32_t exponentialFilter (uint32_t xn, uint32_t yn1){
    float w = 0.4;

    uint32_t yn = (w*xn) + ((1-w)*yn1);

    return yn;
}

void ADC0Seq0_Handler (void){
    ADC0_ISC_R = 0x01; // ack interrupt

    // extract sensor values from ADC

    sensor1 = ADC0_SSFIFO0_R&0xFFF;
    sensor2 = ADC0_SSFIFO0_R&0xFFF;
    sensor3 = ADC0_SSFIFO0_R&0xFFF;
    sensor4 = ADC0_SSFIFO0_R&0xFFF;
    sensor5 = ADC0_SSFIFO0_R&0xFFF;


    if (interruptCount == 0){

        // get initial value for filtering

        filteredSensor1 = sensor1;
        filteredSensor2 = sensor2;
        filteredSensor3 = sensor3;
        filteredSensor4 = sensor4;
        filteredSensor5 = sensor5;

    }
    else{

        // use exponential filter to filter data

        filteredSensor1 = exponentialFilter(sensor1, filteredSensor1);
        filteredSensor2 = exponentialFilter(sensor2, filteredSensor2);
        filteredSensor3 = exponentialFilter(sensor3, filteredSensor3);
        filteredSensor4 = exponentialFilter(sensor4, filteredSensor4);
        filteredSensor5 = exponentialFilter(sensor5, filteredSensor5);

        // display sensor values on serial [FOR DEBUGGING/TESTING PURPOSES]

        SerialWriteLine("flex sensors");
        SerialWriteInt(filteredSensor1);
        SerialWriteInt(filteredSensor2);
        SerialWriteInt(filteredSensor3);
        SerialWriteInt(filteredSensor4);
        SerialWriteInt(filteredSensor5);
        SerialWriteLine("---");
    }


    // every 5 seconds check if letter has changed and then display on LCD

    if (interruptCount % 10 == 0){

        // determine new character

        getFingStatus();

        currLetter = getCurrASLLetter();


        // if current character has not changed from displayed character then don't redisplay the same thing

        if (currLetter != dispLetter){

            dispLetter = currLetter;

            SerialWriteLine(&dispLetter);

            writeChar(dispLetter);

            SerialWriteLine(&dispLetter);

        }
    }

    interruptCount ++;
}

// main.c

int main(void)
{
    // initializations

    PLLInit();
    SystickInit();
    SysTickInterruptInit();
    SetupSerial();

    PortB_Init();
    LCDInit();

    PortF_Init();

    ADC_Init();

    sendCommand(clear_display);
    sendCommand(FirstRow);
    writeString("LCD");

    while(1){
        button1Status = getButtonStatus(1);
        button2Status = getButtonStatus(2);

        if (button1Status){
            sendCommand(clear_display);
            sendCommand(FirstRow);
        }

        if (button2Status){
            sendCommand(0x10);
        }

    }
}

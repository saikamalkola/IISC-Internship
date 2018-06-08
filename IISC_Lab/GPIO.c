#include <stdint.h>
#include "inc/tm4c123gh6pm.h"

//Output Pin definitions
#define red_led 1
#define green_led 3
#define blue_led 2

//Input Pin definitions
#define SW1 4
#define SW2 0

void commit_pins()
{
    GPIO_PORTF_CR_R |= ((1 << red_led) | (1 << green_led) | (1 << blue_led) | (1 << SW1) | (1 << SW2) );
}

void init_pins()
{
    //Inputs
    GPIO_PORTF_DIR_R |= ((1 << red_led) | (1 << green_led) | (1 << blue_led));

    //Outputs
    GPIO_PORTF_DIR_R &= ~((1 << SW1) | (1 << SW2));
}

void enable_pullups()
{
    GPIO_PORTF_PUR_R |= ((1 << SW1) | (1 << SW2));
}

void enable_DFun()
{
    GPIO_PORTF_DEN_R |= ((1 << red_led) | (1 << green_led) | (1 << blue_led) | (1 << SW1) | (1 << SW2) );
}

void init_PORTF()
{
    SYSCTL_RCGC2_R |= 0x00000020;   //Enable Clock to Port F
    GPIO_PORTF_LOCK_R = 0x4C4F434B; //Unlock Port F
    commit_pins();                  //commit pins(to allow changes)
    GPIO_PORTF_AMSEL_R = 0x00;      //disable analog on PF
    GPIO_PORTF_PCTL_R = 0x00000000; //PCTL GPIO on PF4-0
    init_pins();                    //Initializing pins as Inputs or Outputs
    GPIO_PORTF_AFSEL_R = 0x00;      //disable alt funct on PF7-0
    enable_pullups();               //Enabling PullUPs for SW1 and SW2
    enable_DFun();                  //Enabling Digital Functionality of Pins
}

void digitalWrite(int pin, int status)
{
    if(status)
    {
        GPIO_PORTF_DATA_R |= (1 << pin);
    }
    else
    {
        GPIO_PORTF_DATA_R &= ~(1 << pin);
    }
}

int digitalRead(int pin)
{
    return ((GPIO_PORTF_DATA_R >> pin) & 0x01);
}

//int main()
//{
//    init_PORTF();
//    while(1)
//    {
//
//    }
//}

#include <stdint.h>
#include "inc/tm4c123gh6pm.h"

void init_timer0()
{
    SYSCTL_RCGC2_R |= 0x00000020;   //Enable Clock to Port F
    GPIO_PORTF_LOCK_R = 0x4C4F434B; //Unlock Port F
    GPIO_PORTF_DIR_R |= 0x08;
    GPIO_PORTF_CR_R |= 0x08;
    GPIO_PORTF_DEN_R |= 0x08;
    GPIO_PORTF_AFSEL_R |= 0x08;
    GPIO_PORTF_PCTL_R = 0x00007000;

    SYSCTL_RCGCTIMER_R |= 0x02;
    TIMER1_CTL_R = 0x00000000;    //Disabling Timer A
    TIMER1_CFG_R &= ~(0x07);    //Clearing D2:D0 bits
    TIMER1_CFG_R |= 0x04; //Configuring Timer 0 as 16 bit timer
    TIMER1_TBMR_R &= ~(0x03);   //Clearing D1:D0 bits
    TIMER1_TBMR_R |= 0x02;  //Periodic mode
    TIMER1_TBMR_R &= ~(1 << 2);
    TIMER1_TBMR_R |= 0x00;    //Count down mode
    TIMER1_TBMR_R |= (1 << 3);  //PWM Mode
    TIMER1_TBILR_R = 0xFFFF;
    TIMER1_TBPR_R = 0xFF;
    TIMER1_TBMATCHR_R = 0xFFFF;
    TIMER1_TBPMR_R = 0x7F;
    TIMER1_CTL_R |= (1 << 8);    //Enabling Timer A
}

int main()
{
    init_timer0();
    while(1)
    {

    }
}

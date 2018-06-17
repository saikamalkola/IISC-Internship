/*
 * PWM_Module.c
 *
 *  Created on: 11-Jun-2018
 *      Author: kamal
 */

#include <stdint.h>
#include "inc/tm4c123gh6pm.h"

void delayMs(int n);

int main(void)
{
    SYSCTL_RCGCPWM_R |= 0x01;       //enable clock to PWM0
    SYSCTL_RCGCGPIO_R |= ((1 << 1) | (1 << 4)); //Enable clock to Port B and Port E
    SYSCTL_RCC_R &= ~0x00100000;    // no pre-divide for PWM clock

    //Enable PE4, PE5 for M0PWM4, M0PWM5 respectively
    GPIO_PORTE_AFSEL_R |= ((1 << 4) | (1 << 5)); // PE4 and PE5 uses alternate function
    GPIO_PORTE_PCTL_R &= ~(0x00FF0000); // make PE4 and PE5 PWM output pins
    GPIO_PORTE_PCTL_R |= 0x00440000;
    GPIO_PORTE_DEN_R |= ((1 << 4) | (1 << 5)); //Enable digital Functionality for PE4 and PE5 pins

    //Enable PB6, PB7 for M0PWM0, M0PWM1 respectively
    GPIO_PORTB_AFSEL_R |= ((1 << 6) | (1 << 7)); // PB6 and PB7 uses alternate function
    GPIO_PORTB_PCTL_R &= ~(0xFF000000); // make PB6 and PB7 PWM output pins
    GPIO_PORTB_PCTL_R |= 0x44000000;
    GPIO_PORTB_DEN_R |= ((1 << 6) | (1 << 7)); //Enable digital Functionality for PE4 and PE5 pins


    //M0PWM0, M0PWM1
    PWM0_0_CTL_R &= ~(1 << 0); //Stop counter
    PWM0_0_GENA_R = ((2 << 6) | (3 << 4)); //M0PWM0 Generator control register - Drive High on CMPAUP and Drive low on CMPADN
    PWM0_0_GENB_R = ((2 << 10) | (3 << 8)); //M0PWM1 Generator control register - Drive High on CMPbUP and Drive low on CMPBDN
    PWM0_0_LOAD_R = 8000;  //(8MHz / (2 * 8000)  = 500Hz)
    PWM0_0_CMPA_R = 7999; //Duty Cycle  = 0
    PWM0_0_CMPB_R = 7999; //Duty Cycle  = 0
    PWM0_0_CTL_R |= ((1 << 1));  //Counter in count up/down mode

    //M0PWM4, M0PWM5
    PWM0_2_CTL_R &= ~(1 << 0); //Stop counter
    PWM0_2_GENA_R = ((2 << 6) | (3 << 4)); //M0PWM4 Generator control register - Drive High on CMPAUP and Drive low on CMPADN
    PWM0_2_GENB_R = ((2 << 10) | (3 << 8)); //M0PWM5 Generator control register - Drive High on CMPbUP and Drive low on CMPBDN
    PWM0_2_LOAD_R = 8000;  //(16MHz / (2 * 8000)  = 1000Hz)
    PWM0_2_CMPA_R = 7999; //Duty Cycle  = 0
    PWM0_2_CMPB_R = 7999; //Duty Cycle  = 0
    PWM0_2_CTL_R |= ((1 << 1));  //Counter in count up/down mode

    PWM0_ENABLE_R |= ((1 << 0) | (1 << 1)); // Enabling PWM Signal generated M0PWM0 and M0PWM1
    PWM0_ENABLE_R |= ((1 << 4) | (1 << 5)); // Enabling PWM Signal generated M0PWM0 and M0PWM1

    PWM0_0_CTL_R |= 0x01;   //Enabling the PWM
    PWM0_2_CTL_R |= 0x01;   //Enabling the PWM
    int x = 4000;
    for (;;)
    {
       // x = x - 100;

        if (x <= 0)
            x = 8000;

        PWM0_0_CMPA_R = x;
        PWM0_0_CMPB_R = x;
        PWM0_2_CMPA_R = x;
        PWM0_2_CMPB_R = x;

       delayMs(20);
    }
}

/* delay n milliseconds (16 MHz CPU clock) */
void delayMs(int n)
{
    int i, j;

    for (i = 0; i < n; i++)
        for (j = 0; j < 3180; j++)
        {
        } /* do nothing for 1 ms */
}


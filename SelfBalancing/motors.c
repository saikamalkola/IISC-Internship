/*
 * motors.c
 *
 *  Created on: 21-Jun-2018
 *      Author: kshama
 */

#include <stdint.h>
#include "inc/tm4c123gh6pm.h"
#include "motors.h"

void motor(uint8_t index, int value)
{
    int dir = 1;
    if(value > 0)
    {
        dir = 1;
    }
    else
    {
        dir = -1;
        value = -1 * value;
    }
    value = 15999 - value;

    if(value > 15999)
    {
        value = 15999;
    }
    switch(index)
    {
    case 0:
    {
        if(dir == 1)
        {
            GPIO_PORTA_DATA_R |= (1 << 5);
            GPIO_PORTA_DATA_R &= ~(1 << 4);

        }
        else
        {
            GPIO_PORTA_DATA_R &= ~(1 << 5);
            GPIO_PORTA_DATA_R |= (1 << 4);
        }
        PWM0_1_CMPB_R = value;
    }
    break;
    case 1:
    {
        if(dir == 1)
        {
            GPIO_PORTA_DATA_R |= (1 << 3);
            GPIO_PORTA_DATA_R &= ~(1 << 2);

        }
        else
        {
            GPIO_PORTA_DATA_R &= ~(1 << 3);
            GPIO_PORTA_DATA_R |= (1 << 2);
        }
        PWM0_1_CMPA_R = value;
    }
    break;
    }
}

void init_motors(void)
{
    /* Motor PinOut
    *       PB5 - En1
    *       PA5 - InA1
    *       PA4 - InB1
    *       PA3 - InA2
    *       PA2 - InB2
    *       PB4 - En2
    */
    //Initializing PWM pins
    SYSCTL_RCGCPWM_R |= 1;
    SYSCTL_RCGCGPIO_R |= 0x2;
    SYSCTL_RCC_R &= ~0x00100000;

    GPIO_PORTB_LOCK_R = 0x4C4F434B;
    GPIO_PORTB_CR_R |= 0x00000030;
    GPIO_PORTB_AFSEL_R |= 0x30;
    GPIO_PORTB_PCTL_R &= ~0x00FF0000;
    GPIO_PORTB_PCTL_R |= 0x00440000;
    GPIO_PORTB_DEN_R |= 0x30;

    PWM0_1_CTL_R = 0;
    PWM0_1_GENA_R = 0x0000008C;
    PWM0_1_GENB_R = 0x0000080C;
    PWM0_1_LOAD_R = 16000;
    PWM0_1_CMPA_R = 15999;
    PWM0_1_CMPB_R = 15999;
    PWM0_1_CTL_R = 1; /* start timer */

    PWM0_ENABLE_R = 0x0C;

    //Initializing Dir pins
    SYSCTL_RCGC2_R |= 0x01;
    GPIO_PORTA_LOCK_R = 0x4C4F434B;
    GPIO_PORTA_CR_R |= 0x0000003C;
    GPIO_PORTA_AMSEL_R &= ~0x3C;
    GPIO_PORTA_PCTL_R &= ~0x00FFFF00;
    GPIO_PORTA_DIR_R |= 0x3C;
    GPIO_PORTA_AFSEL_R &= ~0x3C;
    GPIO_PORTA_DEN_R |= 0x3C;

}

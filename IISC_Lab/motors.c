/*
 * motors.c
 *
 *  Created on: 16-Jun-2018
 *      Author: kamal
 */

#include <stdbool.h>
#include <stdint.h>
#include "inc/tm4c123gh6pm.h"
#include "motors.h"
#include "PCA9685.h"

extern volatile int desired_velocity[4];

void motors(float *value)
{
    motor(0, value[1]);
    motor(1, value[3]);
    motor(2, value[0]);
    motor(3, value[2]);
}

void set_motor(uint8_t index, int set_point)
{
    //desired_velocity[index] = set_point;
}

void brake()
{
    PCA9685_digitalWrite(0, 1); //Brake
    PCA9685_digitalWrite(1, 1); //Brake
    PCA9685_digitalWrite(2, 1); //Brake
    PCA9685_digitalWrite(3, 1); //Brake
}

void no_brake()
{
    PCA9685_digitalWrite(0, 0); //No Brake
    PCA9685_digitalWrite(1, 0); //No Brake
    PCA9685_digitalWrite(2, 0); //No Brake
    PCA9685_digitalWrite(3, 0); //No Brake
}

void motor(uint8_t index, int value)
{
    int8_t dir = 0;
    if (value >= 0)
    {
        dir = 1;    //Clockwise
    }
    else
    {
        dir = -1;   //Anti Clockwise
        value = -1 * value; //Absolute value
    }

    if (value >= 16000)
    {
        value = 15999;
    }

    value = 16000 - value;

    if (index >= 4)
    {
        return;
    }

    switch (index)
    {
    case 0:
    {
        if (dir == 1)
        {
            GPIO_PORTE_DATA_R |= (1 << 0);
        }
        else if (dir == -1)
        {
            GPIO_PORTE_DATA_R &= ~(1 << 0);
        }
        PWM0_0_CMPA_R = value;
    }
        break;
    case 1:
    {
        if (dir == 1)
        {
            GPIO_PORTF_DATA_R |= (1 << 0);
        }
        else if (dir == -1)
        {
            GPIO_PORTF_DATA_R &= ~(1 << 0);
        }
        PWM0_0_CMPB_R = value;
    }
        break;
    case 2:
    {
        if (dir == 1)
        {
            GPIO_PORTA_DATA_R &= ~(1 << 4);
        }
        else if (dir == -1)
        {
            GPIO_PORTA_DATA_R |= (1 << 4);
        }
        PWM0_2_CMPA_R = value;
    }
        break;
    case 3:
    {
        if (dir == 1)
        {
            GPIO_PORTA_DATA_R &= ~(1 << 3);
        }
        else if (dir == -1)
        {
            GPIO_PORTA_DATA_R |= (1 << 3);
        }
        PWM0_2_CMPB_R = value;
    }
        break;
    default:
        break;
    }
}

void init_motors()
{
    /*
     *Initializing PWM Pins
     *      Front ---> PB6 | Back ---> PB7 | Left ---> PE4 | Right ---> PE5
     */

    SYSCTL_RCGCPWM_R |= 0x01;       //enable clock to PWM0
    SYSCTL_RCGCGPIO_R |= ((1 << 1) | (1 << 4)); //Enable clock to Port B and Port E
    SYSCTL_RCC_R &= ~(0x00100000);    // no pre-divide for PWM clock

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
    PWM0_0_LOAD_R = 16000;  //(16MHz / (2 * 16000)  = 500Hz)
    PWM0_0_CMPA_R = 15999; //Duty Cycle  = 0
    PWM0_0_CMPB_R = 15999; //Duty Cycle  = 0
    PWM0_0_CTL_R |= ((1 << 1));  //Counter in count up/down mode

    //M0PWM4, M0PWM5
    PWM0_2_CTL_R &= ~(1 << 0); //Stop counter
    PWM0_2_GENA_R = ((2 << 6) | (3 << 4)); //M0PWM4 Generator control register - Drive High on CMPAUP and Drive low on CMPADN
    PWM0_2_GENB_R = ((2 << 10) | (3 << 8)); //M0PWM5 Generator control register - Drive High on CMPbUP and Drive low on CMPBDN
    PWM0_2_LOAD_R = 16000;  //(16MHz / (2 * 16000)  = 500Hz)
    PWM0_2_CMPA_R = 15999; //Duty Cycle  = 0
    PWM0_2_CMPB_R = 15999; //Duty Cycle  = 0
    PWM0_2_CTL_R |= ((1 << 1));  //Counter in count up/down mode

    PWM0_ENABLE_R |= ((1 << 0) | (1 << 1)); // Enabling PWM Signal generated M0PWM0 and M0PWM1
    PWM0_ENABLE_R |= ((1 << 4) | (1 << 5)); // Enabling PWM Signal generated M0PWM0 and M0PWM1

    PWM0_0_CTL_R |= 0x01;   //Enabling the PWM
    PWM0_2_CTL_R |= 0x01;   //Enabling the PWM

    /*
     * Initializing dir pins
     *   Front ---> PE0 | Back ---> PF0 | Left ---> PA4 | Right ---> PA3
     */

    //Front Motor - PE0
    SYSCTL_RCGC2_R |= (1 << 4); //Enabling Clock to GPIO PORTE
    GPIO_PORTE_LOCK_R = 0x4C4F434B; //unlock GPIO PORTE
    GPIO_PORTE_CR_R |= (1 << 0); //allow changes to PE0
    GPIO_PORTE_AMSEL_R &= ~(1 << 0); //disable analog on PE0
    GPIO_PORTE_PCTL_R &= ~(0xF); //PCTL GPIO on PE0
    GPIO_PORTE_DIR_R |= (1 << 0); //PE0 as output
    GPIO_PORTE_AFSEL_R &= ~(1 << 0); //disable alternate function on PE0
    GPIO_PORTE_DEN_R |= (1 << 0); //enable digital function on PE0

    //Back Motor - PF0
    SYSCTL_RCGC2_R |= (1 << 5); //Enabling Clock to GPIO PORTF
    GPIO_PORTF_LOCK_R = 0x4C4F434B; //unlock GPIO PORTF
    GPIO_PORTF_CR_R |= (1 << 0); //allow changes to PF0
    GPIO_PORTF_AMSEL_R &= ~(1 << 0); //disable analog on PF0
    GPIO_PORTF_PCTL_R &= ~(0xF); //PCTL GPIO on PF0
    GPIO_PORTF_DIR_R |= (1 << 0); //PE0 as output
    GPIO_PORTF_AFSEL_R &= ~(1 << 0); //disable alternate function on PF0
    GPIO_PORTF_DEN_R |= (1 << 0); //enable digital function on PF0

    //Left and Right Motors - PA3 and PA4
    SYSCTL_RCGC2_R |= (1 << 0); //Enabling Clock to GPIO PORTA
    GPIO_PORTA_LOCK_R = 0x4C4F434B; //unlock GPIO PORTA
    GPIO_PORTA_CR_R |= ((1 << 3) | (1 << 4)); //allow changes to PA3 and PA4
    GPIO_PORTA_AMSEL_R &= ~((1 << 3) | (1 << 4)); //disable analog on PA3 and PA4
    GPIO_PORTA_PCTL_R &= ~(0xF << 12 | 0xF << 16); //PCTL GPIO on PA3 and PA4
    GPIO_PORTA_DIR_R |= ((1 << 3) | (1 << 4)); //PA3 and PA4 as output
    GPIO_PORTA_AFSEL_R &= ~((1 << 3) | (1 << 4)); //disable alternate function on PA3 and PA4
    GPIO_PORTA_DEN_R |= ((1 << 3) | (1 << 4)); //enable digital function on PA3 and PA4

}

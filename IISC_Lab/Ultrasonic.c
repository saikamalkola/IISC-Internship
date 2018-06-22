/*
 * Ultrasonic.c
 *
 *  Created on: 18-Jun-2018
 *      Author: kamal
 */

#include <stdbool.h>
#include <stdint.h>
#include "inc/tm4c123gh6pm.h"
#include "Ultrasonic.h"
#include "PCA9685.h"
#include "odometry.h"
#include "motors.h"
#include "KinModel.h"

volatile int dir = 1;
volatile float error[2] = { 0, 0 }, last_error[2] = { 0, 0 };
volatile float dis_PID = 0, dis_P = 0, dis_I = 0, dis_D = 0;
volatile float dis_Kp = 0.15, dis_Ki = 0, dis_Kd = 0;

extern volatile float distance[4];
volatile int count = 0;

unsigned long micros_overflow = 0;

unsigned long micros()
{
    return ((micros_overflow << 16) + TIME_US);
}

void TIMER1_TA_Handler(void)
{
    micros_overflow++;
    volatile int read_back = 0;
    TIMER1_ICR_R = 0x01;
    read_back = TIMER1_ICR_R;
}

void TIMER2_TA_Handler(void)
{
    if(count > 4)
    {
        count = 0;
    }

    if(count != 4)
    {
        PCA9685_digitalWrite(count + 4, 0); //Low
        delayUs(10);
        PCA9685_digitalWrite(count + 4, 1); //High
        delayUs(10);
        PCA9685_digitalWrite(count + 4, 0); //Low
    }
    else
    {
        //Do PID all distances are ready
        error[0] = distance[0] - distance[1];
        error[1] = distance[2] - distance[3];
        if((distance[0] + distance[1]) > 2500)
        {
            if(distance[0] > 1250 && distance[1] > 1250)
            {
                error[0] = 0;
            }
            if(distance[0] > distance[1])
            {
                error[0] =  890 - distance[1];
            }
            else
            {
                error[0] =  distance[0] - 890;
            }
        }
        if((distance[2] + distance[3]) > 2500)
        {
            if(distance[2] > 1250 && distance[3] > 1250)
            {
                error[1] = 0;
            }
            if(distance[2] > distance[3])
            {
                error[1] =  890 - distance[3];
            }
            else
            {
                error[1] =  distance[2] - 890;
            }
        }
        switch (abs(dir))
        {
        case 1:
        {
            dis_P = dis_Kp * error[1];
            dis_D = dis_Kd * (error[1] - last_error[1]);
            dis_PID = dis_P + dis_D;
            last_error[1] = error[1];
            if (abs(dis_PID) >  10000)
            {
                dis_PID = sign(dis_PID) * 10000;
            }
//            set_motor(0, dis_PID);
//            set_motor(1, -dis_PID);
        }
            break;
        case 2:
        {
            dis_P = dis_Kp * error[0];
            dis_D = dis_Kd * (error[0] - last_error[0]);
            dis_PID = dis_P + dis_D;
            last_error[0] = error[0];
            if (abs(dis_PID) > 10000)
            {
                dis_PID = sign(dis_PID) * 10000;
            }
//            set_motor(2, -dis_PID);
//            set_motor(3, dis_PID);
        }
            break;

    }
    }
    count++;
    volatile int read_back = 0;
    TIMER2_ICR_R = 0x01;
    read_back = TIMER2_ICR_R;
}

uint8_t echo_state(uint8_t index)
{
    uint8_t state = 0;
    switch (index)
    {
    case 0:
    {
        state = ECHO_PIN0;
    }
        break;
    case 1:
    {
        state = ECHO_PIN1;
    }
        break;
    case 2:
    {
        state = ECHO_PIN2;
    }
        break;
    case 3:
    {
        state = ECHO_PIN3;
    }
        break;
    }
    return state;
}

float read_distance(uint8_t index, uint16_t timeOut)
{
    float dist = 0;
    PCA9685_digitalWrite(index + 4, 0); //Low
    delayUs(10);
    PCA9685_digitalWrite(index + 4, 1); //High
    delayUs(10);
    PCA9685_digitalWrite(index + 4, 0); //Low

    TIMER1_CTL_R &= ~(TAEN);
    TIMER1_TAPR_R = 0x10;
    TIMER1_TAILR_R = 0xFFFF;
    TIMER1_CTL_R |= TAEN;

    //Wait until pin becomes HIGH (RISING EDGE)
    while (!echo_state(index))
    {

        if (TIME_US > timeOut)
        {
            return -1;
        }
    }

    //Reload the time and restart the timer
    TIMER1_CTL_R &= ~(TAEN);
    TIMER1_TAPR_R = 0x10;
    TIMER1_TAILR_R = 0xFFFF;
    TIMER1_CTL_R |= TAEN;

    while (echo_state(index))
    {
        if (TIME_US > timeOut)
        {
            return -1;
        }
    }
    dist = (float) TIME_US * 0.17;
    TIMER1_CTL_R &= ~(TAEN);
    return dist;
}

void delayUs(int n)
{
    int i, j;
    for (i = 0; i < n; i++)
        for (j = 0; j < 4; j++)
        {
        }
}

void Ultrasonic_Init(void)
{
    SYSCTL_RCGC2_R |= (1 << 3); //Enabling Clock to GPIO PORTD
    GPIO_PORTD_LOCK_R = 0x4C4F434B; //unlock GPIO PORTD
    GPIO_PORTD_CR_R |= (0x0F); //allow changes to PD0-3
    GPIO_PORTD_AMSEL_R &= ~(0x0F); //disable analog on PD0-3
    GPIO_PORTD_PCTL_R &= ~(0x0000FFFF); //PCTL GPIO on PD0-3
    GPIO_PORTD_DIR_R &= ~(0x0F); //PD0-3 as Inputs
    GPIO_PORTD_AFSEL_R &= ~(0x0F); //disable alternate function on PD0-3
    GPIO_PORTD_DEN_R |= (0x0F); //enable digital function on PD0-3

    //Interrupt Settings
    GPIO_PORTD_IBE_R |= (0x0F); //Enabling Both Edge Interrupts on pins PD0-3
    GPIO_PORTD_ICR_R |= (0x0F);; //Clearing interrupt flags of PD0-3
    GPIO_PORTD_IM_R |= (0x0F);;   //Unmask PD0-3 interrupts
}

void init_timer1A(void)
{
    SYSCTL_RCGCTIMER_R |= 0x02;
    TIMER1_CTL_R &= ~(TAEN);    //Disabling Timer A
    TIMER1_CFG_R &= ~(0x07);    //Clearing D2:D0 bits
    TIMER1_CFG_R |= BIT16_MODE; //Configuring Timer 0 as 16 bit timer
    TIMER1_TAMR_R &= ~(0x03);   //Clearing D1:D0 bits
    TIMER1_TAMR_R |= PERIODIC;  //Periodic mode
    TIMER1_TAMR_R |= COUNT_DOWN;    //Count down mode
    TIMER1_TAPR_R = 0x10;
    TIMER1_TAILR_R = 0xFFFF;
    TIMER1_ICR_R |= 0x01;
    TIMER1_CTL_R |= TAEN;
    TIMER1_IMR_R |= TATORIM;    //Enabling Interrupt Timerout
    NVIC_PRI5_R = (NVIC_PRI5_R & 0xFFFF1FFF) | 0x0000A000;
    NVIC_EN0_R |= (1 << 21);
}

void init_timer2A(int time_ms)
{
    SYSCTL_RCGCTIMER_R |= 0x04;
    TIMER2_CTL_R &= ~(TAEN);    //Disabling Timer A
    TIMER2_CFG_R &= ~(0x07);    //Clearing D2:D0 bits
    TIMER2_CFG_R |= BIT16_MODE; //Configuring Timer 0 as 16 bit timer
    TIMER2_TAMR_R &= ~(0x03);   //Clearing D1:D0 bits
    TIMER2_TAMR_R |= PERIODIC;  //Periodic mode
    TIMER2_TAMR_R |= COUNT_DOWN;    //Count down mode
    uint32_t load_register = (time_ms * 16 * 1000 / 256) - 1; //Calculating ticks required at 16MHz
    TIMER2_TAPR_R = 0xFF;
    TIMER2_TAILR_R = load_register;
    TIMER2_ICR_R |= 0x01;
    TIMER2_CTL_R |= TAEN;    //Enabling Timer A

    TIMER2_IMR_R |= TATORIM;    //Enabling Interrupt Timerout
    NVIC_PRI5_R = (NVIC_PRI5_R & 0x1FFFFFFF) | 0xA0000000;
    NVIC_EN0_R |= (1 << 23);
}


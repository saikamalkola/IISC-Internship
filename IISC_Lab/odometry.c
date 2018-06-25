/*
 * odometry.c
 *
 *  Created on: 17-Jun-2018
 *      Author: kamal
 */

#include <stdbool.h>
#include <stdint.h>
#include "inc/tm4c123gh6pm.h"
#include "odometry.h"
#include <math.h>
#include "motors.h"
#include "PCA9685.h"
#include "KinModel.h"

extern volatile long encoder_value[4];
const float K = 0.261799;    //(PI*D/N)
const float N = 1200;
const float d = 291;  //in metres
volatile int velocity[4] = { 0, 0, 0, 0 };    //Actual RPM of motors
volatile int desired_velocity[4] = { 0, 0, 0, 0 };
volatile int PID[4] = { 0, 0, 0, 0 }, P[4] = { 0, 0, 0, 0 };
volatile int D[4] = { 0, 0, 0, 0 };
volatile int Kp[4] = { 80, 110, 80, 80 }, Kd[4] = { 10, 10, 10, 10 };
volatile int velocity_error[4] = { 0, 0, 0, 0 }, last_velocity_error[4] = { 0, 0, 0, 0 };

struct Position
{
    volatile float x;
    volatile float y;
    volatile float theta;
} position = { 0, 0, 0 };

extern struct location
{
    float distance; //Distance between initial point and origin
    uint8_t side;   //Left - 0 Right - 1
} present;

void TIMER0_TA_Handler(void)
{
    int i = 0;
    float d_F = 0, d_B = 0, d_L = 0, d_R = 0, d_FB = 0, d_LR = 0, theta_FB = 0,
            theta_LR = 0, d_theta = 0;

    d_F = K * encoder_value[0];
    d_B = K * encoder_value[1];
    d_L = K * encoder_value[2];
    d_R = K * encoder_value[3];

    d_FB = (d_F - d_B) * 0.5;
    d_LR = (d_R - d_L) * 0.5;

    theta_FB = (d_F + d_B) / d;
    theta_LR = (d_L + d_R) / d;
    d_theta = (theta_FB + theta_LR) * 0.5;
    d_theta = -d_theta * 0.5;

    position.x += d_FB;//(d_FB * cosf(position.theta + d_theta)) + (d_LR * cosf(position.theta + M_PI_2 + d_theta));
    position.y += d_LR;//(d_FB * sinf(position.theta + d_theta)) + (d_LR * sinf(position.theta + M_PI_2 + d_theta));
    position.theta += (d_theta * 2);// * (180 / M_PI));
//    position.x += (d_FB * cosf(position.theta + d_theta))
//            + (d_LR * cosf(position.theta + M_PI_2 + d_theta));
//    position.y += (d_FB * sinf(position.theta + d_theta))
//            + (d_LR * sinf(position.theta + M_PI_2 + d_theta));
//    position.theta += ((theta_FB + theta_LR) / 2);

    present.distance = position.y;
    for (i = 0; i < 4; i++)
    {
        velocity[i] = encoder_value[i]; //In RPM Conversion factor = (60*1000)/(1200*50) = 1
        velocity_error[i] = desired_velocity[i] - velocity[i];
        P[i] = Kp[i] * velocity_error[i];
        D[i] = Kd[i] * (velocity_error[i] - last_velocity_error[i]);
        last_velocity_error[i] = velocity_error[i];
        PID[i] = P[i] + D[i];
        if (abs(PID[i]) > 10000)
        {
            PID[i] = sign(PID[i]) * 10000;
        }
       //motor(i, -PID[i]);
        encoder_value[i] = 0;
    }
    volatile int read_back = 0;
    TIMER0_ICR_R = 0x01;
    read_back = TIMER0_ICR_R;
}

int sign(int val)
{
    return (val < 0) ? -1 : (val > 0);
}

void init_timer0A(int time_ms)
{
    SYSCTL_RCGCTIMER_R |= 0x01;
    TIMER0_CTL_R &= ~(TAEN);    //Disabling Timer A
    TIMER0_CFG_R &= ~(0x07);    //Clearing D2:D0 bits
    TIMER0_CFG_R |= BIT16_MODE; //Configuring Timer 0 as 16 bit timer
    TIMER0_TAMR_R &= ~(0x03);   //Clearing D1:D0 bits
    TIMER0_TAMR_R |= PERIODIC;  //Periodic mode
    TIMER0_TAMR_R |= COUNT_DOWN;    //Count down mode
    uint32_t load_register = (time_ms * 16 * 1000 / 256) - 1; //Calculating ticks required at 16MHz
    TIMER0_TAPR_R = 0xFF;
    TIMER0_TAILR_R = load_register;
    TIMER0_ICR_R |= 0x01;
    TIMER0_CTL_R |= TAEN;    //Enabling Timer A
    TIMER0_IMR_R |= TATORIM;    //Enabling Interrupt Timerout
    NVIC_PRI4_R = (NVIC_PRI4_R & 0x1FFFFFFF) | 0x20000000;
    NVIC_EN0_R |= (1 << 19);
}

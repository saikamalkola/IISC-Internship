/*
 * SelfNavigation.c
 *
 *  Created on: 13-Jun-2018
 *      Author: kamal
 */

#include <stdbool.h>
#include <stdint.h>
#include "inc/tm4c123gh6pm.h"
#include "encoders.h"
#include "motors.h"
#include "UartSerial.h"
#include "PCA9685.h"
#include "odometry.h"
#include "Ultrasonic.h"
#include <math.h>
#include "KinModel.h"

//#include "FreeRTOS.h"
//#include "task.h"
//#include "queue.h"
//#include "semphr.h"

//Function definitions
void delayMs(int n);
void DisableInterrupts(void);
void EnableInterrupts(void);

extern volatile long encoder_value[4];
extern volatile int lastEncoded[4];
extern volatile int velocity[4];
extern volatile int velocity_error[4], PID[4];

void move(float distance, int dir);
float absolute(float val);
int sign(int val);

volatile float distance[4] = { 0, 0, 0, 0 };

extern volatile float error[2];
extern volatile float dis_PID;
extern volatile int dir;
extern float w[4];
extern float V[3];

extern struct Position
{
    volatile float x;
    volatile float y;
    volatile float theta;
} position;

int main()
{
    //DisableInterrupts();
    Ultrasonic_Init();
    init_encoders();    //Initializing Encoder Pins
    init_motors();  //Initializing Motor Pins
    UART_Init();    //Initialising UART
    init_I2C1();
    init_PCA9685();
    init_timer0A(25);
    init_timer1A();
    init_timer2A(25);
    EnableInterrupts();

    while (1)
    {
        int i = 0;
        //set_motor(0,100);
//        SerialPrintInt(position.x);
//        UART_OutChar('\t');
//        SerialPrintInt(position.y);
//        UART_OutChar('\t');
//        SerialPrintInt(position.theta);
        float a = 4;
        a = sqrt(a);
        for(i = 0; i < 4; i++)
        {
            SerialPrintInt(distance[i]);
            UART_OutChar('\t');
        }
        UART_OutChar('\n');

////// 51200 21305
//          dir = 1;
//         // delayMs(10000);
//          dir = 2;
//          delayMs(10000);
        UART_OutChar('F');

        dir = 1;
        move(500, 1);
        //   delayMs(500);
//        UART_OutChar('R');
        dir = -2;
        move(500, -2);
           delayMs(500);
//        UART_OutChar('B');
        dir = -1;
        move(500, -1);
     //   delayMs(500);
//        UART_OutChar('L');
        dir = 2;
        move(500, 2);
       // delayMs(500);
        UART_OutChar('\n');
    }
}

void move(float distance, int dir)
{
    int i = 0;
    float current = 0, previous = 0;
    switch (dir)
    {
    case 1:
    {
        for (i = 0; i <= 100; i++)
        {
            set_motor(2, -i);
            set_motor(3, i);
            delayMs(1);
        }
        previous = position.y;
        current = previous;
        while ((current - previous) < distance)
        {
            current = position.y;
            set_motor(2, -100);
            set_motor(3, 100);
//            motor(2, 12000 + 1000);
//            motor(3, -11000);
        }
        for (i = 100; i >= 0; i--)
        {
            set_motor(2, -i);
            set_motor(3, i);
            delayMs(1);
        }
//        motor(2,0);
//        motor(3,0);
    }
        break;
    case -1:
    {
        for (i = 0; i <= 100; i++)
        {
            set_motor(2, i);
            set_motor(3, -i);
            delayMs(1);
        }
        previous = position.y;
        current = previous;
        while ((previous - current) < distance)
        {
            current = position.y;
            set_motor(2, 100);
            set_motor(3, -100);
//            motor(2, -11000);
//            motor(3, 11000);
        }
        for (i = 100; i >= 0; i--)
        {
            set_motor(2, i);
            set_motor(3, -i);
            delayMs(1);
        }
//        motor(2,0);
//        motor(3,0);
    }
        break;
    case 2:
    {
        for (i = 0; i <= 100; i++)
        {
            set_motor(0, i);
            set_motor(1, -i);
            delayMs(1);
        }
        previous = position.x;
        current = previous;
        while ((current - previous) < distance)
        {
            current = position.x;
            set_motor(0, 100);
            set_motor(1, -100);
//            motor(0, -11000);
//            motor(1, 11000);
        }
        for (i = 100; i >= 0; i--)
        {
            set_motor(0, i);
            set_motor(1, -i);
            delayMs(1);
        }
    }
        break;
    case -2:
    {
        for (i = 0; i <= 100; i++)
        {
            set_motor(0, -i);
            set_motor(1, i);
            delayMs(1);
        }
        previous = position.x;
        current = previous;
        while ((previous - current) < distance)
        {
            current = position.x;
            set_motor(0, -100);
            set_motor(1, 100);
//            motor(0, 11000);
//            motor(1, -11000);
        }
        for (i = 100; i >= 0; i--)
        {
            set_motor(0, -i);
            set_motor(1, i);
            delayMs(1);
        }
    }
        break;
    }
}

void delayMs(int n)
{
    int i, j;
    for (i = 0; i < n; i++)
        for (j = 0; j < 3180; j++)
        {
        }
}

void DisableInterrupts(void)
{
    __asm ("    CPSID  I\n");
}

void EnableInterrupts(void)
{
    __asm ("    CPSIE  I\n");
}

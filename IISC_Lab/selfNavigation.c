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
void move(float distance, int dir);
float absolute(float val);
int sign(int val);

extern struct Position
{
    volatile float x;
    volatile float y;
    volatile float theta;
} position;

int main()
{
    //DisableInterrupts();
    init_encoders();    //Initializing Encoder Pins
    init_motors();  //Initializing Motor Pins
    UART_Init();    //Initialising UART
    init_I2C1();
    init_PCA9685();
    init_timer(50);
    EnableInterrupts();
    int i = 0, j = 0;
    while (1)
    {
        SerialPrintInt(position.x);
        UART_OutChar('\t');
        SerialPrintInt(position.y);
        UART_OutChar('\t');
        SerialPrintInt(position.theta);
        UART_OutChar('\n');
        move(500, 1);
        delayMs(500);
        move(500, -2);
        delayMs(500);
        move(500, -1);
        delayMs(500);
        move(500, 2);
        delayMs(500);
    }
}

void move(float distance, int dir)
{
    float current = 0, previous = 0;
    switch(dir)
    {
    case 1:
    {
        previous = position.y;
        current = previous;
        while((current - previous) < distance)
        {
            current = position.y;
            motor(2, 11000);
            motor(3, -11000);
        }
        motor(2,0);
        motor(3,0);
    }
    break;
    case -1:
    {
        previous = position.y;
        current = previous;
        while((previous - current) < distance)
        {
            current = position.y;
            motor(2, -11000);
            motor(3, 11000);
        }
        motor(2,0);
        motor(3,0);
    }
    break;
    case 2:
    {
        previous = position.x;
        current = previous;
        while((current - previous) < distance)
        {
            current = position.x;
            motor(0, -11000);
            motor(1, 11000);
        }
        motor(0,0);
        motor(1,0);
    }
    break;
    case -2:
    {
        previous = position.x;
        current = previous;
        while((previous - current) < distance)
        {
            current = position.x;
            motor(0, 11000);
            motor(1, -11000);
        }
        motor(0,0);
        motor(1,0);
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

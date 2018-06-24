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
#include "WiFi_Comm.h"

#define CORRECTION_TIME 10000L
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
void correct_orientation();
int sign(int val);

volatile float distance[4] = { 0, 0, 0, 0 };

extern float w[4];
extern float V[3];

volatile int dir = 1;
volatile float dis_PID = 0, dis_P = 0, dis_I = 0, dis_D = 0;
volatile float dis_Kp = 0.003, dis_Ki = 0, dis_Kd = 0;

extern struct Position
{
    volatile float x;
    volatile float y;
    volatile float theta;
} position;

int main()
{
    DisableInterrupts();
    Ultrasonic_Init();
    init_encoders();    //Initializing Encoder Pins
    init_motors();  //Initializing Motor Pins
    UART_Init();    //Initialising UART
    UART1_Init();
    init_I2C1();
    init_PCA9685();
    init_timer0A(25);
    init_timer1A();
    init_timer2A(50);
    EnableInterrupts();
    delayMs(2000);
    while (1)
    {
        int i = 0;
        correct_orientation();
//        while (1)
//        {
//            PCA9685_digitalWrite(BUZZER, 1);
//            delayMs(1);
//            PCA9685_digitalWrite(BUZZER, 0);
//            delayMs(1);
//
//        }
//        while (1)
//        {
//            for(i = 0; i < 4; i++)
//            {
//                SerialPrintInt(distance[i]);
//                UART_OutChar('\t');
//            }
//            UART_OutChar('\n');
//        }
//        while (1)
//        {
//            SerialPrintInt(position.x);
//            UART_OutChar('\t');
//            SerialPrintInt(position.y);
//            UART_OutChar('\t');
//            SerialPrintInt(position.theta * 180 / 3.14);
//            UART_OutChar('\n');
//        }
////// 51200 21305
//          dir = 1;
//         // delayMs(10000);
//          dir = 2;
//          delayMs(10000);
//        UART_OutChar('F');
//        dir = 1;
//        move(50000, 1);
//        delayMs(500);
////        dir = -2;
////        move(20000, -2);
////        delayMs(500);
////        UART_OutChar('R');
//        dir = -1;
//        move(50000, -1);
//        delayMs(500);
//        UART_OutChar('B');
//        dir = 2;
//        move(20000, 2);
//        delayMs(500);
//        UART_OutChar('\n');
    }
}

void correct_orientation()
{
    float error[3] = { 0, 0, 0 };
    unsigned long prev_ms = millis();

    while ((millis() - prev_ms) < 500)
    {
        motor(0, 0);
        motor(1, 0);
        motor(2, 0);
        motor(3, 0);
    }

//    prev_ms = millis();
//    while ((millis() - prev_ms) < 2000)
//    {
//        error[2] = (-1 * position.theta * 180 / 3.14);
//        V[0] = 0;
//        V[1] = 0;
//        V[2] = Kp * error[2];
//        SerialPrintInt(V[2]);
//        UART_OutChar('\n');
//        set_velocity();
//        delayMs(10);
//    }
//
    prev_ms = millis();
    while (1)//(millis() - prev_ms) < 2000)
    {
        int i = 0;
        error[1] = distance[0] - distance[1];
        error[2] = distance[2] - distance[3];
        if((distance[0] + distance[1]) > 2000)
        {
            if(distance[0] > 900 && distance[1] > 900)
            {
                error[1] = 0;
            }
            if(distance[0] > distance[1])
            {
                error[1] =  890 - distance[1];
            }
            else
            {
                error[1] =  distance[0] - 890;
            }
        }
        if((distance[2] + distance[3]) > 2000)
        {
            if(distance[2] > 900 && distance[3] > 900)
            {
                error[2] = 0;
            }
            if(distance[2] > distance[3])
            {
                error[2] =  890 - distance[3];
            }
            else
            {
                error[2] =  distance[2] - 890;
            }
        }
        for(i = 0; i < 4; i++)
        {
            if(distance[i] == -1)
            {
                distance[i] = 890;
            }
            printString_UI("D :");
            UIPrintInt(distance[i]);
            UI_OutChar('\t');
        }
        printString_UI("Ex :");
        UIPrintInt(error[2]);
        UI_OutChar('\t');
        printString_UI("Ey :");
        UIPrintInt(error[1]);
        UI_OutChar('\n');
        switch (abs(dir))
        {
        case 1:
        {
            dis_P = dis_Kp * error[2];
            dis_PID = dis_P + dis_D;
            if (abs(dis_PID) >  1)
            {
                dis_PID = sign(dis_PID) * 1.5;
            }
            V[0] = -dis_PID;
            V[1] = 0;
            V[2] = 0;
          //  set_velocity();
    //            set_motor(0, dis_PID);
    //            set_motor(1, -dis_PID);
        }
            break;
        case 2:
        {
            dis_P = dis_Kp * error[1];
            dis_PID = dis_P + dis_D;
            if (abs(dis_PID) > 1.5)
            {
                dis_PID = sign(dis_PID) * 1.5;
            }
            V[0] = 0;
            V[1] = dis_PID;
            V[2] = 0;
          //  set_velocity();
        }
            break;
        }
    }
//    Kp = 0.01;
//    while ((millis() - prev_ms) < 2000)
//    {
//        error[0] = (position.x);
//        V[0] = Kp * error[0];
//        V[1] = 0;
//        V[2] = 0;
//        SerialPrintInt(V[0]);
//        UART_OutChar('\n');
//        set_velocity();
//        delayMs(10);
//    }
//    Kp = 0.01;
//    prev_ms = millis();
//    while ((millis() - prev_ms) < 2000)
//    {
//        error[1] = (-1 * position.y);
//        V[0] = 0;
//        V[1] = Kp * error[1];
//        V[2] = 0;
//        SerialPrintInt(V[1]);
//        UART_OutChar('\n');
//        set_velocity();
//        delayMs(10);
//    }
//
    prev_ms = millis();
    while ((millis() - prev_ms) < 2000)
    {
        motor(0, 0);
        motor(1, 0);
        motor(2, 0);
        motor(3, 0);
    }
}

void move(float distance, int dir)
{
    int i = 0;
    float current = 0, previous = 0;
    unsigned long prev_ms = millis();
    switch (dir)
    {
    case 1:
    {
        for (i = 0; i <= 100; i++)
        {
            V[0] = 0;
            V[1] = (float) i / 100;
            V[2] = 0;
            set_velocity();
            delayMs(1);
        }
        previous = position.y;
        current = previous;
        prev_ms = millis();
        while ((current - previous) < distance)
        {
            SerialPrintInt(position.x);
            UART_OutChar('\t');
            SerialPrintInt(position.y);
            UART_OutChar('\t');
            SerialPrintInt(position.theta * 180 / 3.14);
            UART_OutChar('\n');
            if ((millis() - prev_ms) > CORRECTION_TIME)
            {
                correct_orientation();
                prev_ms = millis();
            }
            current = position.y;
            V[0] = 0;
            V[1] = 1.0;
            V[2] = 0;
            set_velocity();
//            motor(2, 12000 + 1000);
//            motor(3, -11000);
        }
        for (i = 100; i >= 0; i--)
        {
            V[0] = 0;
            V[1] = (float) i / 100;
            V[2] = 0;
            set_velocity();
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
            V[0] = 0;
            V[1] = -(float) i / 100;
            V[2] = 0;
            set_velocity();
            delayMs(1);
        }
        previous = position.y;
        current = previous;
        prev_ms = millis();
        while ((previous - current) < distance)
        {
            if ((millis() - prev_ms) > CORRECTION_TIME)
            {
                correct_orientation();
                prev_ms = millis();
            }
            current = position.y;
            V[0] = 0;
            V[1] = -1;
            V[2] = 0;
            set_velocity();
//            motor(2, -11000);
//            motor(3, 11000);
        }
        for (i = 100; i >= 0; i--)
        {
            V[0] = 0;
            V[1] = -(float) i / 100;
            V[2] = 0;
            set_velocity();
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
            V[0] = -1 * (float) i / 100;
            ;
            V[1] = 0;
            V[2] = 0;
            set_velocity();
            delayMs(1);
        }
        prev_ms = millis();
        previous = position.x;
        current = previous;
        while ((current - previous) < distance)
        {
            if ((millis() - prev_ms) > CORRECTION_TIME)
            {
                correct_orientation();
                prev_ms = millis();
            }
            current = position.x;
            V[0] = -1;
            V[1] = 0;
            V[2] = 0;
            set_velocity();
//            motor(0, -11000);
//            motor(1, 11000);
        }
        for (i = 100; i >= 0; i--)
        {
            V[0] = -1 * (float) i / 100;
            ;
            V[1] = 0;
            V[2] = 0;
            set_velocity();
            delayMs(1);
        }
    }
        break;
    case -2:
    {
        for (i = 0; i <= 100; i++)
        {
            V[0] = 1 * (float) i / 100;
            ;
            V[1] = 0;
            V[2] = 0;
            set_velocity();
            delayMs(1);
        }
        prev_ms = millis();
        previous = position.x;
        current = previous;
        while ((previous - current) < distance)
        {
            if ((millis() - prev_ms) > CORRECTION_TIME)
            {
                correct_orientation();
                prev_ms = millis();
            }
            current = position.x;
            V[0] = 1;
            V[1] = 0;
            V[2] = 0;
            set_velocity();
            delayMs(1);
//            motor(0, 11000);
//            motor(1, -11000);
        }
        for (i = 100; i >= 0; i--)
        {
            V[0] = 1 * (float) i / 100;
            V[1] = 0;
            V[2] = 0;
            set_velocity();
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

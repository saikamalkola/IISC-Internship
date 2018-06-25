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

#define CORRECTION_TIME 5000L
//#include "FreeRTOS.h"
//#include "task.h"
//#include "queue.h"
//#include "semphr.h"

//Function definitions
void det_dis_dir();
void WaitForMsg();
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

volatile float dis = 0.0;
volatile int dir = 1;
volatile float dis_PID = 0, dis_P = 0, dis_I = 0, dis_D = 0;
volatile float dis_Kp = 0.003, dis_Ki = 0, dis_Kd = 0;

extern struct Position
{
    volatile float x;
    volatile float y;
    volatile float theta;
} position;

struct location
{
    float distance; //Distance between initial point and origin
    uint8_t side;   //Left - 0 Right - 1
};

struct location room[5] = { { 8000, 0 }, { 16000, 1 }, { 24000, 2 },
                            { 30000, 3 }, { 40000, 4 } };

struct location present = { 0, 0 }; //Initial Postion = 0 (Embedded Systems Lab)
struct location destination = { 40000, 0 };

int present_room = -1, dest_room = -1;  //Index of Lab

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
    PCA9685_digitalWrite(GREEN_LED, 1);
    delayMs(500);
    PCA9685_digitalWrite(GREEN_LED, 0);
    delayMs(500);

    while(1)
    {

    }
    while (1)
    {
        if (present_room != dest_room)
        {
            det_dis_dir();
            move(dis, dir);
            if (destination.side == 0)
            {
                //left side Beep once
                PCA9685_digitalWrite(BUZZER, 1);
                delayMs(500);
                PCA9685_digitalWrite(BUZZER, 0);
                delayMs(500);
            }
            else
            {
                //left side Beep once
                PCA9685_digitalWrite(BUZZER, 1);
                delayMs(250);
                PCA9685_digitalWrite(BUZZER, 0);
                delayMs(250);
                PCA9685_digitalWrite(BUZZER, 1);
                delayMs(250);
                PCA9685_digitalWrite(BUZZER, 0);
                delayMs(250);
            }
            present_room = dest_room;
        }
        //WaitForMsg();   //Wait for person to press button in UI
    }
}

int correction_range_check()
{
    if ((present.distance > 10000.0) && (present.distance < 35000.0))
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

void WaitForMsg()
{
    char response[2];
    if (UI_SerialAvailable() > 0)
    {
        //Read Line and Parse data
        UI_read_line(response);
        print_line(response);
        UART_OutChar('\t');
        dest_room = response[0] - '0';
        UART_OutChar(response[0]);
        UART_OutChar('\n');
        destination.distance = room[dest_room].distance;
        destination.side = room[dest_room].side;
    }
}

void det_dis_dir()
{
    if ((destination.distance - present.distance) > 0.0)
    {
        dir = 1;
        dis = destination.distance - present.distance;
    }
    else
    {
        dir = -1;
        dis = present.distance - destination.distance;
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
    while ((millis() - prev_ms) < 2000)
    {
        int i = 0;
        error[1] = distance[0] - distance[1];
        error[2] = distance[2] - distance[3];
        if ((distance[0] + distance[1]) > 2000)
        {
            if (distance[0] > 900 && distance[1] > 900)
            {
                error[1] = 0;
            }
            if (distance[0] > distance[1])
            {
                error[1] = 890 - distance[1];
            }
            else
            {
                error[1] = distance[0] - 890;
            }
        }
        if ((distance[2] + distance[3]) > 2000)
        {
            if (distance[2] > 900 && distance[3] > 900)
            {
                error[2] = 0;
            }
            if (distance[2] > distance[3])
            {
                error[2] = 890 - distance[3];
            }
            else
            {
                error[2] = distance[2] - 890;
            }
        }
        for (i = 0; i < 4; i++)
        {
            if (distance[i] == -1)
            {
                distance[i] = 890;
            }
        }
        switch (abs(dir))
        {
        case 1:
        {
            dis_P = dis_Kp * error[2];
            dis_PID = dis_P + dis_D;
            if (abs(dis_PID) > 1)
            {
                dis_PID = sign(dis_PID) * 1.5;
            }
            V[0] = -dis_PID;
            V[1] = 0;
            V[2] = 0;
            set_velocity();
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
            set_velocity();
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
            if ((millis() - prev_ms) > CORRECTION_TIME)
            {
                if (correction_range_check())
                {
                    correct_orientation();
                }
                else
                {
                    PCA9685_digitalWrite(BUZZER, 1);
                    delayMs(100);
                    PCA9685_digitalWrite(BUZZER, 0);
                    delayMs(100);
                }
                prev_ms = millis();
            }
            current = position.y;
            V[0] = 0;
            V[1] = 1.0;
            V[2] = 0;
            set_velocity();
        }
        for (i = 100; i >= 0; i--)
        {
            V[0] = 0;
            V[1] = (float) i / 100;
            V[2] = 0;
            set_velocity();
            delayMs(1);
        }
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
                if (correction_range_check())
                {
                    correct_orientation();
                }
                else
                {
                    PCA9685_digitalWrite(BUZZER, 1);
                    delayMs(100);
                    PCA9685_digitalWrite(BUZZER, 0);
                    delayMs(100);
                }
                prev_ms = millis();
            }
            current = position.y;
            V[0] = 0;
            V[1] = -1;
            V[2] = 0;
            set_velocity();
        }
        for (i = 100; i >= 0; i--)
        {
            V[0] = 0;
            V[1] = -(float) i / 100;
            V[2] = 0;
            set_velocity();
            delayMs(1);
        }
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

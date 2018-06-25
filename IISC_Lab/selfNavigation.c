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

void set_dest_corridor(void);
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

float limit_L = 0, limit_H = 0;
uint8_t change_corridor = 0;

extern struct Position
{
    volatile float x;
    volatile float y;
    volatile float theta;
} position;

struct location
{
    float distance_x; //Distance between initial point and origin
    float distance_y;
    uint8_t side;   //Left - 0 Right - 1
};

struct location room[7] = { { 0, 0, 0 }, { 0, 8000, 0 }, { 0, 16000, 1 }, {
        0, 24000, 1 },
                            { 0, 32000, 0 }, { 0, 40000, 1 }, { 10000, 0, 0 } };

struct location present = { 0, 0, 0 }; //Initial Postion = 0 (Embedded Systems Lab)
struct location destination = { 0, 0, 0 };

int present_room = -1, dest_room = -1;  //Index of Lab
uint8_t present_corridor = 0, dest_corridor = 0;
/*
 *  00 - Origin
 *  01 - On X - axis
 *  10 - On Y - axis
 *  11 - not on X and Y axes
 */

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
    PCA9685_digitalWrite(GREEN_LED, 1);
    delayMs(500);
    PCA9685_digitalWrite(GREEN_LED, 0);
    delayMs(500);
    while (1)
    {
        while (present_room != dest_room)
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
            if (change_corridor == 0)
            {
                present_room = dest_room;
            }
            present_corridor = dest_corridor;
        }
        WaitForMsg();   //Wait for person to press button in UI
    }
}

int correction_range_check()
{
    float check_distance = 0;

    if (abs(dir) == 1)
    {
        limit_L = 10000;
        limit_H = 35000;
        check_distance = present.distance_y;
    }
    else if (abs(dir) == 2)
    {
        limit_L = 6000;
        limit_H = 12000;
        check_distance = present.distance_x;
    }

    if ((abs(check_distance) > limit_L) && (abs(check_distance) < limit_H))
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
        dest_room = response[0] - '0';
        destination.distance_x = room[dest_room].distance_x;
        destination.distance_y = room[dest_room].distance_y;
        destination.side = room[dest_room].side;
        set_dest_corridor();
    }
}

void set_dest_corridor()
{
    if (destination.distance_x != 0)
    {
        dest_corridor |= 0x01;
    }
    else
    {
        dest_corridor &= ~(0x01);
    }

    if (destination.distance_y != 0)
    {
        dest_corridor |= 0x02;
    }
    else
    {
        dest_corridor &= ~(0x02);
    }
}

void det_dis_dir()
{
    if (destination.distance_x == 0)
    {
        if ((present_corridor & 0x02) || (present_corridor == 0))
        {
            change_corridor = 0;
            //Bot is there on Y-Axis or origin
            if ((destination.distance_y - present.distance_y) > 0.0)
            {
                dir = 1;
                dis = destination.distance_y - present.distance_y;
            }
            else
            {
                dir = -1;
                dis = present.distance_y - destination.distance_y;
            }
        }
        else if ((present_corridor & 0x01))
        {
            change_corridor = 1;
            //Bot is there on X-axis but need to go to room on Y-axis
            if (present.distance_x > 0)
            {
                dir = -2;
                dis = present.distance_x;
            }
            else
            {
                dir = 2;
                dis = -present.distance_x;
            }
        }
    }
    else if (destination.distance_y == 0)
    {
        if ((present_corridor & 0x01) || (present_corridor == 0))
        {
            change_corridor = 0;
            //Bot is there on X-Axis or origin
            if ((destination.distance_x - present.distance_x) > 0.0)
            {
                dir = 2;
                dis = destination.distance_x - present.distance_x;
            }
            else
            {
                dir = -2;
                dis = present.distance_x - destination.distance_x;
            }
        }
        else if ((present_corridor & 0x02))
        {
            change_corridor = 1;
            //Bot is there on X-axis but need to go to room on Y-axis
            if (present.distance_y > 0)
            {
                dir = -1;
                dis = present.distance_y;
            }
            else
            {
                dir = 1;
                dis = -1 * present.distance_y;
            }
        }
    }
}

void correct_orientation()
{
    float error[3] = { 0, 0, 0 };
    unsigned long prev_ms = millis(), Kp = 0.7;

    while ((millis() - prev_ms) < 500)
    {
        motor(0, 0);
        motor(1, 0);
        motor(2, 0);
        motor(3, 0);
    }
//
//    prev_ms = millis();
//    while ((millis() - prev_ms) < 2000)
//    {
//        error[2] = (-1 * position.theta * 180 / 3.14);
//        V[0] = 0;
//        V[1] = 0;
//        V[2] = Kp * error[2];
//        set_velocity();
//        delayMs(10);
//    }

    prev_ms = millis();
    while ((millis() - prev_ms) < 2000)
    {
        int i = 0;
        for (i = 0; i < 4; i++)
        {
            if (distance[i] == -1)
            {
                distance[i] = 890;
            }
        }

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
    case -2:
    {
        for (i = 0; i <= 100; i++)
        {
            V[0] = -1 * (float) i / 100;
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
            V[0] = -1;
            V[1] = 0;
            V[2] = 0;
            set_velocity();
        }
        for (i = 100; i >= 0; i--)
        {
            V[0] = -1 * (float) i / 100;
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

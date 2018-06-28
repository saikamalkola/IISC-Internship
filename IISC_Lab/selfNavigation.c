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
#include "MPU9250_YawCalc.h"

#define X_MAX   20500
#define Y_MAX   49750
#define RAMP_COMP 300

#define CORRECTION_TIME 5000L

//#include "FreeRTOS.h"
//#include "task.h"
//#include "queue.h"
//#include "semphr.h"

//Function definitions
void obstacle_avoidance();
void buzz(int ms);
void WaitForMsg();
void delayMs(int n);
void DisableInterrupts(void);
void EnableInterrupts(void);

extern volatile long encoder_value[4];
extern volatile int lastEncoded[4];
extern volatile int velocity[4];
extern volatile int velocity_error[4], PID[4];

uint8_t det_corridor(int room_number);
void make_decision_array();
uint8_t det_common_corner(uint8_t present_corridor, uint8_t dest_corridor);
void det_common_corners(uint8_t *comm_corner, uint8_t present_corridor,
                        uint8_t dest_corridor);
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


float Kp = 0.5;

extern volatile float yaw;

float limit_L = 0, limit_H = 0;
uint8_t change_corridor = 0;

uint8_t obstacle = 0;

volatile uint8_t ultra_correct = 0;

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

struct location common_corner[4] = { { 0, 0, 0 }, { X_MAX, 0, 0 }, { X_MAX,
Y_MAX,
                                                                     0 },
                                     { 0, Y_MAX, 0 } };
struct location room[43] = { { 0, 0, 0 }, { 0, 5000, 0 }, { 0, 8750, 1 }, {
        0, 12250, 0 },
                             { 0, 12250, 1 }, { 0, 13750, 0 }, { 0, 19500, 1 },
                             { 0, 26250, 0 }, { 0, 28000, 0 }, { 0, 32000, 1 },
                             { 0, 35250, 0 }, { 0, 37250, 1 }, { 0, 38750, 1 },
                             { 0, 42500, 1 }, { 0, 48250, 1 }, { 0, Y_MAX, 1 },
                             { 3250, Y_MAX, 0 }, { 9000, Y_MAX, 1 },
                             { 10750, Y_MAX, 1 }, { 12500, Y_MAX, 0 }, { 16250,
                                                                         Y_MAX,
                                                                         0 },
                             { X_MAX, Y_MAX, 0 }, { X_MAX, 43250, 0 }, { X_MAX,
                                                                         40500,
                                                                         0 },
                             { X_MAX, 38750, 1 }, { X_MAX, 35000, 1 }, { X_MAX,
                                                                         30700,
                                                                         0 },
                             { X_MAX, 29750, 0 }, { X_MAX, 23250, 0 }, { X_MAX,
                                                                         21750,
                                                                         0 },
                             { X_MAX, 18250, 1 }, { X_MAX, 15500, 0 }, { X_MAX,
                                                                         14500,
                                                                         0 },
                             { X_MAX, 12500, 1 }, { X_MAX, 11000, 1 },
                             { X_MAX, 5250, 1 }, { X_MAX, 3875, 0 }, { X_MAX, 0,
                                                                       0 },
                             { 19000, 0, 0 }, { 11750, 0, 1 }, { 10000, 0, 1 },
                             { 6000, 0, 0 }, { 2500, 0, 0 } };

struct location present = { 0, 0, 0 }; //Initial Postion = 0 (Embedded Systems Lab)
struct location destination = { 0, 0, 0 };

int present_room = 0, dest_room = 0;  //Index of Lab

/*
 * (50,0)      (50,20)
 *  _______________
 *  |      3      |            Y^
 *  |             |             |
 *  |             |             |
 *  |             |             |
 *  |0           2|             |
 *  |             |             ---------->X
 *  |             |
 *  |_____________|
 *  (0,0)  1   (20, 0)
 *
 */

struct decision
{
    float distance;
    int dir;
} decisions[3];

extern uint8_t yaw_calib;
uint8_t decision_count = 0;

int main()
{
    DisableInterrupts();
    Ultrasonic_Init();
    init_encoders();    //Initializing Encoder Pins
    init_motors();  //Initializing Motor Pins
    UART_Init();    //Initialising UART
    UART1_Init();
    //init_BMS();
    init_I2C1();
    init_PCA9685();
    init_timer0A(25);
    init_timer1A();
    init_timer2A(50);

    init_I2C0();
    init_MPU9250();
    init_timer0B(50);    //Initialize timer to trigger interrup every 4ms

    TIMER0_CTL_R |= TBEN;    //Enabling Timer As
    EnableInterrupts();

    //Green Signal indicating initialization routines executed successfully

    while(!yaw_calib);

    PCA9685_digitalWrite(GREEN_LED, 1);
    delayMs(500);
    PCA9685_digitalWrite(GREEN_LED, 0);
    delayMs(500);

    while (1)
    {
        while (present_room != dest_room)
        {
            int i = 0;
            //Execute the instructions
            for (i = 0; i <= decision_count; i++)
            {
                dir = decisions[i].dir;
                move(decisions[i].distance, decisions[i].dir);
            }

            present_room = dest_room;
            if (room[dest_room].side == 0)
            {
                //left side Beep once
                buzz(500);
            }
            else
            {
                //Right side Beep Twice
                buzz(250);
                buzz(250);
            }
        }
        WaitForMsg();   //Wait for person to press button in UI
    }
}

void buzz(int ms)
{
    PCA9685_digitalWrite(BUZZER, 1);
    delayMs(ms);
    PCA9685_digitalWrite(BUZZER, 0);
    delayMs(ms);
}

int correction_range_check()
{
    float check_distance = 0;

    if (abs(dir) == 1)
    {
        limit_L = 5000;
        limit_H = 42000;
        check_distance = present.distance_y;
    }
    else if (abs(dir) == 2)
    {
        limit_L = 4000;
        limit_H = 16000;
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

uint8_t det_common_corner(uint8_t present_corridor, uint8_t dest_corridor)
{
    uint8_t common;

    uint8_t sum = present_corridor + dest_corridor;
    switch (sum)
    {
    case 1:
    {
        common = 0;
    }
        break;
    case 3:
    {
        common = 1;
    }
        break;
    case 5:
    {
        common = 2;
    }
        break;
    case 7:
    {
        common = 3;
    }
        break;
    }
    return common;
}

void det_common_corners(uint8_t *comm_corner, uint8_t present_corridor,
                        uint8_t dest_corridor)
{
    float d1 = 0, d2 = 0;
    if ((present_corridor % 2) == 0)
    {
        d1 = X_MAX + room[present_room].distance_y + room[dest_room].distance_y;
        d2 = X_MAX + (Y_MAX - room[present_room].distance_y)
                + (Y_MAX - room[dest_room].distance_y);

        if (d1 > d2)
        {
            if (present_corridor > dest_corridor)
            {
                comm_corner[0] = 2;
                comm_corner[1] = 3;
            }
            else
            {
                comm_corner[0] = 3;
                comm_corner[1] = 2;
            }
        }
        else
        {
            if (present_corridor > dest_corridor)
            {
                comm_corner[0] = 1;
                comm_corner[1] = 0;
            }
            else
            {
                comm_corner[0] = 0;
                comm_corner[1] = 1;
            }
        }
    }
    else if ((present_corridor % 2) == 1)
    {
        d1 = Y_MAX + room[present_room].distance_x + room[dest_room].distance_x;
        d2 = Y_MAX + (X_MAX - room[present_room].distance_x)
                + (X_MAX - room[dest_room].distance_x);

        if (d1 > d2)
        {
            if (present_corridor > dest_corridor)
            {
                comm_corner[0] = 2;
                comm_corner[1] = 1;
            }
            else
            {
                comm_corner[0] = 1;
                comm_corner[1] = 2;
            }
        }
        else
        {
            if (present_corridor > dest_corridor)
            {
                comm_corner[0] = 3;
                comm_corner[1] = 0;
            }
            else
            {
                comm_corner[0] = 0;
                comm_corner[1] = 3;
            }
        }
    }
}

void make_decision_array()
{
    uint8_t present_corridor = 0, dest_corridor = 0;
    //determine in which corridor bot is present
    present_corridor = det_corridor(present_room);
    //determine in which corridor destination co-ordinate is there
    dest_corridor = det_corridor(dest_room);

    if ((present_corridor == 0 && dest_corridor == 3))
    {
        present_corridor = 4;
    }

    if ((present_corridor == 3 && dest_corridor == 0))
    {
        dest_corridor = 4;
    }

    decision_count = 0;
    if (dest_corridor == present_corridor)
    {
        /*
         * Simple cases - Robot is in same corridor as destination corridor
         *
         * Cases:       P  D
         *              0  0
         *              1  1
         *              2  2
         *              3  3
         */
        if (room[present_room].distance_x == room[dest_room].distance_x)
        {
            decisions[decision_count].distance = abs(
                    room[dest_room].distance_y - room[present_room].distance_y);
            decisions[decision_count].dir = sign(
                    room[dest_room].distance_y - room[present_room].distance_y);
        }
        if (room[present_room].distance_y == room[dest_room].distance_y)
        {
            decisions[decision_count].distance = abs(
                    room[dest_room].distance_x - room[present_room].distance_x);
            decisions[decision_count].dir = 2
                    * sign(room[dest_room].distance_x
                            - room[present_room].distance_x);
        }
    }
    else if (abs(dest_corridor - present_corridor) == 1)
    {
        /*
         * Little Complex cases - When robot is in corridor adjacent to the destination corridor
         *
         * First go to corner which is common to both the corridors and go to destination corridor from there
         *
         *  Cases       P  D
         *              0  1
         *              1  0
         *  Common:    (0,0)
         *
         *              1  2
         *              2  1
         *  Common:    (X_MAX,0)
         *
         *              2  3
         *              3  2
         *  Common:    (X_MAX,Y_MAX)
         *
         *              3  0
         *              0  3
         *  Common:    (0,Y_MAX)
         */
        //First decision go to the common corner
        uint8_t common = det_common_corner(present_corridor, dest_corridor);
        if (present_corridor % 2 == 0)
        {
            // Bot is on X = 0 or X = 20
            decisions[decision_count].distance = abs(
                    common_corner[common].distance_y
                            - room[present_room].distance_y);
            decisions[decision_count].dir = sign(
                    common_corner[common].distance_y
                            - room[present_room].distance_y);

            decision_count++;

            //Now bot will be in common corner
            decisions[decision_count].distance = abs(
                    room[dest_room].distance_x
                            - common_corner[common].distance_x);
            decisions[decision_count].dir = 2
                    * sign(room[dest_room].distance_x
                            - common_corner[common].distance_x);
        }
        else if (present_corridor % 2 == 1)
        {
            //Bot is on Y = 0 or Y = 50
            decisions[decision_count].distance = abs(
                    common_corner[common].distance_x
                            - room[present_room].distance_x);
            decisions[decision_count].dir = 2
                    * sign(common_corner[common].distance_x
                            - room[present_room].distance_x);

            decision_count++;

            //Now bot will be in common corner
            decisions[decision_count].distance = abs(
                    room[dest_room].distance_y
                            - common_corner[common].distance_y);
            decisions[decision_count].dir = sign(
                    room[dest_room].distance_y
                            - common_corner[common].distance_y);
        }
    }

    else if (abs(dest_corridor - present_corridor) == 2)
    {
        /*
         *  Complex cases - When robot is in corridor parallel to the destination corridor
         *
         *  Calculate shortest path and make decision array accordingly
         *
         *  Cases       P   D
         *              0   2
         *              2   0
         *
         *              1   3
         *              3   1
         */

        uint8_t comm_corner[2];
        //Determine common corner based on shortest path
        det_common_corners(comm_corner, present_corridor, dest_corridor);
        if (present_corridor % 2 == 0)
        {
            // Bot is on X = 0 or X = 20
            decisions[decision_count].distance = abs(
                    common_corner[comm_corner[0]].distance_y
                            - room[present_room].distance_y);
            decisions[decision_count].dir = sign(
                    common_corner[comm_corner[0]].distance_y
                            - room[present_room].distance_y);

            decision_count++;

            //Now bot will be in 1st common corner
            decisions[decision_count].distance = abs(
                    common_corner[comm_corner[1]].distance_x
                            - common_corner[comm_corner[0]].distance_x);
            decisions[decision_count].dir = 2
                    * sign(common_corner[comm_corner[1]].distance_x
                            - common_corner[comm_corner[0]].distance_x);

            decision_count++;
            //Now bot will be in 2nd common corner
            decisions[decision_count].distance = abs(
                    room[dest_room].distance_y
                            - common_corner[comm_corner[1]].distance_y);
            decisions[decision_count].dir = sign(
                    room[dest_room].distance_y
                            - common_corner[comm_corner[1]].distance_y);

        }
        else if (present_corridor % 2 == 1)
        {
            //Bot is on Y = 0 or Y = 50
            decisions[decision_count].distance = abs(
                    common_corner[comm_corner[0]].distance_x
                            - room[present_room].distance_x);
            decisions[decision_count].dir = sign(
                    common_corner[comm_corner[0]].distance_x
                            - room[present_room].distance_x);

            decision_count++;

            //Now bot will be in 1st common corner
            decisions[decision_count].distance = abs(
                    common_corner[comm_corner[1]].distance_y
                            - common_corner[comm_corner[0]].distance_y);
            decisions[decision_count].dir = sign(
                    common_corner[comm_corner[1]].distance_y
                            - common_corner[comm_corner[0]].distance_y);

            //Now bot will be in 2nd common corner
            decision_count++;
            decisions[decision_count].distance = abs(
                    common_corner[comm_corner[1]].distance_x
                            - room[present_room].distance_x);
            decisions[decision_count].dir = 2
                    * sign(common_corner[comm_corner[1]].distance_x
                            - room[present_room].distance_x);

        }

    }
}

void WaitForMsg()
{
    char response[3];
    if (UI_SerialAvailable() > 0)
    {
        //Read Line and Parse data
        UI_read_line(response);
        dest_room = response[0] - '0';
        dest_room = dest_room * 10;
        dest_room += (response[1] - '0');
        if (dest_room != present_room)
        {
            make_decision_array();
        }
    }
}

uint8_t det_corridor(int room_number)
{
    if (room[room_number].distance_x == 0)
    {
        //Bot is in corridor 0
        return 0;
    }

    if (room[room_number].distance_y == 0)
    {
        //Bot is in corridor 1
        return 1;
    }

    if (room[room_number].distance_x == X_MAX)
    {
        //Bot is in corridor 2
        return 2;
    }

    if (room[room_number].distance_y == Y_MAX)
    {
        //Bot is in corridor 3
        return 3;
    }

    return 0;
}

void correct_yaw()
{
    float yaw_error = yaw;
    V[0] = 0;
    V[1] = 0;
    unsigned long prev_ms = millis();
    while ((millis() - prev_ms) < 2000)
    {
        yaw_error = yaw;
        if (abs(yaw_error) < 3)
        {
            V[2] = 0;
        }
        else
        {
            V[2] = Kp * yaw_error;
        }
        set_velocity();
        delayMs(10);
    }
    motor(0, 0);
    motor(1, 0);
    motor(2, 0);
    motor(3, 0);
}
void correct_orientation()
{
    ultra_correct = 1;
    float error[3] = { 0, 0, 0 };
    unsigned long prev_ms = millis();

    while ((millis() - prev_ms) < 500)
    {
        motor(0, 0);
        motor(1, 0);
        motor(2, 0);
        motor(3, 0);
    }

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
                dis_PID = sign(dis_PID) * 1;
            }
            V[0] = -dis_PID;
            V[1] = 0;
            //            set_motor(0, dis_PID);
            //            set_motor(1, -dis_PID);
        }
            break;
        case 2:
        {
            dis_P = dis_Kp * error[1];
            dis_PID = dis_P + dis_D;
            if (abs(dis_PID) > 1)
            {
                dis_PID = sign(dis_PID) * 1;
            }
            V[0] = 0;
            V[1] = dis_PID;
        }
            break;
        }
    }
    ultra_correct = 0;
}

void obstacle_avoidance()
{
    if (ultrasonic(dir) > 50 && ultrasonic(dir) < OBSTACLE_DIS_THRESH)
    {
//        while(ultrasonic(dir) < OBSTACLE_DIS_THRESH)
//        {
//            buzz(100);
//            brake();
//        }
//        no_brake();
    }
}

void move(float distance, int dir)
{
    distance -= RAMP_COMP;
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
//            V[2] = 0;
//            set_velocity();
            delayMs(1);
        }
        previous = position.y;
        current = previous;
        prev_ms = millis();
        while ((current - previous) < distance)
        {
            //obstacle_avoidance();
            if ((millis() - prev_ms) > CORRECTION_TIME)
            {
                //correct_yaw();
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
//            V[2] = 0;
//            set_velocity();
        }
        for (i = 100; i >= 0; i--)
        {
            V[0] = 0;
            V[1] = (float) i / 100;
//            V[2] = 0;
//            set_velocity();
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
//            V[2] = 0;
//            set_velocity();
            delayMs(1);
        }
        previous = position.y;
        current = previous;
        prev_ms = millis();
        while ((previous - current) < distance)
        {
            //obstacle_avoidance();
            if ((millis() - prev_ms) > CORRECTION_TIME)
            {
                //correct_yaw();
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
            V[1] = -1.0;
//            V[2] = 0;
//            set_velocity();
        }
        for (i = 100; i >= 0; i--)
        {
            V[0] = 0;
            V[1] = -(float) i / 100;
//            V[2] = 0;
//            set_velocity();
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
//            V[2] = 0;
//            set_velocity();
            delayMs(1);
        }
        prev_ms = millis();
        previous = position.x;
        current = previous;
        while ((current - previous) < distance)
        {
            //obstacle_avoidance();
            if ((millis() - prev_ms) > CORRECTION_TIME)
            {
                //correct_yaw();
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
            V[0] = 1.0;
            V[1] = 0;
//            V[2] = 0;
//            set_velocity();
        }
        for (i = 80; i >= 0; i--)
        {
            V[0] = 1 * (float) i / 100;
            V[1] = 0;
//            V[2] = 0;
//            set_velocity();
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
            //obstacle_avoidance();
            if ((millis() - prev_ms) > CORRECTION_TIME)
            {
                //correct_yaw();
                correct_orientation();
                prev_ms = millis();
            }
            current = position.x;
            V[0] = -1.0;
            V[1] = 0;
//            V[2] = 0;
//            set_velocity();
        }
        for (i = 100; i >= 0; i--)
        {
            V[0] = -1 * (float) i / 100;
            V[1] = 0;
//            V[2] = 0;
//            set_velocity();
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

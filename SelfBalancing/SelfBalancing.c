/*
 * SelfBalancing.c
 *
 *  Created on: 21-Jun-2018
 *      Author: kshama
 */

#include <stdint.h>
#include "inc/tm4c123gh6pm.h"
#include "motors.h"
#include "MPU9250.h"
#include "UartSerial.h"

extern volatile uint8_t Buf[14];
extern volatile int16_t ax, ay, az, gx, gy, gz;

void delayMs(int n);
void DisableInterrupts(void);
void EnableInterrupts(void);

extern volatile float set_point;
extern volatile float PID, dt, gyro_pitch, accel_pitch, pitch;

int main(void)
{
    DisableInterrupts();
    init_I2C0();
    UART_Init();
    init_MPU9250();
    init_motors();
    init_timer0A(2);
    init_timer1A();
    TIMER0_CTL_R |= TAEN;    //Enabling Timer A
    EnableInterrupts();
    while (1)
    {
//        SerialPrintInt(dt * 1000000);
//        UART_OutChar('\t');
//        SerialPrintInt(PID);

        SerialPrintInt(100);
        UART_OutChar('\t');
        SerialPrintInt(-100);
        UART_OutChar('\t');
        SerialPrintInt(dt*1000000);
        UART_OutChar('\t');
        SerialPrintInt(pitch);
        UART_OutChar('\t');
//        SerialPrintInt(accel_pitch);
//        UART_OutChar('\t');
//        SerialPrintInt(gyro_pitch);
        UART_OutChar('\n');
        UART_OutChar(CR);

    }
}

void delayMs(int n)
{
    int i = 0, j = 0;
    for (i = 0; i < n; i++)
    {
        for (j = 0; j < 3180; j++)
        {
            //Do Nothing
        }
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

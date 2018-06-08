/*
 * MPU9250.c
 *
 *  Created on: 07-Jun-2018
 *      Author: kamal
 */

#include <stdint.h>
#include <stdio.h>
#include "inc/tm4c123gh6pm.h"

#define    MPU9250_ADDRESS            0x68
#define    MAG_ADDRESS                0x0C

#define    GYRO_FULL_SCALE_250_DPS    0x00
#define    GYRO_FULL_SCALE_500_DPS    0x08
#define    GYRO_FULL_SCALE_1000_DPS   0x10
#define    GYRO_FULL_SCALE_2000_DPS   0x18

#define    ACC_FULL_SCALE_2_G        0x00
#define    ACC_FULL_SCALE_4_G        0x08
#define    ACC_FULL_SCALE_8_G        0x10
#define    ACC_FULL_SCALE_16_G       0x18

#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_XOUT_H 0x3B

uint8_t Buf[14];
int16_t ax,ay,az,gx,gy,gz;

void setRWMode(uint8_t mode)
{
    switch (mode)
    {
    case 0:
        I2C0_MSA_R &= ~(0x01);  //TX mode
        break;
    case 1:
        I2C0_MSA_R |= 0x01; //RX mode
        break;
    }
}

void waitAndErrorCheck()
{
    while (I2C0_MCS_R & 0x01)
        ; //Controller is busy..Wait here

    //Check for Error
    if (I2C0_MCS_R & 0x02)
    {
        //Error bit is set
        if (I2C0_MCS_R & 0x10)
        {
            //I2C Controller lost Arbitration
        }
        else
        {
            //I2C Controller won Arbitration
            I2C0_MCS_R = (1 << 2);  //Send STOP bit
            while (I2C0_MCS_R & 0x01)
                ;   //Controller is busy..Wait here
        }
    }
}

void wireRead(uint8_t *data, uint8_t num_bytes)
{
    setRWMode(1);
    uint8_t i = 0;
    I2C0_MCS_R = ((1 << 0) | (1 << 1) | (1 << 3)); //Configures START, RUN, STOP and DataAck bits
    /*
     *  DACK-STOP-START-RUN
     *    3    2    1    0
     */
    for (i = 0; i < (num_bytes - 1); i++)
    {
        //Wait and Check for Error
        waitAndErrorCheck();
        data[i] = I2C0_MDR_R & 0xFF;
        I2C0_MCS_R = ((1 << 0) | (1 << 3)); //Configures START, RUN, STOP and DataAck bits
    }
    I2C0_MCS_R = ((1 << 0) | (1 << 2));
    waitAndErrorCheck();
    data[num_bytes - 1] = I2C0_MDR_R & 0xFF;
}

void wireSend(uint8_t byte, uint8_t condition)
{
    setRWMode(0);
    I2C0_MDR_R = byte;  //Write tx byte to the register
    I2C0_MCS_R = condition; //Configures START, RUN, STOP and DataAck bits

    /*
     *  DACK-STOP-START-RUN
     *    3    2    1    0
     */
    while (I2C0_MCS_R & 0x01)
        ; //Controller is busy..Wait here

    //Wait and Check for Error
    waitAndErrorCheck();
}

void I2CwriteByte(uint8_t Register, uint8_t Data)
{
    wireSend(Register, (1 << 0) | (1 << 1));
    wireSend(Data, (1 << 0) | (1 << 2));
}

void setSlaveAddress(uint8_t address)
{
    I2C0_MSA_R = (address << 1);
}



void init_MPU9250()
{
    setSlaveAddress(MPU9250_ADDRESS);  //Setting slave address
    I2CwriteByte(GYRO_CONFIG, GYRO_FULL_SCALE_500_DPS);
    I2CwriteByte(ACCEL_CONFIG, ACC_FULL_SCALE_4_G);
}

void init_I2C0()
{
    SYSCTL_RCGCI2C_R |= (0x01); //Enabling clock to I2C0 module
    SYSCTL_RCGC2_R |= 0x02; //Enable clock to port B
    GPIO_PORTB_LOCK_R = 0x4C4F434B; //Unlock Port B
    GPIO_PORTB_AFSEL_R |= ((1 << 2) | (1 << 3)); //Enabling Alternate Functionality for pins PB2 and PB3
    GPIO_PORTB_DEN_R |= ((1 << 2) | (1 << 3)); //Enabling Digital functionality of pins PB2 and PB3
    GPIO_PORTB_ODR_R |= (1 << 3);   //Enabling Open drain for SDA pin
    GPIO_PORTB_PCTL_R &= ~((0xF << 8) | (0xF << 12));
    GPIO_PORTB_PCTL_R |= ((3 << 8) | (3 << 12)); //Configuring I2C PB2 and PB3 as I2C pins

    I2C0_MCR_R |= (1 << 4); //Configuring I2C0 module as MASTER
    I2C0_MTPR_R = 0x07; //Configuring SCL of I2C0 to run at 400KHz
}

void main()
{
    init_I2C0();
    init_MPU9250();
    int i = 0;
    while (1)
    {
        wireSend(ACCEL_XOUT_H, (1 << 0) | (1 << 1));
        wireRead(Buf,14);
        // Accelerometer
        ax=(Buf[0]<<8 | Buf[1]);
        ay=(Buf[2]<<8 | Buf[3]);
        az=Buf[4]<<8 | Buf[5];

        // Gyroscope
        gx=(Buf[8]<<8 | Buf[9]);
        gy=(Buf[10]<<8 | Buf[11]);
        gz=Buf[12]<<8 | Buf[13];

        for(i = 0; i < 318; i++);
    }
}

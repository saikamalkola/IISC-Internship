/*
 * PCA9685.c
 *
 *  Created on: 08-Jun-2018
 *      Author: kamal
 */

#include <stdint.h>
#include <stdio.h>
#include "inc/tm4c123gh6pm.h"

#define PCA9685_ADDRESS 0x40
#define MODE1_ADDRESS   0x00
#define MODE2_ADDRESS   0x01
#define PRESCALE_ADDRESS 0xFE
#define PRESCALER_1526HZ 0x03

#define LED0_ON_L 0x06

#define ON_LOW 0x00
#define ON_HIGH 0x01
#define OFF_LOW 0x02
#define OFF_HIGH 0x03

int i = 0,j = 0;

uint8_t debug = 5;

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

void wireRead(uint8_t address, uint8_t *data, uint8_t num_bytes)
{
    wireSend(address, (1 << 0) | (1 << 1));
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

void I2CwriteByte(uint8_t Register, uint8_t Data)
{
    wireSend(Register, (1 << 0) | (1 << 1));
    wireSend(Data, (1 << 0) | (1 << 2));
}

void setSlaveAddress(uint8_t address)
{
    I2C0_MSA_R = (address << 1);
}

void PCA9685_analogWrite(uint8_t pin, uint16_t on, uint16_t off)
{
    wireSend(LED0_ON_L + 4 * pin, (1 << 0) | (1 << 1));
    wireSend(on, (1 << 0));
    wireSend(on >> 8, (1 << 0));
    wireSend(off, (1 << 0));
    wireSend(off >> 8, (1 << 0) | (1 << 2));
}

void init_PCA9685()
{
    setSlaveAddress(PCA9685_ADDRESS);  //Setting slave address
    I2CwriteByte(MODE1_ADDRESS, 0x80);
    int i = 0;
    for (i = 0; i < 31800; i++)
        ;  //Wait for 10ms
    I2CwriteByte(MODE1_ADDRESS, 0x10);
    I2CwriteByte(PRESCALE_ADDRESS, PRESCALER_1526HZ);
    I2CwriteByte(MODE1_ADDRESS, 0xA0);
    for (i = 0; i < 31800; i++)
        ;  //Wait for 1ms
    wireRead(MODE1_ADDRESS, &debug, 1);
    //wireRead(MODE1_ADDRESS, &debug, 1);
}

void init_I2C0()
{
    SYSCTL_RCGCI2C_R |= (0x01); //Enabling clock to I2C0 module
    SYSCTL_RCGC2_R |= 0x02; //Enable clock to port B
    GPIO_PORTB_LOCK_R = 0x4C4F434B; //Unlock Port B
    GPIO_PORTB_AFSEL_R |= ((1 << 2) | (1 << 3)); //Enabling Alternate Functionality for pins PB2 and PB3
    GPIO_PORTB_DEN_R |= ((1 << 2) | (1 << 3)); //Enabling Digital functionality of pins PB2 and PB3
    GPIO_PORTB_ODR_R |= (1 << 3);   //Enabling Open drain for SDA pin
    GPIO_PORTB_PUR_R |= ((1 << 2) | (1 << 3));
    GPIO_PORTB_PCTL_R &= ~((0xF << 8) | (0xF << 12));
    GPIO_PORTB_PCTL_R |= ((3 << 8) | (3 << 12)); //Configuring I2C PB2 and PB3 as I2C pins

    I2C0_MCR_R |= (1 << 4); //Configuring I2C0 module as MASTER
    I2C0_MTPR_R = 0x01; //Configuring SCL of I2C0 to run at 400KHz
}

void main()
{
    init_I2C0();
    init_PCA9685();
    PCA9685_analogWrite(15,2048,4095);
    while (1)
    {
    }
}


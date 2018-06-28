/*
 * MPU9250_YAWCALC.c
 *
 *  Created on: 27-Jun-2018
 *      Author: kamal
 */

#include <stdbool.h>
#include <stdint.h>
#include "inc/tm4c123gh6pm.h"
#include "odometry.h"
#include "MPU9250_YawCalc.h"
#include "UartSerial.h"
#include "Ultrasonic.h"
#include "PCA9685.h"
#include <math.h>
#include "KinModel.h"

volatile unsigned long last_t = 0;

extern volatile uint8_t ultra_correct = 0;
uint8_t debug = 0;
uint8_t calib = 0, yaw_calib = 0;

volatile int count_g = 0, count_yaw = 0;

volatile long cal_gx = 0;
volatile float dt = 0;
volatile float gyro_yaw = 0;
volatile float mag_yaw = 0;
volatile float yaw = 0;
volatile long cal_yaw = 0;

volatile uint8_t flag = 0;

extern volatile float V[3];

void delayMs(int n);

void TIMER0_TB_Handler()
{
//    // Read register Status 1 and wait for the DRDY: Data Ready
    setSlaveAddress_I2C0(AK8963_ADDRESS);

    // Read magnetometer data
    uint8_t Mag[7]; //6 bytes for magnetometer readings and last byte for ST2 register
    wireRead_I2C0(AK8963_XOUT_L, Mag, 7);

    // Create 16 bits values from 8 bits data

    // Magnetometer
    int16_t my = -(Mag[1] << 8 | Mag[0]);
    int16_t mz = -(Mag[5] << 8 | Mag[4]);

    mag_yaw = (180 / 3.14) * atan2(my, mz);

    switch (yaw_calib)
    {
    case 0:
    {
        if (count_yaw < 16)
        {
            cal_yaw += mag_yaw;
            count_yaw++;
        }
        if (count_yaw == 16)
        {
            count_yaw++;
            cal_yaw = cal_yaw >> 4;
            yaw_calib = 1;
        }
    }
        break;
    case 1:
    {
        mag_yaw = mag_yaw - cal_yaw;
        yaw = 0.75 * yaw + 0.25 * mag_yaw;
        float yaw_error = yaw;
        if (abs(yaw_error) < 3 || ultra_correct)
        {
            V[2] = 0;
        }
        else
        {
            V[2] = 0.25 * yaw_error;
        }
        set_velocity();
    }
        break;
    }

    //yaw += (-1 * (float) gx * 0.05 / 131);

    volatile int read_back = 0;
    read_back = read_back;
    TIMER0_ICR_R = 0x100;
    read_back = TIMER0_ICR_R;
}

void init_MPU9250()
{
    setSlaveAddress_I2C0(MPU9250_ADDRESS);  //Setting slave address
    delayMs(10);
//Read Who AM I register
    printString("WHO_AM_I REGISTER VALUE: ");
    wireRead_I2C0(WHO_AM_I_MPU9250, &debug, 1);
    SerialPrintInt(debug);
    printString("\n");
    if (debug == 0x73)
    {
        printString("MPU9250 is online...\n");
    }
    else
    {
        printString("MPU9250 is not working properly...\n");
    }
    delayMs(10);
// wake up device
    I2CwriteByte_I2C0(PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
    delayMs(100); // Wait for all registers to reset

// get stable time source
    I2CwriteByte_I2C0(PWR_MGMT_1, 0x01); // Auto select clock source to be PLL gyroscope reference if ready else
    delayMs(200);

    I2CwriteByte_I2C0(CONFIG, 0x03); //Wake UP MPU9250
    delayMs(10);
    I2CwriteByte_I2C0(SMPLRT_DIV, 0x04);
    I2CwriteByte_I2C0(GYRO_CONFIG, GYRO_FULL_SCALE_500_DPS);
    I2CwriteByte_I2C0(ACCEL_CONFIG, ACC_FULL_SCALE_8_G);

// Set by pass mode for the magnetometers
    I2CwriteByte_I2C0(INT_PIN_CFG, 0x02);

// Request continuous magnetometer measurements in 16 bits
    setSlaveAddress_I2C0(AK8963_ADDRESS);  //Setting slave address
    delayMs(10);
    I2CwriteByte_I2C0(AK8963_CNTL, 0x16);
}

void setRWMode_I2C0(uint8_t mode)
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

void waitAndErrorCheck_I2C0()
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

void wireRead_I2C0(uint8_t address, uint8_t *data, uint8_t num_bytes)
{
    wireSend_I2C0(address, (1 << 0) | (1 << 1));
    setRWMode_I2C0(1);

    if (num_bytes == 1)
    {
        I2C0_MCS_R = ((1 << 0) | (1 << 1) | (1 << 2));
        waitAndErrorCheck_I2C0();
        *data = I2C0_MDR_R & 0xFF;
        return;
    }

    uint8_t i = 0;

    I2C0_MCS_R = ((1 << 0) | (1 << 1) | (1 << 3)); //Configures START, RUN, STOP and DataAck bits
    /*
     *  DACK-STOP-START-RUN
     *    3    2    1    0
     */
    for (i = 0; i < (num_bytes - 1); i++)
    {
        //Wait and Check for Error
        waitAndErrorCheck_I2C0();
        data[i] = I2C0_MDR_R & 0xFF;
        I2C0_MCS_R = ((1 << 0) | (1 << 3)); //Configures START, RUN, STOP and DataAck bits
    }
    I2C0_MCS_R = ((1 << 0) | (1 << 2));
    waitAndErrorCheck_I2C0();
    data[num_bytes - 1] = I2C0_MDR_R & 0xFF;
}

void wireSend_I2C0(uint8_t byte, uint8_t condition)
{
    setRWMode_I2C0(0);
    I2C0_MDR_R = byte;  //Write tx byte to the register
    I2C0_MCS_R = condition; //Configures START, RUN, STOP and DataAck bits

    /*
     *  DACK-STOP-START-RUN
     *    3    2    1    0
     */
    while (I2C0_MCS_R & 0x01)
        ; //Controller is busy..Wait here

//Wait and Check for Error
    waitAndErrorCheck_I2C0();
}

void I2CwriteByte_I2C0(uint8_t Register, uint8_t Data)
{
    wireSend_I2C0(Register, (1 << 0) | (1 << 1));
    wireSend_I2C0(Data, (1 << 0) | (1 << 2));
}

void setSlaveAddress_I2C0(uint8_t address)
{
    I2C0_MSA_R = (address << 1);
}

void init_timer0B(int time_ms)
{
    SYSCTL_RCGCTIMER_R |= 0x01;
    TIMER0_CTL_R &= ~(TBEN);    //Disabling Timer A
    TIMER0_CFG_R &= ~(0x07);    //Clearing D2:D0 bits
    TIMER0_CFG_R |= BIT16_MODE; //Configuring Timer 0 as 16 bit timer
    TIMER0_TBMR_R &= ~(0x03);   //Clearing D1:D0 bits
    TIMER0_TBMR_R |= PERIODIC;  //Periodic mode
    TIMER0_TBMR_R |= COUNT_DOWN;    //Count down mode
    uint32_t load_register = (time_ms * 16 * 1000 / 256) - 1; //Calculating ticks required at 16MHz
    TIMER0_TBPR_R = 0xFF;
    TIMER0_TBILR_R = load_register;
    TIMER0_ICR_R |= 0x100;

    TIMER0_IMR_R |= TBTORIM;    //Enabling Interrupt Timerout
    NVIC_PRI5_R = (NVIC_PRI5_R & 0xFFFFFF1F) | 0x000000E0;
    NVIC_EN0_R |= (1 << 20);
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
    I2C0_MTPR_R = 0x01; //Configuring SCL of I2C0 to run at 400KHz
}

/*
 * MPU9250.c
 *
 *  Created on: 21-Jun-2018
 *      Author: kshama
 */

#include <stdint.h>
#include <stdio.h>
#include "inc/tm4c123gh6pm.h"
#include "MPU9250.h"
#include "UartSerial.h"
#include <math.h>
#include "motors.h"

volatile float dt = 0;
float mres = 10. * 4912. / 32760.0;
volatile long cal_gx = 0, cal_gy = 0, cal_gz = 0;
volatile float a = 0, gyro_pitch = 0, accel_pitch = 0, pitch;

uint8_t debug = 0;

uint8_t calib = 0;

void delayMs(int n);
volatile int count = 0;

volatile float Kp = 700, Kd = 1200, P = 0, D = 0, PID = 0, last_error = 0, error =
        0;
volatile float set_point = -4;

void TIMER0_TA_Handler()
{
    uint8_t Buf[14];    //For Accelerometer and Gyrometer

    setSlaveAddress(MPU9250_ADDRESS);

    wireRead(ACCEL_XOUT_H, Buf, 14);
    // Accelerometer
    int16_t ax = (Buf[0] << 8 | Buf[1]);
    int16_t ay = (Buf[2] << 8 | Buf[3]);
    int16_t az = Buf[4] << 8 | Buf[5];

    // Gyroscope
    int16_t gx = (Buf[8] << 8 | Buf[9]);
    int16_t gy = (Buf[10] << 8 | Buf[11]);
    int16_t gz = Buf[12] << 8 | Buf[13];

    // Read register Status 1 and wait for the DRDY: Data Ready
//    setSlaveAddress(AK8963_ADDRESS);
//
////    debug = 0;
//////    while (!debug)
////    {
////        wireRead(AK8963_ST1, &debug, 1);
////        debug = debug & 0x01;
////    }
//
//    // Read magnetometer data
//    uint8_t Mag[7];
//    wireRead(AK8963_XOUT_L, Mag, 7);
//
//    // Create 16 bits values from 8 bits data
//
//    // Magnetometer
//    int16_t mx = -(Mag[3] << 8 | Mag[2]);
//    int16_t my = -(Mag[1] << 8 | Mag[0]);
//    int16_t mz = -(Mag[5] << 8 | Mag[4]);

    switch (calib)
    {
    case 1:
    {
        gx -= cal_gx;
        gy -= cal_gy;
        gz -= cal_gz;

        float axf = ax / 4096.0;
        float ayf = ay / 4096.0;
        float azf = az / 4096.0;

        float gxf = gx * 0.000002665;
        float gyf = gy * 0.000002665;
        float gzf = gz * 0.000002665;

//        //Micro Tesla
//        float mxf = mx * 0.146484375;
//        float myf = my * 0.146484375;
//        float mzf = mz * 0.146484375;

//        MadgwickQuaternionUpdate(gxf, gyf, gzf, axf, ayf, azf, mxf, myf, mzf);
        unsigned long delta_t = TIME_US;
        dt = (float) delta_t * 0.000002;
        TIMER1_CTL_R &= ~TAEN;
        TIMER1_TAPR_R = 0x10;
        TIMER1_TAILR_R = 0xFFFF;
        TIMER1_CTL_R |= TAEN;

        gyro_pitch += ((float) gy * dt / 131);
        pitch += ((float) gy * dt / 131);
        a = (axf * axf) + (ayf * ayf) + (azf * azf);
        a = sqrt(a);
        a = axf / a;
        accel_pitch = -180 * asin(a) / 3.14;
        pitch = (0.996 * pitch) + (0.004 * accel_pitch);

//        SerialPrintInt(axf * 1000);
//        UART_OutChar('\t');
//        SerialPrintInt(ayf * 1000);
//        UART_OutChar('\t');
//        SerialPrintInt(azf * 1000);
//        UART_OutChar('\t');
//        SerialPrintInt(gxf * 1000 / M_PI);
//        UART_OutChar('\t');
//        SerialPrintInt(gyf * 1000 / M_PI);
//        UART_OutChar('\t');
//        SerialPrintInt(gzf * 1000 / M_PI);
//        UART_OutChar('\t');
//        SerialPrintInt(mxf);
//        UART_OutChar('\t');
//        SerialPrintInt(myf);
//        UART_OutChar('\t');
//        SerialPrintInt(mzf);
//        UART_OutChar('\t');
//        SerialPrintInt(delta_t);
//        UART_OutChar('\n');
//        UART_OutChar(CR);
    }
        break;
    case 0:
    {
        if (count < 256)
        {
            cal_gx += gx;
            cal_gy += gy;
            cal_gz += gz;
            count++;
        }

        if (count == 256)
        {
            count++;
            cal_gx = cal_gx >> 8;
            cal_gy = cal_gy >> 8;
            cal_gz = cal_gz >> 8;
            calib = 1;
        }
    }
        break;
    }

    error = set_point - pitch;

    P = Kp * error, D = Kd * (error - last_error);
    last_error = error;
    PID = P + D;

    if (abs(PID) > 12000)
    {
        if (PID > 0)
        {
            PID = 12000;
        }
        else
        {
            PID = -12000;
        }
    }
    if (abs(error) < 30)
    {
        motor(0, -PID);
        motor(1, -PID);
    }
    else
    {
        motor(0, 0);
        motor(1, 0);
    }
    volatile int read_back = 0;
    read_back = read_back;
    TIMER0_ICR_R = 0x01;
    read_back = TIMER0_ICR_R;
}

/*
 void cal_angle()
 {
 // delta_t = TIME_US;
 // TIMER1_CTL_R &= ~TAEN;
 // TIMER1_TAPR_R = 0x10;
 // TIMER1_TAILR_R = 0xFFFF;
 // TIMER1_CTL_R |= TAEN;
 if (calib == 1)
 {
 gyro_pitch += ((float) gy * delta_t * 0.000001 / 131);
 pitch += ((float) gy * delta_t * 0.000001 / 131);
 a = (axf * axf) + (ayf * ayf) + (azf * azf);
 a = sqrt(a);
 a = axf / a;
 accel_pitch = -180 * asin(a) / 3.14;
 pitch = (0.999 * pitch) + (0.001 * accel_pitch);
 }

 SerialPrintInt(mx);
 UART_OutChar('\t');
 SerialPrintInt(my);
 UART_OutChar('\t');
 SerialPrintInt(mz);
 UART_OutChar('\t');
 SerialPrintInt(delta_t);
 UART_OutChar('\t');
 SerialPrintInt(PID);
 UART_OutChar('\t');
 SerialPrintInt(pitch);
 UART_OutChar('\t');
 SerialPrintInt(accel_pitch);
 UART_OutChar('\t');
 SerialPrintInt(gyro_pitch);
 UART_OutChar('\n');
 UART_OutChar(CR);

 error = set_point - pitch;

 P = Kp * error, D = Kd * (error - last_error);
 last_error = error;
 PID = P + D;

 if (abs(PID) > 12000)
 {
 if (PID > 0)
 {
 PID = 12000;
 }
 else
 {
 PID = -12000;
 }
 }
 motor(0, -PID);
 motor(1, -PID);
 }
 */

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

void wireRead(uint8_t address, uint8_t *data, uint8_t num_bytes)
{
    wireSend(address, (1 << 0) | (1 << 1));
    setRWMode(1);

    if (num_bytes == 1)
    {
        I2C0_MCS_R = ((1 << 0) | (1 << 1) | (1 << 2));
        waitAndErrorCheck();
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

void init_timer1A(void)
{
    SYSCTL_RCGCTIMER_R |= 0x02;
    TIMER1_CTL_R &= ~(TAEN);    //Disabling Timer A
    TIMER1_CFG_R &= ~(0x07);    //Clearing D2:D0 bits
    TIMER1_CFG_R |= BIT16_MODE; //Configuring Timer 0 as 16 bit timer
    TIMER1_TAMR_R &= ~(0x03);   //Clearing D1:D0 bits
    TIMER1_TAMR_R |= ONE_SHOT;  //Periodic mode
    TIMER1_TAMR_R |= COUNT_DOWN;    //Count down mode
    TIMER1_TAILR_R = 0xFFFF;
    TIMER1_TAPR_R = 0x10;
    TIMER1_ICR_R |= 0x01;
}

void init_timer0A(int time_ms)
{
    SYSCTL_RCGCTIMER_R |= 0x01;
    TIMER0_CTL_R &= ~(TAEN);    //Disabling Timer A
    TIMER0_CFG_R &= ~(0x07);    //Clearing D2:D0 bits
    TIMER0_CFG_R |= BIT16_MODE; //Configuring Timer 0 as 16 bit timer
    TIMER0_TAMR_R &= ~(0x03);   //Clearing D1:D0 bits
    TIMER0_TAMR_R |= PERIODIC;  //Periodic mode
    TIMER0_TAMR_R |= COUNT_DOWN;    //Count down mode
    uint32_t load_register = (time_ms * 16 * 1000 / 256) - 1; //Calculating ticks required at 16MHz
    TIMER0_TAPR_R = 0xFF;
    TIMER0_TAILR_R = load_register;
    TIMER0_ICR_R |= 0x01;

    TIMER0_IMR_R |= TATORIM;    //Enabling Interrupt Timerout
    NVIC_PRI4_R = (NVIC_PRI4_R & 0x1FFFFFFF) | 0xA0000000;
    NVIC_EN0_R = (1 << 19);
}

void init_MPU9250()
{
//    //Initialising PC4 as RISING pin change interrupt
//     SYSCTL_RCGC2_R |= (1 << 2); //Enabling Clock to GPIO PORTC
//     GPIO_PORTC_LOCK_R = 0x4C4F434B; //unlock GPIO PORTC
//     GPIO_PORTC_CR_R |= (1 << 4); //allow changes to PC6 and PC7
//     GPIO_PORTC_AMSEL_R &= ~(1 << 4); //disable analog on PC6 and PC7
//     GPIO_PORTC_PCTL_R &= ~(0xF << 16); //PCTL GPIO on PC6 and PC7
//     GPIO_PORTC_DIR_R &= ~(1 << 4); //PC6 and PC7 as Inputs
//     GPIO_PORTC_AFSEL_R &= ~(1 << 4); //disable alternate function on PC6 and PC7
//     GPIO_PORTC_DEN_R |= (1 << 4); //enable digital function on PC6 and PC7
//
//     //Interrupt Settings
//     GPIO_PORTC_IS_R &= ~(1 << 4);
//     GPIO_PORTC_IBE_R &= ~(1 << 4); //Disabling Both Edge Interrupts on pins PC6 and PC7
//     GPIO_PORTC_ICR_R |= (1 << 4);//Clearing interrupt flags of PC6 and PC7
//     GPIO_PORTC_IEV_R &= ~(1 << 4);
//     GPIO_PORTC_IM_R |= (1 << 4);   //Unmask PC6 and PC7 interrupts
//     GPIO_PORTC_ICR_R |= (1 << 4);
//     //IRQ Number of PORTC = 2
//     NVIC_PRI0_R &= ~(0x7 << 21);
//     NVIC_PRI0_R |= (0x5 << 21); //Priority level 5
//     NVIC_EN0_R |= (1 << 2); //Enable interrupt 2 in NVIC

    setSlaveAddress(MPU9250_ADDRESS);  //Setting slave address
    delayMs(10);
//Read Who AM I register
    printString("WHO_AM_I REGISTER VALUE: ");
    wireRead(WHO_AM_I_MPU9250, &debug, 1);
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
    I2CwriteByte(PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
    delayMs(100); // Wait for all registers to reset

// get stable time source
    I2CwriteByte(PWR_MGMT_1, 0x01); // Auto select clock source to be PLL gyroscope reference if ready else
    delayMs(200);

    I2CwriteByte(CONFIG, 0x03); //Wake UP MPU9250
    delayMs(10);
    I2CwriteByte(SMPLRT_DIV, 0x04);
    I2CwriteByte(GYRO_CONFIG, GYRO_FULL_SCALE_500_DPS);
    I2CwriteByte(ACCEL_CONFIG, ACC_FULL_SCALE_8_G);

// Set by pass mode for the magnetometers
    I2CwriteByte(INT_PIN_CFG, 0x02);

// Request continuous magnetometer measurements in 16 bits
    setSlaveAddress(AK8963_ADDRESS);  //Setting slave address
    delayMs(10);
    I2CwriteByte(AK8963_CNTL, 0x16);
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

/*
 * MPU9250_Madgwick.c
 *
 *  Created on: 21-Jun-2018
 *      Author: kamal
 */

#include <stdint.h>
#include <stdio.h>
#include "inc/tm4c123gh6pm.h"
#include "MPU9250_madgwick.h"
#include "UartSerial.h"

void accel_gyro_cal_MPU9250(float *gyro, float *accel);

uint8_t debug = 0;
float gyroBias[3] = { 0, 0, 0 };
float accelBias[3] = { 0, 0, 0 };
float magBias[3] = { 0, 0, 0 };
float magScale[3] = { 0, 0, 0 }; // Bias corrections for gyro and accelerometer

float magCalibration[3] = { 0, 0, 0 };  // Factory mag calibration and mag bias

float aRes = 4.0 / 32768.0;
float gRes = 250.0 / 32768.0;;
float mRes = 10. * 4912. / 32760.0;;      // scale resolutions per LSB for the sensors

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

void I2CwriteByte(uint8_t Register, uint8_t Data)
{
    wireSend(Register, (1 << 0) | (1 << 1));
    wireSend(Data, (1 << 0) | (1 << 2));
}

void setSlaveAddress(uint8_t address)
{
    I2C0_MSA_R = (address << 1);
}

void readMagData(int16_t * destination)
{
    uint8_t rawData[7]; // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
    debug = 0;
    wireRead(AK8963_ST1, &debug, 1);
    debug = debug & 0x01;
    if (debug)
    { // wait for magnetometer data ready bit to be set
        wireRead(AK8963_XOUT_L, rawData, 7); // Read the six raw data and ST2 registers sequentially into data array
        uint8_t c = rawData[6]; // End data read by reading ST2 register
        if (!(c & 0x08))
        { // Check if magnetic sensor overflow set, if not then report data
            destination[0] = ((int16_t) rawData[1] << 8) | rawData[0]; // Turn the MSB and LSB into a signed 16-bit value
            destination[1] = ((int16_t) rawData[3] << 8) | rawData[2]; // Data stored as little Endian
            destination[2] = ((int16_t) rawData[5] << 8) | rawData[4];
        }
    }
}
void accel_gyro_cal_MPU9250(float *gyro, float *accel)
{
    uint8_t data[12];
    uint16_t ii, packet_count, fifo_count;
    int32_t gyro_bias[3] = { 0, 0, 0 }, accel_bias[3] = { 0, 0, 0 };

    I2CwriteByte(PWR_MGMT_1, 0x80);
    delayMs(100);

    I2CwriteByte(PWR_MGMT_1, 0x01);
    I2CwriteByte(PWR_MGMT_2, 0x00);
    delayMs(100);

    // Configure device for bias calculation
    I2CwriteByte(INT_ENABLE, 0x00);   // Disable all interrupts
    I2CwriteByte(FIFO_EN, 0x00);      // Disable FIFO
    I2CwriteByte(PWR_MGMT_1, 0x00); // Turn on internal clock source
    I2CwriteByte(I2C_MST_CTRL, 0x00); // Disable I2C master
    I2CwriteByte(USER_CTRL, 0x00); // Disable FIFO and I2C master modes
    I2CwriteByte(USER_CTRL, 0x0C);    // Reset FIFO and DMP
    delayMs(15);

    // Configure MPU6050 gyro and accelerometer for bias calculation
    I2CwriteByte(CONFIG, 0x01);   // Set low-pass filter to 188 Hz
    I2CwriteByte(SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
    I2CwriteByte(GYRO_CONFIG, 0x00); // Set gyro full-scale to 250 degrees per second, maximum sensitivity
    I2CwriteByte(ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

    uint16_t gyrosensitivity = 131;   // = 131 LSB/degrees/sec
    uint16_t accelsensitivity = 16384;  // = 16384 LSB/g

    // Configure FIFO to capture accelerometer and gyro data for bias calculation
    I2CwriteByte(USER_CTRL, 0x40);   // Enable FIFO
    I2CwriteByte(FIFO_EN, 0x78); // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
    delayMs(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

    // At end of sample accumulation, turn off FIFO sensor read
    I2CwriteByte(FIFO_EN, 0x00); // Disable gyro and accelerometer sensors for FIFO
    wireRead(FIFO_COUNTH, data, 2); // read FIFO sample count
    fifo_count = ((uint16_t) data[0] << 8) | data[1];

    SerialPrintInt(fifo_count);
    printString("\n");

    packet_count = fifo_count / 12; // How many sets of full gyro and accelerometer data for averaging

    for (ii = 0; ii < packet_count; ii++)
    {
        int16_t accel_temp[3] = { 0, 0, 0 }, gyro_temp[3] = { 0, 0, 0 };
        wireRead(FIFO_R_W, data, 12); // read data for averaging
        accel_temp[0] = (int16_t) (((int16_t) data[0] << 8) | data[1]); // Form signed 16-bit integer for each sample in FIFO
        accel_temp[1] = (int16_t) (((int16_t) data[2] << 8) | data[3]);
        accel_temp[2] = (int16_t) (((int16_t) data[4] << 8) | data[5]);
        gyro_temp[0] = (int16_t) (((int16_t) data[6] << 8) | data[7]);
        gyro_temp[1] = (int16_t) (((int16_t) data[8] << 8) | data[9]);
        gyro_temp[2] = (int16_t) (((int16_t) data[10] << 8) | data[11]);

        accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
        accel_bias[1] += (int32_t) accel_temp[1];
        accel_bias[2] += (int32_t) accel_temp[2];
        gyro_bias[0] += (int32_t) gyro_temp[0];
        gyro_bias[1] += (int32_t) gyro_temp[1];
        gyro_bias[2] += (int32_t) gyro_temp[2];

    }

    accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
    accel_bias[1] /= (int32_t) packet_count;
    accel_bias[2] /= (int32_t) packet_count;
    gyro_bias[0] /= (int32_t) packet_count;
    gyro_bias[1] /= (int32_t) packet_count;
    gyro_bias[2] /= (int32_t) packet_count;

    if (accel_bias[2] > 0L)
    {
        accel_bias[2] -= (int32_t) accelsensitivity;
    }  // Remove gravity from the z-axis accelerometer bias calculation
    else
    {
        accel_bias[2] += (int32_t) accelsensitivity;
    }

    // Output scaled gyro biases for display in the main program
    gyro[0] = (float) gyro_bias[0] / (float) gyrosensitivity;
    gyro[1] = (float) gyro_bias[1] / (float) gyrosensitivity;
    gyro[2] = (float) gyro_bias[2] / (float) gyrosensitivity;

    // Output scaled accelerometer biases for display in the main program
    accel[0] = (float) accel_bias[0] / (float) accelsensitivity;
    accel[1] = (float) accel_bias[1] / (float) accelsensitivity;
    accel[2] = (float) accel_bias[2] / (float) accelsensitivity;
}

void magcalMPU9250(float * dest1, float * dest2)
{
    uint16_t ii = 0, jj = 0, sample_count = 0;
    int32_t mag_bias[3] = { 0, 0, 0 }, mag_scale[3] = { 0, 0, 0 };
    int16_t mag_max[3] = { -32767, -32767, -32767 }, mag_min[3] =
            { 32767, 32767, 32767 }, mag_temp[3] = { 0, 0, 0 };

    printString("Mag Calibration: Wave device in a figure eight until done!");
    delayMs(2000);

    //Mmode == 0x06
    sample_count = 1500; // at 100 Hz ODR, new mag data is available every 10 ms
    for (ii = 0; ii < sample_count; ii++)
    {
        readMagData(mag_temp);  // Read the mag data
        for (jj = 0; jj < 3; jj++)
        {
            if (mag_temp[jj] > mag_max[jj])
                mag_max[jj] = mag_temp[jj];
            if (mag_temp[jj] < mag_min[jj])
                mag_min[jj] = mag_temp[jj];
        }
        // Mmode == 0x06
        delayMs(12);  // at 100 Hz ODR, new mag data is available every 10 ms
    }

//    Serial.println("mag x min/max:"); Serial.println(mag_max[0]); Serial.println(mag_min[0]);
//    Serial.println("mag y min/max:"); Serial.println(mag_max[1]); Serial.println(mag_min[1]);
//    Serial.println("mag z min/max:"); Serial.println(mag_max[2]); Serial.println(mag_min[2]);

    // Get hard iron correction
    mag_bias[0] = (mag_max[0] + mag_min[0]) / 2; // get average x mag bias in counts
    mag_bias[1] = (mag_max[1] + mag_min[1]) / 2; // get average y mag bias in counts
    mag_bias[2] = (mag_max[2] + mag_min[2]) / 2; // get average z mag bias in counts

    dest1[0] = (float) mag_bias[0] * mRes * magCalibration[0]; // save mag biases in G for main program
    dest1[1] = (float) mag_bias[1] * mRes * magCalibration[1];
    dest1[2] = (float) mag_bias[2] * mRes * magCalibration[2];

    // Get soft iron correction estimate
    mag_scale[0] = (mag_max[0] - mag_min[0]) / 2; // get average x axis max chord length in counts
    mag_scale[1] = (mag_max[1] - mag_min[1]) / 2; // get average y axis max chord length in counts
    mag_scale[2] = (mag_max[2] - mag_min[2]) / 2; // get average z axis max chord length in counts

    float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
    avg_rad /= 3.0;

    dest2[0] = avg_rad / ((float) mag_scale[0]);
    dest2[1] = avg_rad / ((float) mag_scale[1]);
    dest2[2] = avg_rad / ((float) mag_scale[2]);

    printString("Mag Calibration done!");
}

void initAK8963(float * destination)
{
    // First extract the factory calibration for each magnetometer axis
    uint8_t rawData[3];  // x/y/z gyro calibration data stored here
    I2CwriteByte(AK8963_CNTL, 0x00); // Power down magnetometer
    delayMs(10);
    I2CwriteByte(AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
    delayMs(10);
    wireRead(AK8963_ASAX, rawData, 3); // Read the x-, y-, and z-axis calibration values
    destination[0] = (float) (rawData[0] - 128) / 256. + 1.; // Return x-axis sensitivity adjustment values, etc.
    destination[1] = (float) (rawData[1] - 128) / 256. + 1.;
    destination[2] = (float) (rawData[2] - 128) / 256. + 1.;
    I2CwriteByte(AK8963_CNTL, 0x00); // Power down magnetometer
    delayMs(10);
    // Configure the magnetometer for continuous read and highest resolution
    // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
    // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
    I2CwriteByte(AK8963_CNTL, (1 << 4) | (0x06)); // Set magnetometer data resolution and sample ODR
    delayMs(10);
}

void init_MPU9250()
{
    setSlaveAddress(MPU9250_ADDRESS);  //Setting slave address

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

    printString(" Calibrate gyro and accel");
    accel_gyro_cal_MPU9250(gyroBias, accelBias);
    printString("accel biases (mg)\n");
    SerialPrintInt(1000. * accelBias[0]);
    printString("\n");
    SerialPrintInt(1000. * accelBias[1]);
    printString("\n");
    SerialPrintInt(1000. * accelBias[2]);
    printString("\n");
    printString("gyro biases (dps)\n");
    SerialPrintInt(gyroBias[0]);
    printString("\n");
    SerialPrintInt(gyroBias[1]);
    printString("\n");
    SerialPrintInt(gyroBias[2]);
    printString("\n");

    // wake up device
    I2CwriteByte(PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
    delayMs(100); // Wait for all registers to reset

    // get stable time source
    I2CwriteByte(PWR_MGMT_1, 0x01); // Auto select clock source to be PLL gyroscope reference if ready else
    delayMs(200);

    // Configure Gyro and Thermometer
    // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
    // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
    // be higher than 1 / 0.0059 = 170 Hz
    // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
    // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
    I2CwriteByte(CONFIG, 0x03);

    // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
    I2CwriteByte(SMPLRT_DIV, 0x04); // Use a 200 Hz rate; a rate consistent with the filter update rate
    // determined inset in CONFIG above

    // Set gyroscope full scale range
    // Range selects FS_SEL and GFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
    uint8_t c;
    wireRead(GYRO_CONFIG, &c, 1); // get current GYRO_CONFIG register value
    // c = c & ~0xE0; // Clear self-test bits [7:5]
    c = c & ~0x03; // Clear Fchoice bits [1:0]
    c = c & ~0x18; // Clear GFS bits [4:3]
    c = c | (GYRO_FULL_SCALE_250_DPS); // Set full scale range for the gyro
    // c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
    I2CwriteByte(GYRO_CONFIG, c); // Write new GYRO_CONFIG value to register

    // Set accelerometer full-scale range configuration
    wireRead(ACCEL_CONFIG, &c, 1); // get current ACCEL_CONFIG register value
    // c = c & ~0xE0; // Clear self-test bits [7:5]
    c = c & ~0x18;  // Clear AFS bits [4:3]
    c = c | (ACC_FULL_SCALE_4_G); // Set full scale range for the accelerometer
    I2CwriteByte(ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value

    // Set accelerometer sample rate configuration
    // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
    // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
    wireRead(ACCEL_CONFIG2, &c, 1); // get current ACCEL_CONFIG2 register value
    c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
    c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
    I2CwriteByte(ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value

    // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
    // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

    // Configure Interrupts and Bypass Enable
    // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
    // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
    // can join the I2C bus and all can be controlled by the Arduino as master
//   writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);
    I2CwriteByte(INT_PIN_CFG, 0x12); // INT is 50 microsecond pulse and any read to clear
    I2CwriteByte(INT_ENABLE, 0x01); // Enable data ready (bit 0) interrupt
    delayMs(100);

    printString("MPU9250 initialized for active data mode....\n");

    // Read the WHO_AM_I register of the magnetometer, this is a good test of communication
    setSlaveAddress(AK8963_ADDRESS);  //Setting slave address
    printString("WHO_AM_I REGISTER VALUE: ");
    wireRead(AK8963_WHO_AM_I, &debug, 1);
    SerialPrintInt(debug);
    printString("\n");
    if (debug == 0x48)
    {
        printString("Magnetometer is online...\n");
    }

    printString("AK8963 initialized for active data mode...."); // Initialize device for active mode read of magnetometer

    magcalMPU9250(magBias, magScale);

    magcalMPU9250(magBias, magScale);
    printString("\nAK8963 mag biases (mG)\n");
    SerialPrintInt(magBias[0]);
    printString("\n");
    SerialPrintInt(magBias[1]);
    printString("\n");
    SerialPrintInt(magBias[2]);
    printString("\n");
    printString("AK8963 mag scale (mG)");
    SerialPrintInt(magScale[0]);
    printString("\n");
    SerialPrintInt(magScale[1]);
    printString("\n");
    SerialPrintInt(magScale[2]);
    printString("\n");
    delayMs(2000); // add delay to see results before serial spew of data
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

int main()
{
    init_I2C0();
    UART_Init();
    init_MPU9250();
    while (1)
    {

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

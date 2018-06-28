/*
 * PCA9685.h
 *
 *  Created on: 17-Jun-2018
 *      Author: kamal
 */

#ifndef PCA9685_H_
#define PCA9685_H_
#endif /* PCA9685_H_ */

#define RED_LED 14
#define YELLOW_LED    12
#define GREEN_LED   13
#define BUZZER  15

#define PCA9685_ADDRESS 0x40
#define MODE1_ADDRESS   0x00
#define MODE2_ADDRESS   0x01
#define PRESCALE_ADDRESS 0xFE
#define PRESCALER_1526HZ 0x1E

#define LED0_ON_L 0x06

#define ON_LOW 0x00
#define ON_HIGH 0x01
#define OFF_LOW 0x02
#define OFF_HIGH 0x03

//Function Definitions
void setRWMode(uint8_t mode);
void waitAndErrorCheck();
void wireSend(uint8_t byte, uint8_t condition);
void wireRead(uint8_t address, uint8_t *data, uint8_t num_bytes);
void I2CwriteByte(uint8_t Register, uint8_t Data);
void setSlaveAddress(uint8_t address);
void PCA9685_analogWrite(uint8_t pin, uint16_t on, uint16_t off);
void init_PCA9685();
void init_I2C1();
void PCA9685_digitalWrite(uint8_t pin, uint8_t state);

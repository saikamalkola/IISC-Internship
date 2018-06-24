/*
 * Ultrasonic.h
 *
 *  Created on: 18-Jun-2018
 *      Author: kamal
 */

#ifndef ULTRASONIC_H_
#define ULTRASONIC_H_
#endif /* ULTRASONIC_H_ */

#define ECHO_PIN0   ((GPIO_PORTD_DATA_R >> 0) & 0x01)
#define ECHO_PIN1   ((GPIO_PORTD_DATA_R >> 1) & 0x01)
#define ECHO_PIN2   ((GPIO_PORTD_DATA_R >> 2) & 0x01)
#define ECHO_PIN3   ((GPIO_PORTD_DATA_R >> 3) & 0x01)

#define TIME_US (0xFFFF - (TIMER1_TAV_R & 0xFFFF))

void TIMER2_TA_Handler(void);

void delayUs(int n);
float read_distance(uint8_t index, uint16_t timeOut);
uint8_t echo_state(uint8_t index);
void Ultrasonic_Init(void);
void init_timer1A(void);
void init_timer2A(int time_ms);
unsigned long micros();
unsigned long millis();

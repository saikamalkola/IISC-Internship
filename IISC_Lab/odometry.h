/*
 * odometry.h
 *
 *  Created on: 17-Jun-2018
 *      Author: kamal
 */

#ifndef ODOMETRY_H_
#define ODOMETRY_H_
#endif /* ODOMETRY_H_ */

#define TAEN 0x01
#define TBEN 0x00000100
#define COUNT_DOWN 0x00000000
#define COUNT_UP 0x00000010
#define TATORIM 0x01

#define BIT32_MODE 0
#define RTC_MODE 1
#define BIT16_MODE 4
#define ONE_SHOT 1
#define PERIODIC 2
#define CAPTURE 3

#define M_PI_2      1.57079632679489661923  /* pi/2 */

//Function definitions
void init_timer0A(int time_ms);
void TIMER0_TA_Handler(void);
int sign(int val);

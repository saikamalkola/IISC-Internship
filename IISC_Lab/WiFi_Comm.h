/*
 * WiFi_Comm.h
 *
 *  Created on: 24-Jun-2018
 *      Author: kamal
 */

#ifndef WIFI_COMM_H_
#define WIFI_COMM_H_

void update_UI();
void UI_OutChar(char data);
void printString_UI(char *msg);
void UART1_Init(void);
void UIPrintInt(int integer);



#endif /* WIFI_COMM_H_ */

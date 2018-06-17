/*
 * UartSerial.h
 *
 *  Created on: 17-Jun-2018
 *      Author: kamal
 */

#ifndef UARTSERIAL_H_
#define UARTSERIAL_H_
#endif /* UARTSERIAL_H_ */

void SerialPrintInt(int integer);
char *itoa(int value, char *result, int base);
char UART_InChar(void);
int SerialAvailable();
void UART_OutChar(char data);
void read_line(char *buffer);
void print_line(char *data);
void UART_Init(void);

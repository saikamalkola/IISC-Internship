/*
 * UartSerial.c
 *
 *  Created on: 16-Jun-2018
 *      Author: kamal
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "inc/tm4c123gh6pm.h"
#include "UartSerial.h"

void SerialPrintInt(int integer)
{
    char buffer[20];
    itoa(integer, buffer, 10);
    print_line(buffer);
}

char *itoa(int value, char *result, int base)
{
    // check that the base if valid
    if (base < 2 || base > 36)
    {
        *result = '\0';
        return result;
    }

    char* ptr = result, *ptr1 = result, tmp_char;
    int tmp_value;

    do
    {
        tmp_value = value;
        value /= base;
        *ptr++ =
                "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz"[35
                        + (tmp_value - value * base)];
    }
    while (value);

    // Apply negative sign
    if (tmp_value < 0)
        *ptr++ = '-';
    *ptr-- = '\n';
    while (ptr1 < ptr)
    {
        tmp_char = *ptr;
        *ptr-- = *ptr1;
        *ptr1++ = tmp_char;
    }
    return result;
}

char UART_InChar(void)
{
    while ((UART0_FR_R & UART_FR_RXFE) != 0)
        ;
    return ((char) (UART0_DR_R & 0xFF));
}

int SerialAvailable()
{
    if ((UART0_FR_R & UART_FR_RXFE) == 0)
    {
        return 1;
    }

    else
    {
        return -1;
    }
}

void UART_OutChar(char data)
{
    while ((UART0_FR_R & UART_FR_TXFF) != 0)
        ;
    UART0_DR_R = data;
}

void read_line(char *buffer)
{
    int new_line = 0, count = 0;
    while (!new_line)
    {
        buffer[count] = UART_InChar();
        if (buffer[count] == '\n')
        {
            buffer[count] = '\n';
            new_line = 1;
        }
        count++;
    }
}

void print_line(char *data)
{
    int k = 0;
    while (1)
    {
        if (data[k] == '\n')
        {
            //UART_OutChar(data[k]);
            break;
        }
        UART_OutChar(data[k]);
        k++;
    }
}

void UART_Init(void)
{
    SYSCTL_RCGCUART_R |= 0x01;            //activate UART0
    SYSCTL_RCGCGPIO_R |= 0x01;            //activate port A

    while ((SYSCTL_PRGPIO_R & 0x0001) == 0)
    {
    };            // ready?
    UART0_CTL_R &= ~UART_CTL_UARTEN;      //disable UART0
    UART0_IBRD_R = 8;     //IBRD = int(16,000,000 / (16 * 115,200)) = int(8.680)
    UART0_FBRD_R = 44;       //FBRD = round(0.5104 * 64 ) = 44
                             //8 bit word length (no parity bits, one stop bit, FIFOs)
    UART0_LCRH_R = (UART_LCRH_WLEN_8 | UART_LCRH_FEN);
    UART0_CTL_R |= UART_CTL_UARTEN;       //enable UART
    GPIO_PORTA_AFSEL_R |= 0x03;           //enable alt funct on PA1-0
    GPIO_PORTA_DEN_R |= 0x03;             //enable digital I/O on PA1-0
    GPIO_PORTA_PCTL_R = (GPIO_PORTA_PCTL_R & 0xFFFFFF00) + 0x00000011; //configure PA1-0 as UART
    GPIO_PORTA_AMSEL_R &= ~0x03;          //disable analog functionality on PA
}


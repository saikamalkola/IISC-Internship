/*
 * WiFi_Comm.c
 *
 *  Created on: 24-Jun-2018
 *      Author: kamal
 */
#include <stdbool.h>
#include <stdint.h>
#include "inc/tm4c123gh6pm.h"
#include "UartSerial.h"
#include "WiFi_Comm.h"

extern struct Position
{
    volatile float x;
    volatile float y;
    volatile float theta;
} position;

void UI_read_line(char *buffer)
{
    int new_line = 0, count = 0;
    while (!new_line)
    {
        buffer[count] = UI_InChar();
        if (buffer[count] == '\n')
        {
            buffer[count] = '\n';
            new_line = 1;
        }
        count++;
    }
}

char UI_InChar(void)
{
    while ((UART1_FR_R & UART_FR_RXFE) != 0)
        ;
    return ((char) (UART1_DR_R & 0xFF));
}

int UI_SerialAvailable()
{
    if ((UART1_FR_R & UART_FR_RXFE) == 0)
    {
        return 1;
    }

    else
    {
        return -1;
    }
}

void UI_print_line(char *data)
{
    int k = 0;
    while (1)
    {
        if (data[k] == '\n')
        {
            //UART_OutChar(data[k]);
            break;
        }
        UI_OutChar(data[k]);
        k++;
    }
}

void UIPrintInt(int integer)
{
    char buffer[20];
    itoa(integer, buffer, 10);
    UI_print_line(buffer);
}

void update_UI()
{
    UIPrintInt(read_battery_voltage() * 1000);
    UI_OutChar('\t');
    UIPrintInt(position.x);
    UI_OutChar('\t');
    UIPrintInt(position.y);
    UI_OutChar('\t');
    UIPrintInt(position.theta * 180 / 3.14);
    UI_OutChar('\n');
}

void UI_OutChar(char data)
{
    while ((UART1_FR_R & UART_FR_TXFF) != 0)
        ;
    UART1_DR_R = data;
}

void printString_UI(char *msg)
{
    while (*msg)
        UI_OutChar(*msg++);
}

void UART1_Init(void)
{
    SYSCTL_RCGCUART_R |= 0x02; /* activate UART1 */
    SYSCTL_RCGCGPIO_R |= 0x02; /* activate port B */

    int k = 0;
    for (k = 0; k < 10; k++)
        ;
    UART1_CTL_R &= ~UART_CTL_UARTEN; /* disable UART */
    UART1_IBRD_R = 8; /* IBRD = int(16,000,000 / (16 * 115,200)) = int(8.680) */
    UART1_FBRD_R = 44; /* FBRD = round(0.5104 * 64 ) = 44 */
    /* 8 bit word length (no parity bits, one stop bit, FIFOs) */
    UART1_LCRH_R = (UART_LCRH_WLEN_8 | UART_LCRH_FEN);
    UART1_CTL_R |= UART_CTL_UARTEN; /* enable UART */
    GPIO_PORTB_AFSEL_R |= 0x03; /* enable alt funct on PA1-0 */
    GPIO_PORTB_DEN_R |= 0x03; /* enable digital I/O on PA1-0 */
    GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R & 0xFFFFFF00) + 0x00000011; /* configure PA1-0 as UART */
    GPIO_PORTB_AMSEL_R &= ~0x03; /* disable analog functionality on PA */
}


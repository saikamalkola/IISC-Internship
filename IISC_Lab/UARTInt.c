/* This program sets up UART0 on TI ARM LaunchPad (TM4C123GH6PM) to do terminal echo.
 * When a key is pressed at the terminal emulator of the PC, the character is received by
 * UART0 and it is sent out of UART0 back to the terminal.
 */

#include <stdio.h>
#include <stdint.h>
#include "inc/tm4c123gh6pm.h"

// standard ASCII symbols
#define CR   0x0D
#define LF   0x0A
#define BS   0x08
#define ESC  0x1B
#define SP   0x20
#define DEL  0x7F

/* U0Rx receive connected to PA0 */
/* U0Tx transmit connected to PA1 */

#define UART_FR_TXFF            0x00000020  /* UART Transmit FIFO Full */
#define UART_FR_RXFE            0x00000010  /* UART Receive FIFO Empty */
#define UART_LCRH_WLEN_8        0x00000060  /* 8 bit word length */
#define UART_LCRH_FEN           0x00000010  /* UART Enable FIFOs */
#define UART_CTL_UARTEN         0x00000001  /* UART Enable */
#define SYSCTL_RCGC1_UART0      0x00000001  /* UART0 Clock Gating Control */
#define SYSCTL_RCGC2_GPIOA      0x00000001  /* port A Clock Gating Control */

void UART_Init(void);
char UART_InChar(void);
void UART_OutChar(char data);
char *read_line(void);
void print_line(char *data);
int SerialAvailable(void);

char* msg = "\n\rEmbedded Systems Lab\n\r";

int main(void)
{
    char *data_line;
    UART_Init();

    while( 1 )
    {

    }
}

/* UART_Init
* Initialize the UART for 115,200 baud rate (assuming 16 MHz bus clock),
* 8 bit word length, no parity bits, one stop bit, FIFOs enabled
* Input: none
* Output: none
*/
void UART_Init(void)
{
      SYSCTL_RCGCUART_R |= 0x01;            /* activate UART0 */
      SYSCTL_RCGCGPIO_R |= 0x01;            /* activate port A */

      while((SYSCTL_PRGPIO_R&0x0001) == 0){};/* ready? */
      UART0_CTL_R &= ~UART_CTL_UARTEN;      /* disable UART */
      UART0_IBRD_R = 8;        /* IBRD = int(16,000,000 / (16 * 115,200)) = int(8.680) */
      UART0_FBRD_R = 44;       /* FBRD = round(0.5104 * 64 ) = 44 */
                               /* 8 bit word length (no parity bits, one stop bit, FIFOs) */
      UART0_LCRH_R = (UART_LCRH_WLEN_8|UART_LCRH_FEN);
      UART0_CTL_R |= UART_CTL_UARTEN;       /* enable UART */
      GPIO_PORTA_AFSEL_R |= 0x03;           /* enable alt funct on PA1-0 */
      GPIO_PORTA_DEN_R |= 0x03;             /* enable digital I/O on PA1-0 */
      GPIO_PORTA_PCTL_R = (GPIO_PORTA_PCTL_R & 0xFFFFFF00)+0x00000011; /* configure PA1-0 as UART */
      GPIO_PORTA_AMSEL_R &= ~0x03;          /* disable analog functionality on PA */
}

/* UART_InChar
* Wait for new serial port input
* Input: none
* Output: ASCII code for key typed
*
*/
char UART_InChar(void)
{
      while( (UART0_FR_R & UART_FR_RXFE) != 0)
          ;
      return((char)(UART0_DR_R & 0xFF));
}

int SerialAvailable()
{
    if((UART0_FR_R & UART_FR_RXFE) == 0)
    {
        return 1;
    }

    else
    {
        return -1;
    }
}

/* UART_OutChar
* Output 8-bit to serial port
* Input: letter is an 8-bit ASCII character to be transferred
* Output: none
*/
void UART_OutChar(char data)
{
      while((UART0_FR_R & UART_FR_TXFF) != 0)
          ;
      UART0_DR_R = data;
}

char *read_line()
{
    char buffer[256];
    int new_line = 0, count = 0;
    while(!new_line)
    {
        buffer[count] = UART_InChar();
        if(buffer[count] == CR)
        {
            buffer[count] = '\n';
            new_line = 1;
        }
        count++;
    }
    return buffer;
}

void print_line(char *data)
{
    int k = 0;
    while(1)
    {
        if(data[k] == '\n')
        {
            UART_OutChar(data[k]);
            break;
        }
         UART_OutChar(data[k]);
         k++;
    }
}

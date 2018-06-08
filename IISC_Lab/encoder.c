#include <stdint.h>
#include "inc/tm4c123gh6pm.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define RISING  0
#define FALLING 1
#define LOW     2
#define HIGH    3
#define CHANGE 4

#define UART_FR_TXFF            0x00000020  /* UART Transmit FIFO Full */
#define UART_FR_RXFE            0x00000010  /* UART Receive FIFO Empty */
#define UART_LCRH_WLEN_8        0x00000060  /* 8 bit word length */
#define UART_LCRH_FEN           0x00000010  /* UART Enable FIFOs */
#define UART_CTL_UARTEN         0x00000001  /* UART Enable */
#define SYSCTL_RCGC1_UART0      0x00000001  /* UART0 Clock Gating Control */
#define SYSCTL_RCGC2_GPIOA      0x00000001  /* port A Clock Gating Control */

#define outputA GPIO_PORTF_MIS_R & 0x01
#define outputB GPIO_PORTF_MIS_R & 0x10

void DisableInterrupts(void);
void EnableInterrupts(void);
void WaitForInterrupt(void);
void GPIOPortF_Init(void);
void GPIOPortF_Handler(void);
void attachInterrupt(char port, int pin, int mode);


volatile int lastEncoded = 0;
volatile long encoderValue = 0;

void attachInterrupt(char port, int pin, int mode)
{
    int int_sense = 0, int_event = 0;
    int bit_mask = 1 << pin;
    if (mode != CHANGE)
    {
        int_event = mode & 0x01;
        int_sense = (mode >> 1) & 0x01;
    }
    if (port == 'F')
    {
        if (int_sense) // Selecting pin is edge sensitive or level sensitive
        {
            GPIO_PORTF_IS_R |= bit_mask;
        }
        else
        {
            GPIO_PORTF_IS_R &= ~(bit_mask);
        }
        if (mode == CHANGE)
        {
            GPIO_PORTF_IBE_R |= (bit_mask);
        }
        else
        {
            GPIO_PORTF_IBE_R &= ~(bit_mask);
            if (int_event)
            {
                GPIO_PORTF_IEV_R |= bit_mask;
            }
            else
            {
                GPIO_PORTF_IEV_R &= ~(bit_mask);
            }
        }
        GPIO_PORTF_ICR_R |= bit_mask;
        GPIO_PORTF_IM_R |= bit_mask;
        NVIC_PRI7_R = (NVIC_PRI7_R & 0xFF1FFFFF) | 0x00A00000; /*  priority 5 */
        NVIC_EN0_R = 0x40000000; /*  Enable interrupt 30 in NVIC */
    }
}

void GPIOPortF_Init(void)
{
    SYSCTL_RCGC2_R |= 0x00000020; /* 1) activate clock for PortF */
    GPIO_PORTF_LOCK_R = 0x4C4F434B; /* 2) unlock GPIO PortF */
    GPIO_PORTF_CR_R = 0x1F; /* allow changes to PF4-0 */
    GPIO_PORTF_AMSEL_R = 0x00; /* 3) disable analog on PF */
    GPIO_PORTF_PCTL_R = 0x00000000; /* 4) PCTL GPIO on PF4-0 */
    GPIO_PORTF_DIR_R = 0x0E; /* 5) PF4,PF0 in, PF3-1 out */
    GPIO_PORTF_AFSEL_R = 0x00; /* 6) disable alt funct on PF7-0 */
  //  GPIO_PORTF_PUR_R = 0x11; /* enable pull-up on PF0 and PF4 */
    GPIO_PORTF_DEN_R = 0x1F; /* 7) enable digital I/O on PF4-0 */

//    GPIO_PORTF_IS_R &= ~0x10; /*  PF4 is edge-sensitive */
//    GPIO_PORTF_IBE_R &= ~0x10; /*  PF4 is not both edges */
//    GPIO_PORTF_IEV_R &= ~0x10; /*  PF4 falling edge event */
//    GPIO_PORTF_ICR_R = 0x10; /*  Clear flag4 */
//    GPIO_PORTF_IM_R |= 0x10; /*  arm interrupt on PF4 */
//    NVIC_PRI7_R = (NVIC_PRI7_R & 0xFF1FFFFF) | 0x00A00000; /*  priority 5 */
//    NVIC_EN0_R = 0x40000000; /*  Enable interrupt 30 in NVIC */

//EnableInterrupts(); /* Enable global Interrupt flag (I) */
}

int digitalRead(int pin)
{
    return ((GPIO_PORTF_DATA_R >> pin) & 0x01);
}

void UART_OutChar(char data)
{
      while((UART0_FR_R & UART_FR_TXFF) != 0)
          ;
      UART0_DR_R = data;
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

char *itoa (int value, char *result, int base)
{
    // check that the base if valid
    if (base < 2 || base > 36) { *result = '\0'; return result; }

    char* ptr = result, *ptr1 = result, tmp_char;
    int tmp_value;

    do {
        tmp_value = value;
        value /= base;
        *ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz" [35 + (tmp_value - value * base)];
    } while ( value );

    // Apply negative sign
    if (tmp_value < 0) *ptr++ = '-';
    *ptr-- = '\n';
    while (ptr1 < ptr) {
        tmp_char = *ptr;
        *ptr--= *ptr1;
        *ptr1++ = tmp_char;
    }
    return result;
}

void SerialPrintInt(int integer)
{
    char buffer[20];
    itoa(integer, buffer, 10);
    print_line(buffer);
}



void GPIOPortF_Handler(void)
{
    int MSB = digitalRead(0); //MSB = most significant bit
    int LSB = digitalRead(4); //LSB = least significant bit

    int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
    int sum  = (lastEncoded << 2) | encoded; //adding it to the previous encoded value

    if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue ++;
    if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue --;

    lastEncoded = encoded; //store this value for next time
    volatile int readback;
        if(outputA)
        {
            GPIO_PORTF_ICR_R = 0x01;
        }
        else if(outputB)
        {
            GPIO_PORTF_ICR_R = 0x10;
        }
    readback = GPIO_PORTF_ICR_R; /* a read to force clearing of interrupt flag */
}

int main(void)
{
    UART_Init();
    GPIOPortF_Init(); /* initialize GPIO Port F interrupt */
    attachInterrupt('F', 0, CHANGE);
    attachInterrupt('F', 4, CHANGE);
    EnableInterrupts(); /* Enable global Interrupt flag (I) */
    int i = 0;
    while (1)
    {
        SerialPrintInt(encoderValue);
        for(i = 0; i < 32; i++)
        {
        }
    }
}

/*********** DisableInterrupts ***************
 *
 * disable interrupts
 *
 * inputs:  none
 * outputs: none
 */

void DisableInterrupts(void)
{
    __asm ("    CPSID  I\n");
}

/*********** EnableInterrupts ***************
 *
 * enable interrupts
 *
 * inputs:  none
 * outputs: none
 */

void EnableInterrupts(void)
{
    __asm ("    CPSIE  I\n");
}

/*********** WaitForInterrupt ************************
 *
 * go to low power mode while waiting for the next interrupt
 *
 * inputs:  none
 * outputs: none
 */

void WaitForInterrupt(void)
{
    __asm ("    WFI\n");
}

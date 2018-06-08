#include <stdint.h>
#include "inc/tm4c123gh6pm.h"

#define RISING  0
#define FALLING 1
#define LOW     2
#define HIGH    3
#define CHANGE 4

void DisableInterrupts(void);
void EnableInterrupts(void);
void WaitForInterrupt(void);
void GPIOPortF_Init(void);
void GPIOPortF_Handler(void);
void attachInterrupt(char port, int pin, int mode);

void attachInterrupt(char port, int pin, int mode)
{
    int int_sense = 0, int_event = 0;
    int bit_mask = 1 << pin;
    if (mode != CHANGE)
    {
        int_event = mode & 0x01;
        int_sense = (mode >> 1) & 0x01;
    }
    if(port == 'F')
    {
        if(int_sense)// Selecting pin is edge sensitive or level sensitive
        {
            GPIO_PORTF_IS_R |= bit_mask;
        }
        else
        {
            GPIO_PORTF_IS_R &= ~(bit_mask);
        }
        if(mode == CHANGE)
        {
            GPIO_PORTF_IBE_R |= (bit_mask);
        }
        else
        {
            GPIO_PORTF_IBE_R &= ~(bit_mask);
            if(int_event)
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
    GPIO_PORTF_PUR_R = 0x11; /* enable pull-up on PF0 and PF4 */
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

void GPIOPortF_Handler(void)
{
    volatile int readback;
    volatile int i = 0, int_pin, SR = GPIO_PORTF_MIS_R;
    for(i = 0; i < 8; i++)  //To determine which pin triggered this interrupt.
    {
        if((SR >> i)& 0x01)
        {
            int_pin = i;
            break;
        }
    }
    if(i == 0)
    {
        GPIO_PORTF_DATA_R ^= (1 << 1); /* toggle Blue LED */
    }
    else if(i == 4)
    {
        GPIO_PORTF_DATA_R ^= (1 << 2); /* toggle Blue LED */
    }
    GPIO_PORTF_ICR_R = 0x11; /* clear PF4 int */
    readback = GPIO_PORTF_ICR_R; /* a read to force clearing of interrupt flag */
}

int main(void)
{
    GPIOPortF_Init(); /* initialize GPIO Port F interrupt */
    attachInterrupt('F',4,FALLING);
    attachInterrupt('F',0,CHANGE);
    EnableInterrupts(); /* Enable global Interrupt flag (I) */
    while (1)
    {
 //      WaitForInterrupt();
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

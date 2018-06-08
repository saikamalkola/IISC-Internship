/*
 * PLL_Config.c
 *
 *  Created on: 10-May-2018
 *      Author: kamal
 */

#include <stdint.h>
#include "inc/tm4c123gh6pm.h"
#include "GPIO.c"

#define SYSCTL_RIS_PLLLRIS      0x00000040  /* PLL Lock Raw Interrupt Status */
#define SYSCTL_RCC_XTAL_M       0x000007C0  /* Crystal Value */
#define SYSCTL_RCC_XTAL_6MHZ    0x000002C0  /* 6 MHz Crystal */
#define SYSCTL_RCC_XTAL_8MHZ    0x00000380  /* 8 MHz Crystal */
#define SYSCTL_RCC_XTAL_16MHZ   0x00000540  /* 16 MHz Crystal */
#define SYSCTL_RCC2_USERCC2     0x80000000  /* Use RCC2 */
#define SYSCTL_RCC2_DIV400      0x40000000  /* Divide PLL as 400 MHz vs. 200 MHz */
//#define SYSCTL_RCC2_SYSDIV2_M   0x1FC00000  /* System Clock Divisor 2 */
#define SYSCTL_RCC2_PWRDN2      0x00002000  /* Power-Down PLL 2 */
#define SYSCTL_RCC2_BYPASS2     0x00000800  /* PLL Bypass 2 */
#define SYSCTL_RCC2_OSCSRC2_M   0x00000070  /* Oscillator Source 2 */
#define SYSCTL_RCC2_OSCSRC2_MO  0x00000000  /* MOSC */

void PLL_Init(uint32_t freq)
{
    freq = (400 / freq) - 1;
    SYSCTL_RCC2_R |= SYSCTL_RCC2_USERCC2; //configure the system to use RCC2 for advanced features
    SYSCTL_RCC2_R |= SYSCTL_RCC2_BYPASS2;     //bypass PLL while initializing
    SYSCTL_RCC_R &= ~SYSCTL_RCC_XTAL_M;       //clear XTAL field
    SYSCTL_RCC_R += SYSCTL_RCC_XTAL_16MHZ;    //configure for 16 MHz crystal
    SYSCTL_RCC2_R &= ~SYSCTL_RCC2_OSCSRC2_M;  //clear oscillator source field
    SYSCTL_RCC2_R += SYSCTL_RCC2_OSCSRC2_MO; //configure for main oscillator source
    SYSCTL_RCC2_R &= ~SYSCTL_RCC2_PWRDN2;     //activate PLL by clearing PWRDN
    SYSCTL_RCC2_R |= SYSCTL_RCC2_DIV400;      //use 400 MHz PLL
    SYSCTL_RCC2_R &= ~(SYSCTL_RCC2_SYSDIV2_M);      //clear SYSDIV bits
    SYSCTL_RCC2_R |= (freq << 22);           //Set SYSDIV for required frequency
    while ((SYSCTL_RIS_R & SYSCTL_RIS_PLLLRIS) == 0)
        ;             //wait for the PLL to lock by polling PLLLRIS
    SYSCTL_RCC2_R &= ~SYSCTL_RCC2_BYPASS2; //enable use of PLL by clearing BYPASS
}

void Delay(unsigned long ulCount)
{
    __asm ( "loop:    subs    r0, #1\n"
            "    bne     loop\n");
}

int main()
{
    PLL_Init(80);   //Initialize Runmode System Clock as 80Mhz
    init_PORTF();
    while (1)
    {
        digitalWrite(blue_led, 1);
        Delay(13333333); /* delay ~0.5 sec at 80 MHz */
        digitalWrite(blue_led, 0);
        Delay(13333333); /* delay ~0.5 sec at 80 MHz */
    }
}

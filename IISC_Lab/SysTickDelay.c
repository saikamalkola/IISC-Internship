/*
 * SysTickDelay.c
 *Implemented delay_ms()
 *Implemented delay_us()
 *Using Systick Timer
 *  Created on: 10-May-2018
 *      Author: kamal
 */

#include "GPIO.c"
#include <stdint.h>
#include "inc/tm4c123gh6pm.h"

#define NVIC_ST_CTRL_COUNT 0x00010000

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

void init_SysTick(void)
{
    NVIC_ST_CTRL_R = 0;   //disable Timer while setting it up
    NVIC_ST_RELOAD_R = 0x00FFFFFF; //Initializing Timer Reload with maximum value it can hold
    NVIC_ST_CURRENT_R = 0; //Clearing Current value of Timer
    NVIC_ST_CTRL_R = 0x00000005;    //Enable SysTick timer and its clock source
}

void delay_us(uint32_t delay)
{
    NVIC_ST_RELOAD_R = delay * 16 - 1;
    NVIC_ST_CURRENT_R = 0;
    while((NVIC_ST_CTRL_R & NVIC_ST_CTRL_COUNT) == 0);
}

void delay_ms(uint32_t delay)
{
    NVIC_ST_RELOAD_R = delay * 16000 - 1;
    NVIC_ST_CURRENT_R = 0;
    while((NVIC_ST_CTRL_R & NVIC_ST_CTRL_COUNT) == 0);
}

int main()
{
    init_PORTF();
    init_SysTick();
    while(1)
    {
        //digitalWrite(blue_led, 1);

        delay_us(1000000);
        digitalWrite(blue_led, 0);
        delay_us(1000000);
    }
}

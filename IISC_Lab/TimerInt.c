#include <stdint.h>
#include "inc/tm4c123gh6pm.h"

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

void DisableInterrupts(void);
void EnableInterrupts(void);

int a = 0;

void TIMER0_TA_Handler(void)
{
    volatile int read_back = 0;
    GPIO_PORTF_DATA_R ^= (1 << 1);  //Toggle blue LED
    TIMER0_ICR_R = 0x01;
    read_back = TIMER0_ICR_R;
}

void init_timer(int time_ms)
{
    SYSCTL_RCGCTIMER_R |= 0x01;
    TIMER0_CTL_R &= ~(TAEN);    //Disabling Timer A
    TIMER0_CFG_R &= ~(0x07);    //Clearing D2:D0 bits
    TIMER0_CFG_R |= BIT16_MODE; //Configuring Timer 0 as 16 bit timer
    TIMER0_TAMR_R &= ~(0x03);   //Clearing D1:D0 bits
    TIMER0_TAMR_R |= PERIODIC;  //Periodic mode
    TIMER0_TAMR_R |= COUNT_DOWN;    //Count down mode
    uint32_t load_register = (time_ms * 16 * 1000 / 256) - 1; //Calculating ticks required at 16MHz
    TIMER0_TAPR_R = 0xFF;
    TIMER0_TAILR_R = load_register;
    TIMER0_ICR_R = 0x01;
    TIMER0_CTL_R |= TAEN;    //Enabling Timer A
    TIMER0_IMR_R |= TATORIM;    //Enabling Interrupt Timerout
    NVIC_PRI4_R = (NVIC_PRI4_R & 0x1FFFFFFF) | 0xA0000000;
    NVIC_EN0_R = (1 << 19);
    EnableInterrupts();
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
}

int main()
{
    GPIOPortF_Init();   // Initialising PORTF
    init_timer(1000);   //Initialising Timer0 - Timer A at 100ms
    while(1){

    }
}

void DisableInterrupts(void)
{
    __asm ("    CPSID  I\n");
}

void EnableInterrupts(void)
{
    __asm ("    CPSIE  I\n");
}

void timer0A_delayMs(int ttime)
{
    int i;

    SYSCTL_RCGCTIMER_R |= 1;     /* enable clock to Timer Block 0 */

    TIMER0_CTL_R = 0;            /* disable Timer before initialization */
    TIMER0_CFG_R = 0x04;         /* 16-bit option */
    TIMER0_TAMR_R = 0x02;        /* periodic mode and down-counter */
    TIMER0_TAILR_R = 16000 - 1;  /* Timer A interval load value register */
    TIMER0_ICR_R = 0x1;          /* clear the TimerA timeout flag*/
    TIMER0_CTL_R |= 0x01;        /* enable Timer A after initialization */

    for(i = 0; i < ttime; i++) {
        while ((TIMER0_RIS_R & 0x1) == 0)
            ;                    /* wait for TimerA timeout flag */

        TIMER0_ICR_R = 0x1;      /* clear the TimerA timeout flag */
    }
}

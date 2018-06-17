/* Use PWM to control LED intensity */

/* In the infinite loop, the value of CMPA register is decremented
 * by 100 every 20 ms. The decreasing CMPA value causes the duty cycle
 * to lengthen.
*/

#include <stdint.h>
#include "inc/tm4c123gh6pm.h"

void delayMs(int n);

int main(void)
{
    int x = 15999;

    /* Enable Peripheral Clocks */
    SYSCTL_RCGCPWM_R |= 2;       /* enable clock to PWM1 */
    SYSCTL_RCGCGPIO_R |= 0x20;   /* enable clock to PORTF */
    SYSCTL_RCC_R &= ~0x00100000; /* no pre-divide for PWM clock */

    /* Enable port PF3 for PWM1 M1PWM7 */
    GPIO_PORTF_AFSEL_R = 8;      /* PF3 uses alternate function */
    GPIO_PORTF_PCTL_R &= ~0x0000F000; /* make PF3 PWM output pin */
    GPIO_PORTF_PCTL_R |= 0x00005000;
    GPIO_PORTF_DEN_R |= 8;       /* pin digital */

    PWM1_3_CTL_R = 0;            /* stop counter */
    PWM1_3_GENB_R = 0x0000008C;  /* M1PWM7 output set when reload, */
                                 /* clear when match PWMCMPA */
    PWM1_3_LOAD_R = 16000;       /* set load value for 1kHz (16MHz/16000) */
    PWM1_3_CMPA_R = 15999;       /* set duty cycle to min */
    PWM1_3_CTL_R = 1;            /* start timer */
    PWM1_ENABLE_R = 0x80;        /* start PWM1 ch7 */

    for(;;) {
        x = x - 100;

        if (x <= 0)
            x = 16000;

        PWM1_3_CMPA_R = x;

        delayMs(20);
    }
}


/* delay n milliseconds (16 MHz CPU clock) */
void delayMs(int n)
{
    int i, j;

    for(i = 0 ; i < n; i++)
        for(j = 0; j < 3180; j++)
            {}  /* do nothing for 1 ms */
}

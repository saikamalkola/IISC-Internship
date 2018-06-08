/* pllmain.c
*
*  Tested on TM4C123G LaunchPad
*  Test the PLL function to verify that the system clock is running at the expected rate.
*  Use the debugger if possible or an oscilloscope connected to PF2.
*
*  The #define statement in the file pll.h allows PLL_Init() to initialize the PLL
*  to the desired frequency. When using an oscilloscope to look at LED1,
*  it should be clear to see that the LED flashes about 2 (80/40) times faster
*  with a 80 MHz clock than with a 40 MHz clock.
*
*/

#include <stdint.h>
#include "inc/tm4c123gh6pm.h"
#include "pll.h"

#define GPIO_PORTF2     (*((volatile uint32_t *)0x40025010))

/* delay function for testing which delays 3*ulCount cycles */

void Delay(unsigned long ulCount)
{
    __asm (	"loop:    subs    r0, #1\n"
            "    bne     loop\n");
}

int main(void)
{  
	PLL_Init(Bus80MHz);

	SYSCTL_RCGCGPIO_R |= 0x20;   	/* activate port F */
	while((SYSCTL_PRGPIO_R&0x0020) == 0){};/* ready? */

	GPIO_PORTF_DIR_R |= 0x04;    	/* make PF2 out (PF2 built-in LED) */
	GPIO_PORTF_AFSEL_R &= ~0x04; 	/* regular port function */
	GPIO_PORTF_DEN_R |= 0x04;    	/* enable digital I/O on PF2 */
		                       		/* configure PF4 as GPIO */
	GPIO_PORTF_PCTL_R = (GPIO_PORTF_PCTL_R&0xFFF0FFFF)+0x00000000;
	GPIO_PORTF_AMSEL_R = 0;      	/* disable analog functionality on PF */

	while(1) {
		GPIO_PORTF2 = 0x04;        	/* turn on LED1 (blue) */
		Delay(13333333*2);           	/* delay ~0.5 sec at 80 MHz */
		GPIO_PORTF2 = 0x00;        	/* turn off LED1 (blue) */
		Delay(13333333);           	/* delay ~0.5 sec at 80 MHz */
	}
}

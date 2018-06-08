#include <stdint.h>
#include "inc/tm4c123gh6pm.h"

int val = -1;
void config_SS3()
{
    ADC0_ACTSS_R &= ~(0x08);
    ADC0_EMUX_R |= (0xF000);
    ADC0_SSMUX3_R &= ~(0xF);
    ADC0_SSMUX3_R |= 0xA;
    ADC0_SSCTL3_R &= ~(0xF);
    ADC0_SSCTL3_R |= (1 << 1);
    ADC0_SSCTL3_R |= (1 << 2);
    ADC0_IM_R &= ~(1 << 3);
    ADC0_ACTSS_R |= 0x08;
}

void init_PORTB()
{
    SYSCTL_RCGC2_R |= 0x02; //Enabling clock to PORTB
    GPIO_PORTB_AFSEL_R |= (1 << 4); // Enabling alternate function for PB4
    GPIO_PORTB_DEN_R &= ~(1 << 4);  //Disabling Digital Functionality
    GPIO_PORTB_AMSEL_R |= (1 << 4); //Disabling analog isolation circuitry for PB4
}

void init_ADC()
{
    SYSCTL_RCGCADC_R |= 0x01;   //Enabling ADC0
}

int main()
{
    init_PORTB();   //Initialising PB4 as analog Input
    init_ADC(); //Enabling ADC0
    config_SS3();   //Configuring SS3
    while(1)
    {
        while(!(ADC0_RIS_R & 0x8));
        val = ADC0_SSFIFO3_R & 0xFFF;
        ADC0_ISC_R |= 0x8;
    }
}

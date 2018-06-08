#include <stdint.h>
#include "inc/tm4c123gh6pm.h"

#define INPUT 0
#define OUTPUT 1

//Output Pin definitions
uint8_t led[8] = { 5, 0, 1, 4, 5, 4, 5, 6 };
char led_port[8] = { 'B', 'B', 'B', 'E', 'E', 'B', 'A', 'A' };

uint8_t sel_pin[4] = { 2, 3, 4, 5 };
char sel_port[4] = { 'B', 'B', 'C', 'C' };

void commit_pins(char port, int pin)
{
    switch (port)
    {
    case 'A':
    {
        GPIO_PORTA_CR_R |= (1 << pin);
    }
        break;
    case 'B':
    {
        GPIO_PORTB_CR_R |= (1 << pin);
    }
        break;
    case 'C':
    {
        GPIO_PORTC_CR_R |= (1 << pin);
    }
        break;
    case 'E':
    {
        GPIO_PORTE_CR_R |= (1 << pin);
    }
        break;
    }
}

void pinMode(char port, int pin, int state)
{
    switch (port)
    {
    case 'A':
    {
        if (state)
        {
            GPIO_PORTA_DIR_R |= (1 << pin);
        }
        else
        {
            GPIO_PORTA_DIR_R &= ~(1 << pin);
        }
    }
        break;
    case 'B':
    {
        if (state)
        {
            GPIO_PORTB_DIR_R |= (1 << pin);
        }
        else
        {
            GPIO_PORTB_DIR_R &= ~(1 << pin);
        }
    }
        break;
    case 'C':
    {
        if (state)
        {
            GPIO_PORTC_DIR_R |= (1 << pin);
        }
        else
        {
            GPIO_PORTC_DIR_R &= ~(1 << pin);
        }
    }
        break;
    case 'E':
    {
        if (state)
        {
            GPIO_PORTE_DIR_R |= (1 << pin);
        }
        else
        {
            GPIO_PORTE_DIR_R &= ~(1 << pin);
        }
    }
        break;
    }
}

void enable_DFun()
{
    switch (port)
    {
    case 'A':
    {
        GPIO_PORTA_DEN_R |= (1 << pin);
    }
        break;
    case 'B':
    {
        GPIO_PORTB_DEN_R |= (1 << pin);
    }
        break;
    case 'C':
    {
        GPIO_PORTC_DEN_R |= (1 << pin);
    }
        break;
    case 'E':
    {
        GPIO_PORTE_DEN_R |= (1 << pin);
    }
        break;
    }
}

void init_ports()
{
    SYSCTL_RCGC2_R |= 0x00000017;   //Enable Clock to Port A and Port D
//    GPIO_PORTA_LOCK_R = 0x4C4F434B; //Unlock Port A
//    GPIO_PORTA_LOCK_R = 0x4C4F434B; //Unlock Port A
//    GPIO_PORTA_LOCK_R = 0x4C4F434B; //Unlock Port A
//    GPIO_PORTE_LOCK_R = 0x4C4F434B; //Unlock Port D
    commit_pins();                  //commit pins(to allow changes)
    GPIO_PORTA_AMSEL_R = 0x00;      //disable analog on PA
    GPIO_PORTD_AMSEL_R = 0x00;      //disable analog on PD
    GPIO_PORTA_PCTL_R = 0x00000000; //PCTL GPIO on PA7-4
    GPIO_PORTD_PCTL_R = 0x00000000; //PCTL GPIO on PF3-0
    init_pins();                    //Initializing pins as Inputs or Outputs
    GPIO_PORTA_AFSEL_R = 0x00;      //disable alt funct on PA7-0
    GPIO_PORTD_AFSEL_R = 0x00;      //disable alt funct on PA7-0
    enable_pulldowns();               //Enabling PullUPs for SW1 and SW2
    enable_DFun();                  //Enabling Digital Functionality of Pins
}

void digitalWrite(char port, int pin, int status)
{
    switch (port)
    {
    case 'A':
    {
        if (state)
        {
            GPIO_PORTA_DATA_R |= (1 << pin);
        }
        else
        {
            GPIO_PORTA_DATA_R &= ~(1 << pin);
        }
    }
        break;
    case 'B':
    {
        if (state)
        {
            GPIO_PORTB_DATA_R |= (1 << pin);
        }
        else
        {
            GPIO_PORTB_DATA_R &= ~(1 << pin);
        }
    }
        break;
    case 'C':
    {
        if (state)
        {
            GPIO_PORTC_DATA_R |= (1 << pin);
        }
        else
        {
            GPIO_PORTC_DATA_R &= ~(1 << pin);
        }
    }
        break;
    case 'E':
    {
        if (state)
        {
            GPIO_PORTE_DATA_R |= (1 << pin);
        }
        else
        {
            GPIO_PORTE_DATA_R &= ~(1 << pin);
        }
    }
        break;
    }
}


int main()
{
    init_ports();
    while (1)
    {
        int i = 0, j = 0, k = 0;
        for (i = 0; i < 4; i++)
        {
            enable_row(row[i]);
            for (j = 0; j < 4; j++)
            {
                status[i][j] = digitalRead('A', column[j]);
                if (status[i][j])
                {
                    data = k;
                }
                k++;
            }
        }
    }
}


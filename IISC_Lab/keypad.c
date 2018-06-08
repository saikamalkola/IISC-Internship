#include <stdint.h>
#include "inc/tm4c123gh6pm.h"

#define bit_mask_column 0xF0
#define bit_mask_row 0x0F

//Output Pin definitions
uint8_t row[4] = { 3,2,1,0};
//Input Pin definitions
uint8_t column[4] = { 7,6,5,4};

//Status of keys
uint8_t status[4][4];
uint16_t data = 0;

void commit_pins()
{
    GPIO_PORTA_CR_R |= bit_mask_column;
    GPIO_PORTD_CR_R |= bit_mask_row;
}

void init_pins()
{
    //Inputs
    GPIO_PORTA_DIR_R &= ~(bit_mask_column);

    //Outputs
    GPIO_PORTD_DIR_R |= (bit_mask_row);
}

void enable_pulldowns()
{
    GPIO_PORTA_PDR_R |= (bit_mask_column);
}

void enable_DFun()
{
    GPIO_PORTA_DEN_R |= (bit_mask_column);
    GPIO_PORTD_DEN_R |= (bit_mask_row);
}

void init_ports()
{
    SYSCTL_RCGC2_R |= 0x00000009;   //Enable Clock to Port A and Port D
    GPIO_PORTA_LOCK_R = 0x4C4F434B; //Unlock Port A
    GPIO_PORTD_LOCK_R = 0x4C4F434B; //Unlock Port D
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
    if (port == 'A')
    {
        if (status)
        {
            GPIO_PORTA_DATA_R |= (1 << pin);
        }
        else
        {
            GPIO_PORTA_DATA_R &= ~(1 << pin);
        }
    }
    else if (port == 'D')
    {
        if (status)
        {
            GPIO_PORTD_DATA_R |= (1 << pin);
        }
        else
        {
            GPIO_PORTD_DATA_R &= ~(1 << pin);
        }
    }
}

int digitalRead(char port, int pin)
{
    if (port == 'A')
    {
        return ((GPIO_PORTA_DATA_R >> pin) & 0x01);
    }
}

void enable_row(int row)
{
    int i = 0;
    for (i = 0; i < 4; i++)
    {
        if (i == row)
        {
            digitalWrite('D', row, 1);
        }
        else
        {
            digitalWrite('D', i, 0);
        }
    }
}

int main()
{
    init_ports();
    while (1)
    {
        int i = 0, j = 0, k = 0;
        for(i = 0; i < 4; i++)
        {
            enable_row(row[i]);
            for(j = 0; j < 4; j++)
            {
                status[i][j] = digitalRead('A',column[j]);
                if(status[i][j])
                {
                    data = k;
                }
                k++;
            }
        }
    }
}



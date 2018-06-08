#include <stdint.h>
#include "inc/tm4c123gh6pm.h"

void keypad_init(void);
unsigned char keypad_kbhit(void);
void keypad_findkey(void);

char data[16] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B',
                  'C', 'D', 'E', 'F' };

/* this function initializes the ports connected to the keypad */
void keypad_init(void)
{
    SYSCTL_RCGC2_R |= 0x04; /* enable clock to GPIOC (COLUMN) */
    SYSCTL_RCGC2_R |= 0x20; /* enable clock to GPIOF (ROW) */

    GPIO_PORTF_DIR_R |= 0x0F; /* set row pins 3-0 as output */
    GPIO_PORTF_DEN_R |= 0x0F; /* set row pins 3-0 as digital pins */
    GPIO_PORTF_ODR_R |= 0x0F; /* set row pins 3-0 as open drain */

    GPIO_PORTC_DIR_R &= ~0xF0; /* set column pin 7-4 as input */
    GPIO_PORTC_DEN_R |= 0xF0; /* set column pin 7-4 as digital pins */
    GPIO_PORTC_PUR_R |= 0xF0; /* enable pull-ups for pin 7-4 */
}

/* This is a non-blocking function. */
/* If a key is pressed, it returns 1. Otherwise, it returns a 0 (not ASCII 0).*/
unsigned char keypad_kbhit(void)
{
    int col;

    /* check to see any key pressed */
    GPIO_PORTF_DATA_R = 0; /* enable all rows */
    col = GPIO_PORTC_DATA_R & 0xF0; /* read all columns */

    if (col == 0xF0)
        return 0; /* no key pressed */
    else
        return 1; /* a key is pressed */
}

/* Find the key pressed */
void keypad_findkey(void)
{
    int col, code, i;

    GPIO_PORTF_DATA_R &= ~0x01;
    col = GPIO_PORTC_DATA_R & 0xF0; /* read all columns */

    if (col == 0xF0)
    {

        for (i = 1; i < 4; i++)
        {
            /*Shift the zero leftwards, making all bits before it ones*/
            GPIO_PORTF_DATA_R = ~GPIO_PORTF_DATA_R << 1;
            GPIO_PORTF_DATA_R = ~GPIO_PORTF_DATA_R;

            col = GPIO_PORTC_DATA_R & 0xF0; /* read all columns */

            if (col != 0xF0)
                break;
        }

        if (i == 1)
        {
            if (col == ~0x80)
                code = data[4];
            else if (col == ~0x40)
                code = data[5];
            else if (col == ~0x20)
                code = data[6];
            else if (col == ~0x10)
                code = data[7];
        }

        if (i == 2)
        {
            if (col == ~0x80)
                code = data[8];
            else if (col == ~0x40)
                code = data[9];
            else if (col == ~0x20)
                code = data[10];
            else if (col == ~0x10)
                code = data[11];
        }

        if (i == 3)
        {
            if (col == ~0x80)
                code = data[12];
            else if (col == ~0x40)
                code = data[13];
            else if (col == ~0x20)
                code = data[14];
            else if (col == ~0x10)
                code = data[15];
        }

    }

    else if (col == ~0x80)
        code = data[0];
    else if (col == ~0x40)
        code = data[1];
    else if (col == ~0x20)
        code = data[2];
    else if (col == ~0x10)
        code = data[3];

}

/* delay n milliseconds (16 MHz CPU clock) */
void delayMs(int n)
{
    int i, j;
    for (i = 0; i < n; i++)
        for (j = 0; j < 3180; j++)
            ;
    /* do nothing for 1 ms */
}

int main(void)
{
    // unsigned char key;

    keypad_init();

    while (1)
    {
        if (keypad_kbhit() != 0) /* if a key is pressed */
            delayMs(10); /* wait for a while */
        keypad_findkey();
    }

}

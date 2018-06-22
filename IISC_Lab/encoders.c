/*
 * encoders.c
 *
 *  Created on: 16-Jun-2018
 *      Author: kamal
 */

#include <stdbool.h>
#include <stdint.h>
#include "inc/tm4c123gh6pm.h"
#include "encoders.h"
#include "odometry.h"
#include "Ultrasonic.h"

volatile long encoder_value[4] = { 0, 0, 0, 0 };
volatile int lastEncoded[4] = { 0, 0, 0, 0 };

extern volatile float distance[4];

volatile unsigned long echo_start[4], echo_end[4];

void GPIOPortC_Handler(void)
{
    int MSB = encoder_F_A; //MSB = most significant bit
    int LSB = encoder_F_B; //LSB = least significant bit

    int encoded = (MSB << 1) | LSB; //converting the 2 pin value to single number
    int sum = (lastEncoded[0] << 2) | encoded; //adding it to the previous encoded value

    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
        encoder_value[0]++;
    if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
        encoder_value[0]--;

    lastEncoded[0] = encoded; //store this value for next time

    volatile int readback = 0;
    readback = readback;
    if (GPIO_PORTC_MIS_R & (1 << 6))
    {
        GPIO_PORTC_ICR_R |= (1 << 6);
    }
    else if (GPIO_PORTC_MIS_R & (1 << 7))
    {
        GPIO_PORTC_ICR_R |= (1 << 7);
    }
    readback = GPIO_PORTC_ICR_R; //a read to force clearing of interrupt flag
}

void GPIOPortE_Handler(void)
{
    int MSB = encoder_B_A; //MSB = most significant bit
    int LSB = encoder_B_B; //LSB = least significant bit

    int encoded = (MSB << 1) | LSB; //converting the 2 pin value to single number
    int sum = (lastEncoded[1] << 2) | encoded; //adding it to the previous encoded value

    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
        encoder_value[1]++;
    if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
        encoder_value[1]--;

    lastEncoded[1] = encoded; //store this value for next time

    volatile int readback = 0;
    readback = readback;
    if (GPIO_PORTE_MIS_R & (1 << 2))
    {
        GPIO_PORTE_ICR_R |= (1 << 2);
    }
    else if (GPIO_PORTE_MIS_R & (1 << 3))
    {
        GPIO_PORTE_ICR_R |= (1 << 3);
    }
    readback = GPIO_PORTE_ICR_R; //a read to force clearing of interrupt flag
}

void GPIOPortD_Handler(void)
{
    volatile int i = 0, int_pin, SR = GPIO_PORTD_MIS_R;
    for (i = 0; i < 8; i++)  //To determine which pin triggered this interrupt.
    {
        if ((SR >> i) & 0x01)
        {
            int_pin = i;
            break;
        }
    }
    if (int_pin >= 4)
    {
        int MSB = encoder_L_A; //MSB = most significant bit
        int LSB = encoder_L_B; //LSB = least significant bit

        int encoded = (MSB << 1) | LSB; //converting the 2 pin value to single number
        int sum = (lastEncoded[2] << 2) | encoded; //adding it to the previous encoded value

        if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
            encoder_value[2]++;
        if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
            encoder_value[2]--;

        lastEncoded[2] = encoded; //store this value for next time
    }
    else
    {
        switch (echo_state(int_pin))
        {
        case 1:
        {
//            TIMER1_CTL_R &= ~(TAEN);
//            TIMER1_TAPR_R = 0x10;
//            TIMER1_TAILR_R = 0xFFFF;
//            TIMER1_CTL_R |= TAEN;
            echo_start[int_pin] = micros();
        }
            break;
        case 0:
        {
            echo_end[int_pin] = micros();
            distance[int_pin] = (echo_end[int_pin] - echo_start[int_pin]) * 0.17;
            if(distance[int_pin] > 4000)
            {
                distance[int_pin] = -1;
            }
//            TIMER1_CTL_R &= ~(TAEN);
        }
            break;
        }
    }
    volatile int readback = 0;
    readback = readback;
    GPIO_PORTD_ICR_R |= (1 << int_pin);
    readback = GPIO_PORTD_ICR_R; //a read to force clearing of interrupt flag
}

void GPIOPortF_Handler(void)
{
    int MSB = encoder_R_A; //MSB = most significant bit
    int LSB = encoder_R_B; //LSB = least significant bit

    int encoded = (MSB << 1) | LSB; //converting the 2 pin value to single number
    int sum = (lastEncoded[3] << 2) | encoded; //adding it to the previous encoded value

    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
        encoder_value[3]++;
    if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
        encoder_value[3]--;

    lastEncoded[3] = encoded; //store this value for next time

    volatile int readback = 0;
    readback = readback;
    if (GPIO_PORTF_MIS_R & (1 << 2))
    {
        GPIO_PORTF_ICR_R |= (1 << 2);
    }
    else if (GPIO_PORTF_MIS_R & (1 << 4))
    {
        GPIO_PORTF_ICR_R |= (1 << 4);
    }
    readback = GPIO_PORTF_ICR_R; //a read to force clearing of interrupt flag
}

void init_encoders()
{
    //Front Encoder ---> PC6 and PC7
    SYSCTL_RCGC2_R |= (1 << 2); //Enabling Clock to GPIO PORTC
    GPIO_PORTC_LOCK_R = 0x4C4F434B; //unlock GPIO PORTC
    GPIO_PORTC_CR_R |= ((1 << 6) | (1 << 7)); //allow changes to PC6 and PC7
    GPIO_PORTC_AMSEL_R &= ~((1 << 6) | (1 << 7)); //disable analog on PC6 and PC7
    GPIO_PORTC_PCTL_R &= ~((0xF << 24) | (0xF << 28)); //PCTL GPIO on PC6 and PC7
    GPIO_PORTC_DIR_R &= ~((1 << 6) | (1 << 7)); //PC6 and PC7 as Inputs
    GPIO_PORTC_AFSEL_R &= ~((1 << 6) | (1 << 7)); //disable alternate function on PC6 and PC7
    GPIO_PORTC_DEN_R |= ((1 << 6) | (1 << 7)); //enable digital function on PC6 and PC7

    //Interrupt Settings
    GPIO_PORTC_IBE_R |= ((1 << 6) | (1 << 7)); //Enabling Both Edge Interrupts on pins PC6 and PC7
    GPIO_PORTC_ICR_R |= ((1 << 6) | (1 << 7)); //Clearing interrupt flags of PC6 and PC7
    GPIO_PORTC_IM_R |= ((1 << 6) | (1 << 7));    //Unmask PC6 and PC7 interrupts

    //IRQ Number of PORTC = 2
    NVIC_PRI0_R &= ~(0x7 << 21);
    NVIC_PRI0_R |= (0x5 << 21); //Priority level 5
    NVIC_EN0_R |= (1 << 2); //Enable interrupt 2 in NVIC

    //Back Encoder ---> PE2 and PE3
    SYSCTL_RCGC2_R |= (1 << 4); //Enabling Clock to GPIO PORTE
    GPIO_PORTE_LOCK_R = 0x4C4F434B; //unlock GPIO PORTE
    GPIO_PORTE_CR_R |= ((1 << 2) | (1 << 3)); //allow changes to PE2 and PE3
    GPIO_PORTE_AMSEL_R &= ~((1 << 2) | (1 << 3)); //disable analog on PE2 and PE3
    GPIO_PORTE_PCTL_R &= ~((0xF << 8) | (0xF << 12)); //PCTL GPIO on PE2 and PE3
    GPIO_PORTE_DIR_R &= ~((1 << 2) | (1 << 3)); //PE2 and PE3 as Inputs
    GPIO_PORTE_AFSEL_R &= ~((1 << 2) | (1 << 3)); //disable alternate function on PE2 and PE3
    GPIO_PORTE_DEN_R |= ((1 << 2) | (1 << 3)); //enable digital function on PE2 and PE3
    //GPIO_PORTE_PUR_R |= ((1 << 2) | (1 << 3));

    //Interrupt Settings
    GPIO_PORTE_IBE_R |= ((1 << 2) | (1 << 3)); //Enabling Both Edge Interrupts on pins PE2 and PE3
    GPIO_PORTE_ICR_R |= ((1 << 2) | (1 << 3)); //Clearing interrupt flags of PE2 and PE3
    GPIO_PORTE_IM_R |= ((1 << 2) | (1 << 3));    //Unmask PE2 and PE3 interrupts

    //IRQ Number of PORTE = 4
    NVIC_PRI1_R &= ~(0x7 << 5);
    NVIC_PRI1_R |= (0x5 << 5); //Priority level 5
    NVIC_EN0_R |= (1 << 4); //Enable interrupt 4 in NVIC

    //Left Encoder ---> PD6 and PD7
    SYSCTL_RCGC2_R |= (1 << 3); //Enabling Clock to GPIO PORTD
    GPIO_PORTD_LOCK_R = 0x4C4F434B; //unlock GPIO PORTD
    GPIO_PORTD_CR_R |= ((1 << 6) | (1 << 7)); //allow changes to PD6 and PD7
    GPIO_PORTD_AMSEL_R &= ~((1 << 6) | (1 << 7)); //disable analog on PD6 and PD7
    GPIO_PORTD_PCTL_R &= ~((0xF << 24) | (0xF << 28)); //PDTL GPIO on PD6 and PD7
    GPIO_PORTD_DIR_R &= ~((1 << 6) | (1 << 7)); //PD6 and PD7 as Inputs
    GPIO_PORTD_AFSEL_R &= ~((1 << 6) | (1 << 7)); //disable alternate function on PD6 and PD7
    GPIO_PORTD_DEN_R |= ((1 << 6) | (1 << 7)); //enable digital function on PD6 and PD7

    //Interrupt Settings
    GPIO_PORTD_IBE_R |= ((1 << 6) | (1 << 7)); //Enabling Both Edge Interrupts on pins PD6 and PD7
    GPIO_PORTD_ICR_R |= ((1 << 6) | (1 << 7)); //Clearing interrupt flags of PD6 and PD7
    GPIO_PORTD_IM_R |= ((1 << 6) | (1 << 7));   //Unmask PD 6 and PD7 interrupts

    //IRQ Number of PORTD = 3
    NVIC_PRI0_R &= ~(0x7 << 29);
    NVIC_PRI0_R |= (0x5 << 29); //Priority level 5
    NVIC_EN0_R |= (1 << 3); //Enable interrupt 3 in NVIC

    //Right Encoder ---> PF2 and PF4
    SYSCTL_RCGC2_R |= (1 << 5); //Enabling Clock to GPIO PORTE
    GPIO_PORTF_LOCK_R = 0x4C4F434B; //unlock GPIO PORTE
    GPIO_PORTF_CR_R |= ((1 << 2) | (1 << 4)); //allow changes to PF2 and PF3
    GPIO_PORTF_AMSEL_R &= ~((1 << 2) | (1 << 4)); //disable analog on PF2 and PF3
    GPIO_PORTF_PCTL_R &= ~((0xF << 8) | (0xF << 16)); //PCTL GPIO on PF2 and PF3
    GPIO_PORTF_DIR_R &= ~((1 << 2) | (1 << 4)); //PF2 and PF3 as Inputs
    GPIO_PORTF_AFSEL_R &= ~((1 << 2) | (1 << 4)); //disable alternate function on PF2 and PF3
    GPIO_PORTF_DEN_R |= ((1 << 2) | (1 << 4)); //enable digital function on PF2 and PF3
    //GPIO_PORTF_PUR_R |= ((1 << 2) | (1 << 3));
    //Interrupt Settings
    GPIO_PORTF_IBE_R |= ((1 << 2) | (1 << 4)); //Enabling Both Edge Interrupts on pins PF2 and PF3
    GPIO_PORTF_ICR_R |= ((1 << 2) | (1 << 4)); //Clearing interrupt flags of PF2 and PF3
    GPIO_PORTF_IM_R |= ((1 << 2) | (1 << 4));    //Unmask PF2 and PF3 interrupts

    //IRQ Number of PORTE = 30
    NVIC_PRI7_R &= ~(0x7 << 21);
    NVIC_PRI7_R |= (0x5 << 21); //Priority level 5
    NVIC_EN0_R |= (1 << 30); //Enable interrupt 30 in NVIC
}

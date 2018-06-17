/*
 * encoders.h
 *
 *  Created on: 16-Jun-2018
 *      Author: kamal
 */

#ifndef ENCODERS_H_
#define ENCODERS_H_
#endif /* ENCODERS_H_ */

void init_encoders(void);

//Interrupt Handler Declarations
void GPIOPortC_Handler(void);
void GPIOPortE_Handler(void);
void GPIOPortD_Handler(void);
void GPIOPortF_Handler(void);
/*
 *Encoder Connections
 *     Front A ---> PC6 | Back A ---> PE2 | Left A ---> PD6 | Right A ---> PF2
 *     Front B ---> PC7 | Back B ---> PE3 | Left B ---> PD7 | Right B ---> PF4
 */

#define encoder_F_A ((GPIO_PORTC_DATA_R >> 6) & 0x01)
#define encoder_F_B ((GPIO_PORTC_DATA_R >> 7) & 0x01)

#define encoder_B_A ((GPIO_PORTE_DATA_R >> 2) & 0x01)
#define encoder_B_B ((GPIO_PORTE_DATA_R >> 3) & 0x01)

#define encoder_L_A ((GPIO_PORTD_DATA_R >> 6) & 0x01)
#define encoder_L_B ((GPIO_PORTD_DATA_R >> 7) & 0x01)

#define encoder_R_A ((GPIO_PORTF_DATA_R >> 2) & 0x01)
#define encoder_R_B ((GPIO_PORTF_DATA_R >> 4) & 0x01)

#include <stdbool.h>
#include <stdint.h>
#include "inc/tm4c123gh6pm.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#define CR   0x0D

//Output Pin definitions
#define red_led 1
#define green_led 3
#define blue_led 2

//Input Pin definitions
#define SW1 4
#define SW2 0

//UART Definitions
/* U0Rx receive connected to PA0 */
/* U0Tx transmit connected to PA1 */

#define UART_FR_TXFF            0x00000020  /* UART Transmit FIFO Full */
#define UART_FR_RXFE            0x00000010  /* UART Receive FIFO Empty */
#define UART_LCRH_WLEN_8        0x00000060  /* 8 bit word length */
#define UART_LCRH_FEN           0x00000010  /* UART Enable FIFOs */
#define UART_CTL_UARTEN         0x00000001  /* UART Enable */
#define SYSCTL_RCGC1_UART0      0x00000001  /* UART0 Clock Gating Control */
#define SYSCTL_RCGC2_GPIOA      0x00000001  /* port A Clock Gating Control */

xQueueHandle qh = 0;

void UART_Init(void)
{
    SYSCTL_RCGCUART_R |= 0x02; /* activate UART0 */
    SYSCTL_RCGCGPIO_R |= 0x02; /* activate port A */

    int k = 0;
    for(k = 0; k < 10; k++);
    UART1_CTL_R &= ~UART_CTL_UARTEN; /* disable UART */
    UART1_IBRD_R = 8; /* IBRD = int(16,000,000 / (16 * 115,200)) = int(8.680) */
    UART1_FBRD_R = 44; /* FBRD = round(0.5104 * 64 ) = 44 */
    /* 8 bit word length (no parity bits, one stop bit, FIFOs) */
    UART1_LCRH_R = (UART_LCRH_WLEN_8 | UART_LCRH_FEN);
    UART1_CTL_R |= UART_CTL_UARTEN; /* enable UART */
    GPIO_PORTB_AFSEL_R |= 0x03; /* enable alt funct on PA1-0 */
    GPIO_PORTB_DEN_R |= 0x03; /* enable digital I/O on PA1-0 */
    GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R & 0xFFFFFF00) + 0x00000011; /* configure PA1-0 as UART */
    GPIO_PORTB_AMSEL_R &= ~0x03; /* disable analog functionality on PA */
}

int SerialAvailable()
{
    if ((UART1_FR_R & UART_FR_RXFE) == 0)
    {
        return 1;
    }

    else
    {
        return -1;
    }
}

char UART_InChar(void)
{
    while ((UART1_FR_R & UART_FR_RXFE) != 0)
        ;
    return ((char) (UART1_DR_R & 0xFF));
}

void UART_OutChar(char data)
{
    while ((UART1_FR_R & UART_FR_TXFF) != 0)
        ;
    UART1_DR_R = data;
}

void read_line(char *buffer)
{
    int new_line = 0, count = 0;
    while (!new_line)
    {
        buffer[count] = UART_InChar();
        if (buffer[count] == '\n')
        {
            buffer[count] = '\n';
            new_line = 1;
        }
        count++;
    }
}

void print_line(char *data)
{
    int k = 0;
    while (1)
    {
        if (data[k] == '\n')
        {
            UART_OutChar(data[k]);
            break;
        }
        UART_OutChar(data[k]);
        k++;
    }
}

void commit_pins()
{
    GPIO_PORTF_CR_R |= ((1 << red_led) | (1 << green_led) | (1 << blue_led)
            | (1 << SW1) | (1 << SW2));
}

void init_pins()
{
    //Inputs
    GPIO_PORTF_DIR_R |= ((1 << red_led) | (1 << green_led) | (1 << blue_led));

    //Outputs
    GPIO_PORTF_DIR_R &= ~((1 << SW1) | (1 << SW2));
}

void enable_pullups()
{
    GPIO_PORTF_PUR_R |= ((1 << SW1) | (1 << SW2));
}

void enable_DFun()
{
    GPIO_PORTF_DEN_R |= ((1 << red_led) | (1 << green_led) | (1 << blue_led)
            | (1 << SW1) | (1 << SW2));
}

void init_PORTF()
{
    SYSCTL_RCGC2_R |= 0x00000020;   //Enable Clock to Port F
    GPIO_PORTF_LOCK_R = 0x4C4F434B; //Unlock Port F
    commit_pins();                  //commit pins(to allow changes)
    GPIO_PORTF_AMSEL_R = 0x00;      //disable analog on PF
    GPIO_PORTF_PCTL_R = 0x00000000; //PCTL GPIO on PF4-0
    init_pins();                    //Initializing pins as Inputs or Outputs
    GPIO_PORTF_AFSEL_R = 0x00;      //disable alt funct on PF7-0
    enable_pullups();               //Enabling PullUPs for SW1 and SW2
    enable_DFun();                  //Enabling Digital Functionality of Pins
}

void digitalWrite(int pin, int status)
{
    if (status)
    {
        GPIO_PORTF_DATA_R |= (1 << pin);
    }
    else
    {
        GPIO_PORTF_DATA_R &= ~(1 << pin);
    }
}

int digitalRead(int pin)
{
    return ((GPIO_PORTF_DATA_R >> pin) & 0x01);
}

void control_leds(char *led_status)
{
    int led = -1, led_state = 0;
    switch (led_status[0])
    {
    case 'R':
    {
        led = red_led;
    }
        break;
    case 'G':
    {
        led = green_led;
    }
        break;
    case 'B':
    {
        led = blue_led;
    }
        break;
    }
    led_state = led_status[1] - '0';
    if (led != -1)
    {
        digitalWrite(led, led_state);
    }
}

void parse_response(char *response, char *led_status)
{
    int i = 0, j = 0;
    for (i = 0;; i++)
    {
        if (response[i] == '@')
        {
            break;
        }
    }
    i = -1;
    for (j = i + 1; j < i + 3; j++)
    {
        led_status[j - i - 1] = response[j];
    }
    led_status[j - i - 1] = '\n';
}

void uart_rx(void *p)
{
    char response[12];
    char led_status[2] = "";
    while (1)
    {
        if (SerialAvailable() > 0)
        {
            read_line(response);
            //print_line(response);
            parse_response(response, led_status);
            print_line(led_status);
            control_leds(led_status);
        }
        vTaskDelay(1);
    }
}

void toggle_led(void *p)
{
    int led = red_led;
    while (1)
    {
        if (xQueueReceive(qh, &led, 100))
        {
            GPIO_PORTF_DATA_R &= ~(0xE);
            GPIO_PORTF_DATA_R ^= (1 << led);
        }
        vTaskDelay(1);
    }
}

void SW1_task(void *p)
{
    //Boss task
    int button_status = 1, last_button_status = 1;
    int led = red_led;
    while (1)
    {
        button_status = digitalRead(SW1);
        if (button_status != last_button_status && button_status == 0)
        {
            //Toggle LEDs between states
            if (led > 3)
            {
                led = red_led;
            }
            xQueueSend(qh, &led, 100);
            led++;
            vTaskDelay(1);  //Wait for 1ms to Toggle the switch
        }
        last_button_status = button_status;
    }
}

int main()
{
    init_PORTF();
    UART_Init();
    qh = xQueueCreate(1, sizeof(int));
    xTaskCreate(uart_rx, (const portCHAR *) "UART Control", 128, NULL, 1, NULL);
    xTaskCreate(toggle_led, (const portCHAR *) "led toggle", 128, NULL, 2, NULL);
    xTaskCreate(SW1_task, (const portCHAR *) "SW1", 128, NULL, 1, NULL);
    vTaskStartScheduler();
}

/*****************************************************************************
 * serial_printf.c
 *  Redirection of the printf stream to the AVR serial port
 *    1. Initialize usart @ 57600 bps (change in BAUD)
 *    2. Define the low level put_char mechanism
 *    3. Redirect the printf io stream
 *
 *  Created on: 13/09/2016
 *      Author: jpsousa@fe.up.pt (gcc-avr)
 *      paco@f.up.pt (some cosmetic changes)
 *****************************************************************************/

#include <stdio.h>
#include <avr/io.h>

#define	BAUD 9600                     // Desired baud rate
//#define BAUDGEN ((F_CPU/(16*BAUD))-1)  // Calculated divider (may under/overflow for some cases)

void usart_init(void) 
{
  //UBRR0 = BAUDGEN;  
  UBRR0 = 103;  //para modulo bluetooth que funciona com baud de 9600
  UCSR0B = (1 << RXEN0) | (1 << TXEN0);
  UCSR0C = (1 << USBS0) | (3 << UCSZ00);
}

int usart_putchar(char c, FILE *stream) 
{
  while (!( UCSR0A & (1 << UDRE0)))
    ;
  UDR0 = c;
  return 0;
}

static FILE mystdout = FDEV_SETUP_STREAM(usart_putchar, NULL, _FDEV_SETUP_WRITE);

void printf_init(void) 
{
  usart_init();
  stdout = &mystdout;
}


void serial_send(uint8_t data)
{
  // Wait for empty transmit buffer
  while (!( UCSR0A & (1<<UDRE0)));
  // Put data into buffer, sends the data */
  UDR0 = data;
}

uint8_t serial_receive(void)
{
  // Wait for data to be received
  while (!(UCSR0A & (1<<RXC0)));
  // Get and return received data from buffer
  return UDR0;
}

uint8_t serial_receive_ready(void)
{
  // check if there is data to be received
  return (UCSR0A & (1<<RXC0));
}
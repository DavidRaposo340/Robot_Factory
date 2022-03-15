/*
 * serial.h
 *  Redirection of the printf stream to the AVR serial port
 *  Header file
 *  Created on: 13/09/2016
 *      Authors: jpsousa@fe.up.pt
 *               paco@fe.up.pt
 */

#ifndef SERIAL_PRINTF_H_
#define SERIAL_PRINTF_H_

#include <stdio.h>

void serial_send(uint8_t data);
uint8_t serial_receive(void);
uint8_t serial_receive_ready(void);

void printf_init(void);

#endif /* SERIAL_PRINTF_H_ */

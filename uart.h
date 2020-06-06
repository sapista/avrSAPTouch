/*
 * uart.h
 *
 *  Created on: 17 dic. 2017
 *      Author: sapista
 */

#ifndef UART_H_
#define UART_H_

int uart_putchar(char c, FILE *stream);
int uart_getchar(FILE *stream);

void uart_init(void);

FILE uart_output = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);
FILE uart_input = FDEV_SETUP_STREAM(NULL, uart_getchar, _FDEV_SETUP_READ);

#endif /* UART_H_ */

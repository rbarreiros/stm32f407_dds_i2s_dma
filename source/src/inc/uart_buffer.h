/*
 * uart_buffer.h
 *
 *  Created on: 2 Apr 2017
 *      Author: dimtass
 */

#ifndef UART_BUFFER_H_
#define UART_BUFFER_H_

#include <stddef.h>
#include <stdint.h>

/**
 * @brief This is a struct that is used for UART comms
 */
typedef struct {
	/* tx vars */
	uint8_t		*tx_buffer;
	size_t		tx_buffer_size;
	uint8_t 	tx_ready;
	uint16_t 	tx_ptr_in;
	uint16_t 	tx_ptr_out;
	uint16_t 	tx_length;
	uint8_t  	tx_int_en;
	/* rx vars */
	uint8_t		*rx_buffer;
	size_t		rx_buffer_size;
	uint8_t 	rx_ready;
	uint8_t		rx_ready_tmr;
	uint16_t	rx_ptr_in;
} tp_uart;

extern tp_uart m_buff;

#endif /* UART_BUFFER_H_ */

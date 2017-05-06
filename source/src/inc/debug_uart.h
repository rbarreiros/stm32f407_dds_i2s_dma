/*
 * debug_uart.h
 *
 *  Created on: 1 Apr 2017
 *      Author: dimtass
 *
 * Add this line to an 1ms timer interrupt:
 * 		if (m_buff.rx_ready) m_buff.rx_ready_tmr++;
 *
 * Add this line to the __io_putchar(int ch) function (used for printf)
 * 		debug_uart_send(ch);
 *
 * Add this line to the main loop in order to check for incoming data
 * 		debug_uart_rx_poll()
 *
 * Add this function in the interrupt handler of the used UART port
 * 		debug_uart_irq()
 *
 * Initialize the debug uart in main
 *
 *	debug_uart_init(USART1, 115200, glb.uart_rx_buff, UART_RX_BUFF_SIZE, glb.uart_tx_buff, UART_TX_BUFF_SIZE, &cmd_handler);
 *	debug_uart_set_trace_level(TRACE_LEVEL_DEFAULT, 1);
 */

#ifndef DEBUG_UART_H_
#define DEBUG_UART_H_

#include <stdint.h>
#include "stm32f4xx.h"
#include "uart_buffer.h"

/**
 * Trace levels for this project.
 * Have in mind that these are bit flags!
 */
typedef enum {
	TRACE_LEVEL_DEFAULT = 0x00000001,
	TRACE_LEVEL_USB = 0x00000002,
	TRACE_LEVEL_ESP = 0x00000004,
	TRACE_LEVEL_FLASH = 0x00000008,
} en_trace_level;

/* export the uart buffer so it's visible to other modules */
extern tp_uart m_buff;

/**
 * @brief Callback function definition for reception
 * @param[in] buffer Pointer to the RX buffer
 * @param[in] bufferlen The length of the received data
 */
typedef void (*fp_debug_uart_cb)(uint8_t *buffer, size_t bufferlen, uint8_t sender);

/**
 * @brief Initialize the debugging UART interface
 * @param[in] port A pointer to the STM32 UART
 * @param[in] baudrate The UART baudrate speed
 * @param[in] rx_buff A pointer to an buffer that will be used for RX
 * @param[in] rx_buff_len The size of the RX buffer
 * @param[in] tx_buff A pointer to a buffer that will be used for TX
 * @param[in] tx_buff_size The size of the TX buffer
 * @param[in] callback_func This will be called when data are received
 */
void debug_uart_init(USART_TypeDef * port, size_t baudrate, uint8_t * rx_buff, size_t rx_buff_size,
		uint8_t * tx_buff, size_t tx_buff_size, fp_debug_uart_cb callback_func);

/**
 * @brief Enable/Disable trace levels
 * @param[in] level The level to enable/disable
 * @param[in] enable 0: disable, 1: enable level
 */
void debug_uart_set_trace_level(en_trace_level level, uint8_t enable);

/**
 * @brief Check if a trace level is enabled
 * @param[in] level The trace level to check
 * @return int 0: level is disabled, 1: level is enabled
 */
int debug_uart_check_trace_level(en_trace_level level);

/**
 * @brief Get all the trace levels
 * @return uint32_t The bit flags for all levels (see: en_trace_level)
 */
uint32_t debug_uart_get_trace_levels(void);

/**
 * @brief IRQ handler for the debug interface
 */
void debug_uart_irq(void);

/**
 * @brief Send a single byte on the debug uart. This should be used with the
 * 		syscalls.c file to implement the printf() function
 * @param[in] ch The byte to send
 * @return int The byte sent
 */
int debug_uart_send(int ch);

/**
 * @brief Configure the STM32 uart port (including the port pins)
 */
void debug_uart_config(void);

/**
 * @brief Poll the RX buffer for new data. If new data are found then
 * 		the fp_debug_uart_cb will be called.
 */
void debug_uart_rx_poll(void);


#endif /* DEBUG_UART_H_ */

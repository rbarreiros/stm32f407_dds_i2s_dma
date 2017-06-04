/******************** (C) COPYRIGHT 2011 STMicroelectronics ********************
* File Name          : platform_config.h
* Author             : MCD Application Team
* Version            : V3.3.0
* Date               : 21-March-2011
* Description        : Evaluation board specific configuration file.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PLATFORM_CONFIG_H
#define __PLATFORM_CONFIG_H

#include <stdint.h>
#include <stdio.h>
#include "stm32f4xx.h"
#include "debug_uart.h"
#include "dds_defs.h"

#define DEBUG_TRACE

/**
 * System options
 */
#define TMR_1MSEC
#define TMR_100MSEC
#define TMR_250MSEC
#define TMR_1000MSEC

#define UART_RX_BUFF_SIZE	128
#define UART_TX_BUFF_SIZE	4096

#ifdef DEBUG_TRACE
#define TRACE(X) TRACEL(TRACE_LEVEL_DEFAULT, X)
#define TRACEL(TRACE_LEVEL, X) if (debug_uart_check_trace_level(TRACE_LEVEL)) printf X
#else
#define TRACE(X)
#endif

typedef struct {
	uint16_t ptr_in;
	uint16_t ptr_out;
	uint16_t length;
	uint8_t  int_en;
} tp_buff_pointers;

typedef enum {
	LED_PATTERN_BOOT = 0xFF,
	LED_PATTERN_HEARTBEAT = 0x0F,
} en_led_pattern;

typedef struct {
	/* Timers */
#ifdef TMR_1MSEC
	uint16_t tmr_1ms;
#endif
#ifdef TMR_100MSEC
	uint16_t tmr_100ms;
#endif
#ifdef TMR_250MSEC
	uint16_t tmr_250ms;
#endif
#ifdef TMR_1000MSEC
	uint16_t tmr_1sec;
#endif
	uint8_t tmr_boot_delay;
	uint16_t tmr_fpga_sm_tmr;
	uint16_t tmr_reset;	// When
	/* UART buffers */
	uint8_t	uart_rx_ready;
	uint8_t	uart_rx_ready_tmr;
	uint8_t uart_rx_buff[UART_RX_BUFF_SIZE];
	tp_buff_pointers uart_rx_buff_p;
	uint8_t uart_tx_buff[UART_TX_BUFF_SIZE];
	tp_buff_pointers uart_tx_buff_p;
	/* system state */
	uint8_t sys_led_pattern;
	uint8_t sys_led_pattern_index;
	uint32_t trace_level;

	float		dds_freq_L;
	float		dds_freq_R;
	uint32_t	phaseAccumulator_L;
	uint32_t	phaseAccumulator_R;
} tp_glb;

extern tp_glb glb;

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#define PIN_LED0		GPIO_Pin_9
#define PIN_LED0_PORT	GPIOF
#define PIN_LED1		GPIO_Pin_10
#define PIN_LED1_PORT	GPIOF
#define PIN_KEY0		GPIO_Pin_4
#define	PIN_KEY0_PORT	GPIOE
#define PIN_KEY1		GPIO_Pin_3
#define	PIN_KEY0_PORT	GPIOE
#define PIN_DEBUG1		GPIO_Pin_12
#define PIN_DEBUG2		GPIO_Pin_13
#define	PIN_DEBUG_PORT	GPIOD

#endif /* __PLATFORM_CONFIG_H */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/

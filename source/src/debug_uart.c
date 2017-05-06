/*
 * debug_uart.cpp
 *
 *  Created on: 1 Apr 2017
 *      Author: dimtass
 */
#include "debug_uart.h"
#include <stdio.h>

size_t			m_baudrate = 0;
USART_TypeDef * m_port = NULL;
uint32_t m_trace_levels = 0;
tp_uart m_buff = {0};
fp_debug_uart_cb m_callback_func = NULL;

void debug_uart_init(USART_TypeDef * port, size_t baudrate, uint8_t * rx_buff, size_t rx_buff_size,
		uint8_t * tx_buff, size_t tx_buff_size, fp_debug_uart_cb callback_func)
{
	m_port = port;
	m_baudrate = baudrate;
	m_callback_func = callback_func;

	/* reset TX */
	m_buff.tx_buffer = tx_buff;
	m_buff.tx_buffer_size = tx_buff_size;
	m_buff.tx_int_en = 0;
	m_buff.tx_length = 0;
	m_buff.tx_ptr_in = 0;
	m_buff.tx_ptr_out = 0;
	m_buff.tx_ready = 0;
	/* reset RX */
	m_buff.rx_buffer = rx_buff;
	m_buff.rx_buffer_size = rx_buff_size;
	m_buff.rx_ready = 0;
	m_buff.rx_ready_tmr = 0;
	m_buff.rx_ptr_in = 0;

	m_trace_levels = 0;
	debug_uart_config();
}


void debug_uart_rx_poll(void)
{
	if (m_buff.rx_ready_tmr >= 5) {
		m_buff.rx_ready = 0;
		m_buff.rx_ready_tmr = 0;

		if (m_callback_func != NULL) {
			m_callback_func(m_buff.rx_buffer, m_buff.rx_ptr_in, 0);
		}
		/* reset RX */
		m_buff.rx_ptr_in = 0;
	}
}

void debug_uart_set_trace_level(en_trace_level level, uint8_t enable)
{
	if (enable) {
	m_trace_levels |= (uint32_t) level;
	}
	else {
		m_trace_levels &= ~((uint32_t) level);
	}
}


int debug_uart_check_trace_level(en_trace_level level)
{
	return(m_trace_levels & (uint32_t)level);
}

uint32_t debug_uart_get_trace_levels(void)
{
	return(m_trace_levels);
}


void debug_uart_irq(void)
{
	if (USART_GetITStatus(m_port, USART_IT_RXNE) != RESET) {
		/* Read one byte from the receive data register */
		if (m_buff.rx_ptr_in == m_buff.rx_buffer_size)
			m_port->DR;	//discard data
		m_buff.rx_buffer[m_buff.rx_ptr_in++] = m_port->DR;
//		printf("%c", m_buff.rx_buffer[m_buff.rx_ptr_in-1]);

		/* Disable the USARTy Receive interrupt */
		//USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
		/* flag the byte reception */
		m_buff.rx_ready = 1;
		/* reset receive expire timer */
		m_buff.rx_ready_tmr = 0;
		m_port->SR &= ~USART_FLAG_RXNE;	          // clear interrupt
	}

	if (USART_GetITStatus(m_port, USART_IT_TXE) != RESET) {
		if (m_buff.tx_ptr_out != m_buff.tx_ptr_in) {
			m_port->DR = m_buff.tx_buffer[m_buff.tx_ptr_out];
			m_buff.tx_ptr_out = (m_buff.tx_ptr_out + 1)%m_buff.tx_buffer_size;
			m_buff.tx_length--;
		}
		else {
			/* Disable the USARTy Transmit interrupt */
			USART_ITConfig(m_port, USART_IT_TXE, DISABLE);
			m_buff.tx_int_en = 0;
			/* Rest uart buffer */
			m_buff.tx_ptr_in = 0;
			m_buff.tx_ptr_out = 0;
			m_buff.tx_length = 0;
		}
		m_port->SR &= ~USART_FLAG_TXE;	          // clear interrupt
	}
}

void debug_uart_config(void)
{

#ifdef STM32F4
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure; // this is used to configure the NVIC (nested vector interrupt controller)

	/* Enable GPIO clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	/* Enable UART clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	if (m_port == USART1) {
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	}
	else if (m_port == USART2) {
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART1);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	}

	/* Configure USART Tx as alternate function  */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);


	if (m_port == USART1) {
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	}
	else if (m_port == USART2) {
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART1);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	}

	/* Configure USART Rx as alternate function  */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	/* USART configuration */
	USART_Init(USART1, &USART_InitStructure);

	/*
	 Jump to the USART1_IRQHandler() function
	 if the USART1 receive interrupt occurs
	 */
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // enable the USART1 receive interrupt
	//USART_ITConfig(USART1, USART_IT_TXE, ENABLE); 	// enable the USART1 receive interrupt

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;// we want to configure the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // this sets the priority group of the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;// this sets the subpriority inside the group
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	// the USART1 interrupts are globally enabled
	NVIC_Init(&NVIC_InitStructure);	// the properties are passed to the NVIC_Init function which takes care of the low level stuff

	/* Enable the USART1 */
	USART_Cmd(USART1, ENABLE);
	USART_ClearFlag(USART1, USART_FLAG_TC);
#elif STM32F1
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure; // this is used to configure the NVIC (nested vector interrupt controller)

	if (m_port == USART1) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	}
	else if (m_port == USART2) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	}

	/* Configure USART Tx as alternate function push-pull */
	if (m_port == USART1) {
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	}
	else if (m_port == USART2) {
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	}
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure USART Rx as input floating */
	if (m_port == USART1) {
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	}
	else if (m_port == USART2) {
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	}
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = m_baudrate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl =
			USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	/* USART configuration */
	USART_Init(m_port, &USART_InitStructure);

	/*
	 Jump to the USART1_IRQHandler() function
	 if the USART1 receive interrupt occurs
	 */
	USART_ITConfig(m_port, USART_IT_RXNE, ENABLE); // enable the USART1 receive interrupt
	//USART_ITConfig(USART1, USART_IT_TXE, ENABLE); 	// enable the USART1 receive interrupt

	if (m_port == USART1) {
		NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;// we want to configure the USART1 interrupts
	}
	else if (m_port == USART2) {
		NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;// we want to configure the USART1 interrupts
	}
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // this sets the priority group of the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;// this sets the subpriority inside the group
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	// the USART1 interrupts are globally enabled
	NVIC_Init(&NVIC_InitStructure);	// the properties are passed to the NVIC_Init function which takes care of the low level stuff

	/* Enable the USART */
	USART_Cmd(m_port, ENABLE);
#endif
}

/**
 * This is a (weak) function in syscalls.c and is used from printf
 * to print data to the UART1
 */
int debug_uart_send(int ch)
{
	if ((m_buff.tx_ptr_in + 1)%m_buff.tx_buffer_size == m_buff.tx_ptr_out) {
		return ch;
	}

	m_buff.tx_length++;
	m_buff.tx_buffer[m_buff.tx_ptr_in] = ch;
	m_buff.tx_ptr_in = (m_buff.tx_ptr_in + 1)%m_buff.tx_buffer_size;

	/* If INT is disabled then enable it */
	if (!m_buff.tx_int_en) {
		m_buff.tx_int_en = 1;
		USART_ITConfig(m_port, USART_IT_TXE, ENABLE); 	// enable the USART1 receive interrupt
	}

	return ch;
}

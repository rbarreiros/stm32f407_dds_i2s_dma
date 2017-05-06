/**
 ******************************************************************************
 * @file    hw_config.c
 * @author  MCD Application Team
 * @version V4.0.0
 * @date    21-January-2013
 * @brief   Hardware Configuration & Setup
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
 *
 * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
 * You may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 *
 *        http://www.st.com/software_license_agreement_liberty_v2
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/

#include "stm32f4xx_it.h"
#include "platform_config.h"
#include "hw_config.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
ErrorStatus HSEStartUpStatus;
EXTI_InitTypeDef EXTI_InitStructure;


void TMR_1ms(void)
{
#ifdef TMR_1MSEC
	glb.tmr_1ms++;
#endif
#ifdef TMR_100MSEC
	glb.tmr_100ms++;
#endif
#ifdef TMR_250MSEC
	glb.tmr_250ms++;
#endif
#ifdef TMR_1000MSEC
	glb.tmr_1sec++;
#endif

	if (m_buff.rx_ready) m_buff.rx_ready_tmr++;

	if (glb.tmr_1ms) {
		glb.tmr_1ms = 0;
//		PIN_LED0_PORT->ODR ^= PI/N_LED1;
//		GPIO_ToggleBits(GPIOF, PIN_LED0 | PIN_LED1);
	}
	if (glb.tmr_1sec >= 1000) {
//		TRACE(("glb.tmr_1sec: %d\n", glb.tmr_1sec));
		glb.tmr_1sec = 0;
		GPIO_ToggleBits(GPIOF, PIN_LED0 | PIN_LED1);
	}
}

void NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable the TIM3 global Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}


void IWDG_Configuration(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, ENABLE);
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	IWDG_SetPrescaler(IWDG_Prescaler_32);
	IWDG_SetReload(1250*10);
	IWDG_ReloadCounter();
	IWDG_Enable();
}

void UART1_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure; // this is used to configure the NVIC (nested vector interrupt controller)

	/* Enable GPIO clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	/* Enable UART clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

	/* Configure USART Tx as alternate function  */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure USART Rx as alternate function  */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
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
}

void TIM3_Configuration(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	TIM_TimeBaseStructure.TIM_Period = 9;
	TIM_TimeBaseStructure.TIM_Prescaler = 7200;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM3, ENABLE);
}

void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);

	GPIO_InitStructure.GPIO_Pin = PIN_LED0 | PIN_LED1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(PIN_LED0_PORT, &GPIO_InitStructure);


	GPIO_InitStructure.GPIO_Pin = PIN_DEBUG1 | PIN_DEBUG2;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(PIN_DEBUG_PORT, &GPIO_InitStructure);

}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

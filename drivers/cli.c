/*
 * cli_uart.c
 *
 *  Created on: 28.12.2017
 *      Author: Mateusz
 */

#include "cli.h"
#include "stm32f1xx.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"


xQueueHandle uartTransmitQueue;

void uart_initialize(){

	uartTransmitQueue = xQueueCreate(20, sizeof(char));

	/* GPIO for UART configuration  */
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN; 	//portA clock enable

	//gpio A2 - tx
	GPIOA->CRL |= GPIO_CRL_CNF2_1 | GPIO_CRL_MODE2_1; // output push-pull , 2MHz
	GPIOA->CRL &= ~(GPIO_CRL_CNF2_0 | GPIO_CRL_MODE2_0); // output push-pull , 2MHz

	//gpio A3 - rx
	GPIOA->CRL |= GPIO_CRL_CNF3_1; // input pullup
	GPIOA->CRL &= ~( GPIO_CRL_CNF3_1 |  GPIO_CRL_CNF3_0 | GPIO_CRL_CNF3_0); // input pullup


	/* UART CLI 9600, 8, N, 1 */
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN; //clock enable for UART2
	USART2->CR1 |= USART_CR1_UE; 			// usart enable
	USART2->BRR |= USART_BRR(SystemCoreClock/2, 9600);
	USART2->CR1 |= USART_CR1_TE | USART_CR1_RE;	// sending idle frame
	USART2->CR1 |= USART_CR1_TXEIE;	// uart transmit interrupt enable

	NVIC_ClearPendingIRQ(USART2_IRQn);
	NVIC_EnableIRQ(USART2_IRQn);
	NVIC_SetPriority(USART2_IRQn, 6);

}

void uart_send(char *s){
	while(*s){
		xQueueSendToBack(uartTransmitQueue, s, 100);
		s++;
	}

	USART2->CR1 |= USART_CR1_TXEIE;
	// TODO unlock sender task
}

void cli_task(){
	// TODO initialize uart2

	// TODO create sender task

	// TODO create receiver task


	for(;;){



	}

}


void USART2_IRQHandler(){
	char c;
	portBASE_TYPE xHigherPriorityHasWoken = pdFALSE;

	GPIOA->ODR ^= GPIO_PIN_5;

	if (USART2->SR & USART_SR_TXE){
		USART2->SR &= ~USART_SR_TXE;

		uint8_t status = xQueueReceiveFromISR(uartTransmitQueue, &c, &xHigherPriorityHasWoken);
		if (status == pdPASS){
			USART2->DR = c;
		}else{
			USART2->CR1 &= ~USART_CR1_TXEIE; // disable interrupt if nothing to send
		}
	}
	NVIC_ClearPendingIRQ(USART2_IRQn);
}

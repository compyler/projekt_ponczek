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
xQueueHandle uartReceiveQueue;

void uart_initialize(){

	uartTransmitQueue = xQueueCreate(20, sizeof(char));
	uartReceiveQueue = xQueueCreate(20, sizeof(char));
	/* GPIO for UART configuration  */
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN; 	//portA clock enable

	//gpio A2 - tx
	GPIOA->CRL |= GPIO_CRL_CNF2_1 | GPIO_CRL_MODE2_1; // output push-pull , 2MHz
	GPIOA->CRL &= ~(GPIO_CRL_CNF2_0 | GPIO_CRL_MODE2_0); // output push-pull , 2MHz

	//gpio A3 - rx
	GPIOA->CRL |= GPIO_CRL_CNF3_0; // input pullup
	GPIOA->CRL &= ~( GPIO_CRL_CNF3_1 |  GPIO_CRL_MODE3_1 | GPIO_CRL_MODE3_0); // input pullup


	/* UART CLI 9600, 8, N, 1 */
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN; //clock enable for UART2
	USART2->CR1 |= USART_CR1_UE; 			// usart enable
	USART2->BRR |= USART_BRR(SystemCoreClock/2, 9600);
	USART2->CR1 |= USART_CR1_TE | USART_CR1_RE;	// sending idle frame
	USART2->CR1 |= USART_CR1_TXEIE | USART_CR1_RXNEIE;	// uart transmit interrupt enable

	NVIC_ClearPendingIRQ(USART2_IRQn);
	NVIC_EnableIRQ(USART2_IRQn);
	NVIC_SetPriority(USART2_IRQn, 6);

}

void uart_send(char *s){
	while(*s){
		xQueueSendToBack(uartTransmitQueue, s, 100 / portTICK_RATE_MS);
		s++;
	}
	USART2->CR1 |= USART_CR1_TXEIE; // enable transmition interrupt

}

char * uart_receive(){


	return 0;
}

void cli_task(){
	//initializing uart2
	uart_initialize();

	//create sender task
	//not needed

	// TODO create receiver task


	for(;;){

		taskYIELD();

	}

}


void USART2_IRQHandler(){
	static char uartReceiverBuffer[20];
	char c;

	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;


	if (USART2->SR & USART_SR_TXE){
		USART2->SR &= ~USART_SR_TXE;

		uint8_t status = xQueueReceiveFromISR(uartTransmitQueue, &c, &xHigherPriorityTaskWoken);
		if (status == pdPASS){
			USART2->DR = c;
		} else {
			USART2->CR1 &= ~USART_CR1_TXEIE; // disable interrupt if nothing to send
		}
	}
	if (USART2->SR & USART_SR_RXNE){
		static uint8_t idx = 0;

		//odczytaj dane
		char data = USART2->DR;

		if (data == '\r'){
			uartReceiverBuffer[idx] = 0;
			idx = 0;

			char *buffer = uartReceiverBuffer;
			while (*buffer){		// when <enter> copy buffer to queue
				xQueueSendToBackFromISR(uartReceiveQueue, buffer, &xHigherPriorityTaskWoken);
				buffer++;
			}
			// tymczasowo ...
				if (strcmp("tog", uartReceiverBuffer) == 0){
					GPIOA->ODR ^= GPIO_PIN_5; // for debug puroposes
				}


		} else if (data == 127){ 	// backspace button in putty
			if (idx) idx--;

		} else {
			uartReceiverBuffer[idx] = data;
			if (idx < 20) idx++;
		}

//		echo
		if (data == '\r'){
			const char d[] = "\n\r";
			xQueueSendToBackFromISR(uartTransmitQueue, &d[0], &xHigherPriorityTaskWoken);
			xQueueSendToBackFromISR(uartTransmitQueue, &d[1], &xHigherPriorityTaskWoken);
		} else {
			xQueueSendToBackFromISR(uartTransmitQueue, &data, &xHigherPriorityTaskWoken);
		}
		USART2->CR1 |= USART_CR1_TXEIE; // enable transmition interrupt



		// TODO obudz receiver task


	}


	NVIC_ClearPendingIRQ(USART2_IRQn);
}








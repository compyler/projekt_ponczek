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
#include <string.h>


#include "wifi.h"


xQueueHandle uartTransmitQueue;
xQueueHandle uartReceiveQueue;


void uart_initialize(){

	uartTransmitQueue = xQueueCreate(200, sizeof(char));
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

void parse(char *buf){
	// tymczasowo ...
	// TODO usunac
		if (strcmp("tog", buf) == 0){
			GPIOA->ODR ^= GPIO_PIN_5; // for debug puroposes
		} else {
			wifi_send(buf);
			wifi_send("\r\n");
		}
	// 	^^^

}

void uart_receiver_task (){
	static char uartReceiverBuffer[50];
	static uint8_t idx = 0;
	char data;

	for(;;){
		//odczytaj dane

		xQueueReceive(uartReceiveQueue, &data, portMAX_DELAY);	// czekaj na dane

		if (data == '\r'){
			uartReceiverBuffer[idx] = 0;
			idx = 0;

			parse(uartReceiverBuffer);

		} else if (data == 127){ 	// backspace button in putty
			if (idx) idx--;

		} else {
			uartReceiverBuffer[idx] = data;
			if (idx < 50) idx++;
		}

//		echo
		if (data == '\r'){
			uart_send("\n\r");
		} else {
			char tab[2] = {};
			tab[0] = data;
			uart_send(tab);
		}
	}
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
	char data;

	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;


	if (USART2->SR & USART_SR_TXE){
		USART2->SR &= ~USART_SR_TXE;

		uint8_t status = xQueueReceiveFromISR(uartTransmitQueue, &data, &xHigherPriorityTaskWoken);
		if (status == pdPASS){
			USART2->DR = data;
		} else {
			USART2->CR1 &= ~USART_CR1_TXEIE; // disable interrupt if nothing to send
		}
	}
	if (USART2->SR & USART_SR_RXNE){
		data = USART2->DR;
		xQueueSendToBackFromISR(uartReceiveQueue, &data, &xHigherPriorityTaskWoken);

	}

	NVIC_ClearPendingIRQ(USART2_IRQn);
}








/*
 * cli_uart.c
 *
 *  Created on: 28.12.2017
 *      Author: Mateusz
 */

#include "wifi.h"
#include "stm32f1xx.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include <string.h>

#include "cli.h"


xQueueHandle wifiTransmitQueue;
xQueueHandle wifiReceiveQueue;


void wifi_initialize(){

	wifiTransmitQueue = xQueueCreate(50, sizeof(char));
	wifiReceiveQueue = xQueueCreate(500, sizeof(char));

	/* GPIO for WIFI configuration  */
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN; 	//portC clock enable

	//gpio C10 - tx
	GPIOC->CRH |= GPIO_CRH_CNF10_1 | GPIO_CRH_MODE10_1; // output push-pull , 2MHz
	GPIOC->CRH &= ~(GPIO_CRH_CNF10_0 | GPIO_CRH_MODE10_0); // output push-pull , 2MHz

	//gpio C11 - rx
	GPIOC->CRH |= GPIO_CRH_CNF11_0; // input pullup
	GPIOC->CRH &= ~( GPIO_CRH_CNF11_1 |  GPIO_CRH_MODE11_1 | GPIO_CRH_MODE11_0); // input pullup

	//TODO uart3 remap
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
	AFIO->MAPR |= AFIO_MAPR_USART3_REMAP_0;


	/* UART WIFI 115200, 8, N, 1 */
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN; //clock enable for UART3
	USART3->CR1 |= USART_CR1_UE; 			// usart enable
	USART3->BRR |= USART_BRR(SystemCoreClock/2, 115200);
	USART3->CR1 |= USART_CR1_TE | USART_CR1_RE;	// sending idle frame
	USART3->CR1 |= USART_CR1_TXEIE | USART_CR1_RXNEIE;	// uart transmit interrupt enable

	NVIC_ClearPendingIRQ(USART3_IRQn);
	NVIC_EnableIRQ(USART3_IRQn);
	NVIC_SetPriority(USART3_IRQn, 6);

}

void wifi_send(char *s){
	while(*s){
		xQueueSendToBack(wifiTransmitQueue, s, 100 / portTICK_RATE_MS);
		s++;
	}
	USART3->CR1 |= USART_CR1_TXEIE; // enable transmition interrupt

}

void dparse(char *buf){
	// tymczasowo ...
	// TODO usunac
		if (strcmp("tog", buf) == 0){
			GPIOA->ODR ^= GPIO_PIN_5; // for debug puroposes
		}
	// 	^^^

}

void wifi_receiver_task (){
//	static char wifiReceiverBuffer[20];
//	static uint8_t idx = 0;
	char data;

	for(;;){
		//odczytaj dane

		xQueueReceive(wifiReceiveQueue, &data, portMAX_DELAY);	// czekaj na dane

		char tab[2] = {0,0};
		tab[0] = data;
		uart_send(tab);



//		if (data == '\r'){
//			uartReceiverBuffer[idx] = 0;
//			idx = 0;
//
//			dparse(uartReceiverBuffer);
//
//		} else if (data == 127){ 	// backspace button in putty
//			if (idx) idx--;
//
//		} else {
//			uartReceiverBuffer[idx] = data;
//			if (idx < 20) idx++;
//		}
//
////		echo
//		if (data == '\r'){
//			wifi_send("\n\r");
//		} else {
//			char tab[2] = {};
//			tab[0] = data;
//			wifi_send(tab);
//		}
	}
}




void USART3_IRQHandler(){
	char data;

	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;


	if (USART3->SR & USART_SR_TXE){
		USART3->SR &= ~USART_SR_TXE;

		uint8_t status = xQueueReceiveFromISR(wifiTransmitQueue, &data, &xHigherPriorityTaskWoken);
		if (status == pdPASS){
			USART3->DR = data;
		} else {
			USART3->CR1 &= ~USART_CR1_TXEIE; // disable interrupt if nothing to send
		}
	}
	if (USART3->SR & USART_SR_RXNE){
		data = USART3->DR;
		xQueueSendToBackFromISR(wifiReceiveQueue, &data, &xHigherPriorityTaskWoken);

	}

	NVIC_ClearPendingIRQ(USART3_IRQn);
}








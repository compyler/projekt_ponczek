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
#include "ssd1306.h"


xQueueHandle wifiTransmitQueue;
xQueueHandle wifiReceiveQueue;


void wifi_initialize(){

	wifiTransmitQueue = xQueueCreate(50, sizeof(char));
	wifiReceiveQueue = xQueueCreate(200, sizeof(char));

	/* GPIO for WIFI configuration  */
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN; 	//portC clock enable

	//gpio C10 - tx
	GPIOC->CRH |= GPIO_CRH_CNF10_1 | GPIO_CRH_MODE10_1; // output push-pull , 2MHz
	GPIOC->CRH &= ~(GPIO_CRH_CNF10_0 | GPIO_CRH_MODE10_0); // output push-pull , 2MHz

	//gpio C11 - rx
	GPIOC->CRH |= GPIO_CRH_CNF11_0; // input pullup
	GPIOC->CRH &= ~( GPIO_CRH_CNF11_1 |  GPIO_CRH_MODE11_1 | GPIO_CRH_MODE11_0); // input pullup

	//uart3 remap
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
	NVIC_SetPriority(USART3_IRQn, 10);

}

void wifi_send(char *s){
	while(*s){
		xQueueSendToBack(wifiTransmitQueue, s, 0);
		s++;
	}
	USART3->CR1 |= USART_CR1_TXEIE; // enable transmition interrupt

}

void wifiparse(char *buf){

	while(*buf != ':') buf++;
	buf += 1;


	uart_send(buf);
	uart_send("\r\n");

}

void wifi_receiver_task (){

	char data;

	static char buffer[100];
	static uint8_t bidx = 0;

	for(;;){
		//odczytaj dane
		xQueueReceive(wifiReceiveQueue, &data, portMAX_DELAY);	// czekaj na dane

		if (data == '\n'){
			buffer[bidx++] = data;
			buffer[bidx] = 0;
			bidx = 0;

//			ssd1306_clear_screen();

//			ssd1306_draw_string(buf ,0,0);

			uart_send(buffer);
		}else {
			buffer[bidx] = data;
			bidx++;
		}
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
	if ( (USART3->SR & USART_SR_RXNE) || (USART3->SR & USART_SR_ORE) ){
		data = USART3->DR;
		xQueueSendToBackFromISR(wifiReceiveQueue, &data, &xHigherPriorityTaskWoken);

	}

	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


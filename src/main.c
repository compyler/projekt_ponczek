/**
 ******************************************************************************
 * @file    main.c
 * @author  Ac6
 * @version V1.0
 * @date    01-December-2013
 * @brief   Default main function.
 ******************************************************************************
 */

#include "stm32f1xx.h"
#include "stm32f1xx_nucleo.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdlib.h>

#include "../drivers/cli.h"
#include "../drivers/wifi.h"


void led_init(){
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	GPIOA->CRL |= GPIO_CRL_MODE5_1;
	GPIOA->CRL &= ~(GPIO_CRL_CNF5_1 | GPIO_CRL_CNF5_0 | GPIO_CRL_MODE5_0);
}

void LED_Task() {

	for (;;) {
		GPIOA->ODR ^= GPIO_PIN_5;
		vTaskDelay(100 / portTICK_RATE_MS);
	}
}

void uartTask() {
	uint8_t i = 0;

	char tab[10];

	for (;;) {
		itoa((i++)%10, tab, 10);
		uart_send(tab);
		uart_send(" - Hello World\n\r");

		vTaskDelay(1000 / portTICK_RATE_MS);
	}
}

int main(void) {
	SystemCoreClockUpdate();
//	NVIC_SetPriorityGrouping( 0 );
	led_init();

	GPIOA->ODR ^= GPIO_PIN_5;
	uart_initialize();
	wifi_initialize();

	xTaskCreate(LED_Task, "LED", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
//	xTaskCreate(uartTask, "UART", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	xTaskCreate(uart_receiver_task, "CLI", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	xTaskCreate(wifi_receiver_task, "WIFI", configMINIMAL_STACK_SIZE, NULL, 1, NULL);


	vTaskStartScheduler();

	for (;;);
}

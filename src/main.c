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


void led_init(){
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	GPIOA->CRL |= GPIO_CRL_MODE5_1;
	GPIOA->CRL &= ~(GPIO_CRL_CNF5_1 | GPIO_CRL_CNF5_0 | GPIO_CRL_MODE5_0);
}

void LED_Task() {

	for (;;) {
		GPIOA->ODR ^= GPIO_PIN_5;
		vTaskDelay(500 / portTICK_RATE_MS);
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
	NVIC_SetPriorityGrouping( 0 );
	led_init();

	uart_initialize();

//	xTaskCreate(LED_Task, "CLI", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
//	xTaskCreate(uartTask, "UART", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	xTaskCreate(uart_receiver_task, "UART", configMINIMAL_STACK_SIZE, NULL, 1, NULL);



	vTaskStartScheduler();

	for (;;);
}

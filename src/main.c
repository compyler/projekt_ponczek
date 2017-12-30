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
	RCC->APB1ENR |= RCC_APB2ENR_IOPAEN;
	GPIOA->CRL |= GPIO_CRL_MODE5_1;
	GPIOA->CRL &= ~(GPIO_CRL_CNF5_1 | GPIO_CRL_CNF5_0 | GPIO_CRL_MODE5_0);
}

void Task() {

	for (;;) {
		GPIOA->ODR ^= GPIO_PIN_5;
		vTaskDelay(500 / portTICK_RATE_MS);
	}
}

void uartTask() {

	UART_HandleTypeDef s_UARTHandle;

	s_UARTHandle.Instance = USART2;
	s_UARTHandle.Init.BaudRate = 9600;
	s_UARTHandle.Init.WordLength = UART_WORDLENGTH_8B;
	s_UARTHandle.Init.StopBits = UART_STOPBITS_1;
	s_UARTHandle.Init.Parity = UART_PARITY_NONE;
	s_UARTHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	s_UARTHandle.Init.Mode = UART_MODE_TX_RX;

	HAL_UART_Init(&s_UARTHandle);


	uint8_t i = 0;

	char buf[] = " - Hello World\n\r";
	char tab[10];

	for (;;) {
		itoa((i++)%10, tab, 10);
		uart_send(tab);
		uart_send(buf);
		uart_send("\n\r");
//		HAL_UART_Transmit(&s_UARTHandle, (uint8_t *) tab, 1, HAL_MAX_DELAY);
//		HAL_UART_Transmit(&s_UARTHandle, (uint8_t *) buf, sizeof(buf), HAL_MAX_DELAY);
		vTaskDelay(1000 / portTICK_RATE_MS);
	}
}

int main(void) {
	SystemCoreClockUpdate();
	uart_initialize();
//	HAL_Init();

	led_init();

//	xTaskCreate(Task, "task", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
	xTaskCreate(uartTask, "UART", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

	vTaskStartScheduler();

	for (;;);
}

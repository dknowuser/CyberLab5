#include "stm32f4xx.h"                  // Device header
#include "FreeRTOS.h"                   // ARM.FreeRTOS::RTOS:Core
#include "task.h"                       // ARM.FreeRTOS::RTOS:Core
#include "queue.h"                      // ARM.FreeRTOS::RTOS:Core
#include "semphr.h"                     // ARM.FreeRTOS::RTOS:Core

#define SOS_LED_DELAY					500
#define SOS_LED_SHORT_DELAY		83
#define n											3

static SemaphoreHandle_t semaphore;
static xQueueHandle queue;

void delay(const int constr)
{
	TickType_t xLastWakeTime;
	const TickType_t xPeriod = pdMS_TO_TICKS(constr);
	xLastWakeTime = xTaskGetTickCount();
	
	vTaskDelayUntil(&xLastWakeTime, xPeriod); 
};

void EXTI0_IRQHandler(void)
{
	BaseType_t needCS = pdFALSE;
	EXTI->PR |= (1 << 0); // Clear interrupt flag
	xSemaphoreGiveFromISR(semaphore, &needCS);
	portYIELD_FROM_ISR(needCS);
};

void InitHardware(void)
{
	// Enable clock for GPIO port A, port D and SYSCFG
	RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIODEN);
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	
	// PD15 output mode
	GPIOD->MODER |= (1 << 30);
	
	// Enable 0 interrupt line for PA0 pin
	EXTI->IMR |= EXTI_IMR_IM0;
	// Falling trigger for PA0 pin
	EXTI->FTSR |= EXTI_IMR_IM0;
	NVIC_EnableIRQ(EXTI0_IRQn);
	
	__enable_irq();
};

void vTaskVisualize(void *pvParameters)
{
	uint16_t i = 0;
	uint16_t chosenMode = 0;
	while(1) {
		xQueuePeek(queue, &chosenMode, 0);
		if(!chosenMode) {
			for(i = 0; i < n; i++) {
				delay(SOS_LED_SHORT_DELAY);
				GPIOD->ODR |= (1 << 15);
				delay(SOS_LED_SHORT_DELAY);
				GPIOD->ODR &= ~((uint32_t)(1 << 15));
			};
			
			for(i = 0; i < n; i++) {
				delay(SOS_LED_DELAY);
				GPIOD->ODR |= (1 << 15);
				delay(SOS_LED_DELAY);
				GPIOD->ODR &= ~((uint32_t)(1 << 15));
			};
			
			for(i = 0; i < n; i++) {
				delay(SOS_LED_SHORT_DELAY);
				GPIOD->ODR |= (1 << 15);
				delay(SOS_LED_SHORT_DELAY);
				GPIOD->ODR &= ~((uint32_t)(1 << 15));
			};
			delay(SOS_LED_DELAY);
		};
	};
};

void vTaskMode(void *pvParameters)
{
	uint16_t chosenMode = 0;
	while(1) {
		while(xSemaphoreTake(semaphore, 0) == pdFALSE);
		if(xQueueReceive(queue, &chosenMode, 0) == pdTRUE) {
			if(!chosenMode) {
				chosenMode = 1;
				xQueueSendToBack(queue, &chosenMode, 0);
			}
			else {
				chosenMode = 0;
				xQueueSendToBack(queue, &chosenMode, 0);
			};
		}
		else {
			chosenMode = 1;
			xQueueSendToBack(queue, &chosenMode, 0);
		};
	};
};

int main(void)
{
	InitHardware();
	
	semaphore = xSemaphoreCreateBinary();
	if(semaphore == NULL) return 1;	
	queue = xQueueCreate(1, sizeof(uint16_t));
	if(queue == NULL) return 1;

	xTaskCreate(vTaskVisualize, "Visualize",
		configMINIMAL_STACK_SIZE, NULL, 2, NULL);
	xTaskCreate(vTaskMode, "Mode",
		configMINIMAL_STACK_SIZE, NULL, 2, NULL);
	
	vTaskStartScheduler();
	
	while(1);
};

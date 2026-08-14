#include "board.h"
#include <FreeRTOS.h>
#include <task.h>

#define TASK1_TASK_PRIO     5
#define TASK1_STK_SIZE      256
#define TASK2_TASK_PRIO     5
#define TASK2_STK_SIZE      256

static TaskHandle_t Task1Task_Handler;
static TaskHandle_t Task2Task_Handler;

void task1(void *pvParameters)
{
  while(1)
  {
    LED_BLUE_ON;
    vTaskDelay(250);
    LED_BLUE_OFF;
    vTaskDelay(250);
  }
}

void task2(void *pvParameters)
{
  while(1)
  {
    LED_RED_ON;
    vTaskDelay(500);
    LED_RED_OFF;
    vTaskDelay(500);
  }
}

int main(void)
{
  HalInit();

  xTaskCreate(task2, "task2", TASK2_STK_SIZE, nullptr, TASK2_TASK_PRIO, &Task2Task_Handler);
  xTaskCreate(task1, "task1", TASK1_STK_SIZE, nullptr, TASK1_TASK_PRIO, &Task1Task_Handler);
  vTaskStartScheduler();

  while(1)
    __WFI();
}

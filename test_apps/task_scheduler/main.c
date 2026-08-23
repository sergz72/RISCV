#include <stdio.h>
#include "os.h"

void __attribute__((naked, noreturn)) task1(void)
{
  puts("Task1 start");
  for (int i = 0; i < 10; i++)
  {
    osDelay(500);
    puts("Task1");
  }
  puts("Task1 end");
  osExit(1);
}

void __attribute__((naked, noreturn)) task2(void)
{
  puts("Task2 start");
  for (int i = 0; i < 40; i++)
  {
    osDelay(250);
    puts("Task2");
  }
  puts("Task2 end");
  osExit(2);
}

int main(void)
{
  osInit(3, 2048);
  int rc = osTaskCreate(task1);
  if (rc)
    return rc;
  rc = osTaskCreate(task2);
  if (rc)
    return rc;
  puts("Main task start");
  for (;;)
  {
    osDelay(1000);
    puts("Main task");
  }
}

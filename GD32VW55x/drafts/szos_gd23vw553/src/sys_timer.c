#include "board.h"
#include "sys_timer.h"

unsigned volatile long long int time_since_boot;

void eclic_mtip_handler(void)
{
  time_since_boot++;
  rv_counter_t value = SysTimer_GetLoadValue();
  SysTimer_SetCompareValue(value + SystemCoreClock / 4000);
  ECLIC_ClearPendingIRQ(CLIC_INT_TMR);
}

void sys_timer_init(void)
{
  time_since_boot = 0;
  //SysTimer_SetControlValue(SysTimer_MTIMECTL_CMPCLREN_Msk);
  SysTimer_SetLoadValue(0);
  SysTimer_SetCompareValue(SystemCoreClock / 4000);
  __ECLIC_SetTrigIRQ(CLIC_INT_TMR, ECLIC_POSTIVE_EDGE_TRIGGER);
  eclic_irq_enable(CLIC_INT_TMR, 0, 0);
}

void delayms(unsigned int ms)
{
  unsigned long long int to = time_since_boot + ms;
  while (time_since_boot < to)
    __WFI();
}

void delay(unsigned int us)
{
  rv_counter_t to = SysTimer_GetLoadValue() + (unsigned long long int)us * SystemCoreClock / 4000000;
  while (SysTimer_GetLoadValue() < to)
    ;
}
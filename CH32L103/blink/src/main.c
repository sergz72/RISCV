#include <delay.h>
#include <ch32l103.h>
#include <system_ch32l103.h>

#define CORE_CLOCK 1000000
#define LSE_CLOCK  32768

#define LED1_CLOCK_ENABLE RCC_PB2Periph_GPIOB
#define LED1_PORT  GPIOB
#define LED1_PIN   GPIO_Pin_9
#define LED1_ON    LED1_PORT->BCR = LED1_PIN
#define LED1_OFF   LED1_PORT->BSHR = LED1_PIN

#define LED2_CLOCK_ENABLE RCC_PB2Periph_GPIOC
#define LED2_PORT  GPIOC
#define LED2_PIN   GPIO_Pin_13
#define LED2_ON    LED2_PORT->BCR = LED2_PIN
#define LED2_OFF   LED2_PORT->BSHR = LED2_PIN

uint32_t SystemCoreClock = CORE_CLOCK;

static bool timer_interrupt;

void __attribute__((interrupt("WCH-Interrupt-fast"))) LPTIM_IRQHandler(void)
{
  if(LPTIM->ISR & LPTIM_ISR_ARRM)
  {
    timer_interrupt = true;
    LPTIM->ICR = LPTIM_ICR_ARRMCF;
  }
}

void SystemInit(void)
{
  RCC->CTLR |= (uint32_t)0x00000001;
  RCC->CFGR0 &= (uint32_t)0x08FF0000;
  RCC->CTLR &= (uint32_t)0xFEF6FFFB;
  RCC->CTLR &= (uint32_t)0xFFFBFFFF;
  RCC->CFGR0 &= (uint32_t)0xFF00FFFF;
  RCC->INTR = 0x009F0000;
}

static void GPIOInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_PB2PeriphClockCmd(LED1_CLOCK_ENABLE, ENABLE);
  LED1_OFF;
  GPIO_InitStructure.GPIO_Pin = LED1_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(LED1_PORT, &GPIO_InitStructure);

  RCC_PB2PeriphClockCmd(LED2_CLOCK_ENABLE, ENABLE);
  LED2_OFF;
  GPIO_InitStructure.GPIO_Pin = LED2_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(LED2_PORT, &GPIO_InitStructure);
}

#define LPTIM_InClockSource_LSE 0x04000000

static void LPTIMInit(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  RCC_PB1PeriphClockCmd(RCC_PB1Periph_LPTIM, ENABLE);

  //Timer 250ms
  LPTIM->CFGR = LPTIM_InClockSource_LSE; // internal clock, lse, prescaler = 1
  LPTIM->ARR = LSE_CLOCK / 4;

  NVIC_InitStructure.NVIC_IRQChannel = LPTIM_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  LPTIM->CR |= LPTIM_CR_ENABLE;
  LPTIM->CR |= LPTIM_CR_CNTSTRT;
  LPTIM->ICR = LPTIM_ICR_ARRMCF;
  LPTIM->IER |= LPTIM_IER_ARRMIE;
}

static void main_loop(void)
{
  bool led_state = false;

  while (1)
  {
    __WFI();
    if (timer_interrupt)
    {
      led_state = !led_state;
      if (led_state)
      {
        LED1_ON;
        LED2_OFF;
      }
      else
      {
        LED1_OFF;
        LED2_ON;
      }
      timer_interrupt = false;
    }
  }
}

static void LDO_Deal(void)
{
  //reduced power 80uA
  /* LDOTRIM[1:0] == 01b */
  uint32_t tmp = 0;
  tmp = EXTEN->EXTEN_CTR;
  tmp &= ~EXTEN_LDO_TRIM;
  tmp |= EXTEN_LDO_TRIM0;
  EXTEN->EXTEN_CTR = tmp;
}

// to erase chip:
// wlink erase --method power-off --chip ch32l103
int main(void)
{
  timer_interrupt = false;

  //DBGMCU_Config(DBGMCU_SLEEP|DBGMCU_STOP|DBGMCU_STANDBY, ENABLE);

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

  LDO_Deal();

  RCC_PB2PeriphClockCmd(RCC_PB2Periph_AFIO, ENABLE);
  RCC_PB1PeriphClockCmd(RCC_PB1Periph_PWR | RCC_PB1Periph_BKP, ENABLE);
  PWR_FLASH_LP_Cmd(ENABLE);
  RCC->CTLR |= 4; // hsi in low power mode, 1 MHz
  PWR_BackupAccessCmd(ENABLE);
  RCC_LSEConfig(RCC_LSE_ON);
  PWR_BackupAccessCmd(DISABLE);
  Delay_Init();
  while (!RCC_GetFlagStatus(RCC_FLAG_LSERDY))
    Delay_Ms(1);

  GPIOInit();
  LPTIMInit();

  main_loop();
}

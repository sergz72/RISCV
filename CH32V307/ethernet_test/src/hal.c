#include "board.h"
#include <ch32v30x_gpio.h>
#include "debug.h"
#include "delay.h"
#include <eth_driver.h>

volatile unsigned int timeCnt;
volatile unsigned int timer_interrupt;

unsigned char MACAddr[6];
unsigned char IPAddr[4]   = {0, 0, 0, 0};                    //IP address
unsigned char GWIPAddr[4] = {0, 0, 0, 0};                    //Gateway IP address
unsigned char IPMask[4]   = {0, 0, 0, 0};                    //subnet mask

/*********************************************************************
 * @fn      ETH_IRQHandler
 *
 * @brief   This function handles ETH exception.
 *
 * @return  none
 */
void __attribute__((interrupt("WCH-Interrupt-fast"))) ETH_IRQHandler(void)
{
  WCHNET_ETHIsr();
}

/*********************************************************************
 * @fn      TIM2_IRQHandler
 *
 * @brief   This function handles TIM2 exception.
 *
 * @return  none
 */
void __attribute__((interrupt("WCH-Interrupt-fast"))) TIM2_IRQHandler(void)
{
  timeCnt++;
  timer_interrupt = 1;
  WCHNET_TimeIsr(WCHNETTIMERPERIOD);
  TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
}

static void GPIOInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  // led green
  GPIO_InitStructure.GPIO_Pin = LED_BLUE_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(LED_BLUE_PORT, &GPIO_InitStructure);

  // led red
  GPIO_InitStructure.GPIO_Pin = LED_RED_PIN;
  GPIO_Init(LED_RED_PORT, &GPIO_InitStructure);
}

/*********************************************************************
 * @fn      TIM2_Init
 *
 * @brief   Initializes TIM2.
 *
 * @return  none
 */
void TIM2_Init( void )
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure={0};
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

  TIM_TimeBaseStructure.TIM_Period = SystemCoreClock / 1000000;
  TIM_TimeBaseStructure.TIM_Prescaler = WCHNETTIMERPERIOD * 1000 - 1;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
  TIM_ITConfig(TIM2, TIM_IT_Update ,ENABLE);
  TIM_Cmd(TIM2, ENABLE);
  TIM_ClearITPendingBit(TIM2, TIM_IT_Update );
  NVIC_EnableIRQ(TIM2_IRQn);
}

/*static void Configure_Memory_Split(void)
{
  // Get the current User Option Byte value
  uint16_t userByte = FLASH_GetUserOptionByte();

  // Check if bits 7:6 are NOT '00' (meaning it is not yet in 192K/128K mode)
  if ((userByte & 0x00C0) != 0x0000) {
    // Clear bits 7 and 6 to set the 192KB Flash + 128KB RAM profile
    uint8_t targetConfig = (userByte & 0x3F) | 0x00;

    // Unlock flash registers for programming
    FLASH_Unlock();

    // Clear existing option bytes
    FLASH_EraseOptionBytes();

    // Program the configuration data into the user option byte address
    FLASH_ProgramOptionByteData(0x1FFFF802, targetConfig);

    // The hardware change requires a cold system reset to take effect
    NVIC_SystemReset();
  }
}*/

void HalInit(void)
{
  //Configure_Memory_Split();

  timeCnt = timer_interrupt = 0;

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  Delay_Init();

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

  GPIOInit();
  TIM2_Init();

  WCHNET_GetMacAddr(MACAddr);
  unsigned char rc = ETH_LibInit(IPAddr,GWIPAddr,IPMask,MACAddr);
  if(rc != WCHNET_ERR_SUCCESS)
  {
    LED_RED_ON;
    while(1)
      __WFI();
  }
}
#include "board.h"
#include <ch32v30x_gpio.h>
#include "debug.h"
#include "delay.h"
#include <eth_driver.h>
#include <eth_queue.h>
#include <eth.h>
#include <common_printf.h>

const USART_InitTypeDef USART_InitStructure = {
  .USART_BaudRate = USART_BAUDRATE,
  .USART_WordLength = USART_WordLength_8b,
  .USART_StopBits = USART_StopBits_1,
  .USART_Parity = USART_Parity_No,
  .USART_HardwareFlowControl = USART_HardwareFlowControl_None,
  .USART_Mode = USART_Mode_Tx | USART_Mode_Rx
};

unsigned char MACAddr[6];

volatile unsigned int timeCnt;
volatile unsigned int timer_interrupt;

void __attribute__((interrupt("WCH-Interrupt-fast"))) USART_IRQHandler(void)
{
  if(USART_GetITStatus(USART_INST, USART_IT_RXNE) != RESET)
  {
    char c = (char)USART_ReceiveData(USART_INST);
    *rx_buffer_write_p++ = c;
    if (rx_buffer_write_p == rx_buffer + RX_BUF_LEN)
      rx_buffer_write_p = rx_buffer;
  }
}

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

static void RNG_Init(void)
{
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_RNG, ENABLE);
  RNG_Cmd(ENABLE);
}

static void USARTInit(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure = {0};
  NVIC_InitTypeDef  NVIC_InitStructure = {0};

  USART_PORT_CLOCK_ENABLE;
  USART_CLOCK_ENABLE;

  USART_REMAP;

  GPIO_InitStructure.GPIO_Pin = USART_TX_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(USART_PORT, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = USART_RX_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(USART_PORT, &GPIO_InitStructure);

  USART_Init(USART_INST, (USART_InitTypeDef*)&USART_InitStructure);
  USART_ITConfig(USART_INST, USART_IT_RXNE, ENABLE);

  NVIC_InitStructure.NVIC_IRQChannel = USART_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = USART_INTERRUPT_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  USART_Cmd(USART_INST, ENABLE);
}

void HalInit(const unsigned char* ntp_server_address)
{
  //Configure_Memory_Split();

  timeCnt = timer_interrupt = 0;

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  Delay_Init();

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

  GPIOInit();
  TIM2_Init();
  RNG_Init();
  USARTInit();

  ETH_QueueInit();
  WCHNET_GetMacAddr(MACAddr);
  ETH_Common_Init(MACAddr, ntp_server_address, nullptr, ETH_LOGLEVEL_NONE);
  ETH_Init(MACAddr);
}

void usart_transmit(char c)
{
  while(USART_GetFlagStatus(USART_INST, USART_FLAG_TXE) == RESET) /* waiting for sending finish */
    ;
  USART_SendData(USART_INST, c);
}

void puts_(const char *s)
{
  while (*s)
    usart_transmit(*s++);
}

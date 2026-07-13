#include "board.h"
#include <usart.h>
#include <common_printf.h>

const USART_InitTypeDef USART_InitStructure = {
  .USART_BaudRate = USART_BAUDRATE,
  .USART_WordLength = USART_WordLength_8b,
  .USART_StopBits = USART_StopBits_1,
  .USART_Parity = USART_Parity_No,
  .USART_HardwareFlowControl = USART_HardwareFlowControl_None,
  .USART_Mode = USART_Mode_Tx | USART_Mode_Rx
};

void USARTInit(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  NVIC_InitTypeDef  NVIC_InitStructure;

  USART_CLOCK_ENABLE;

  GPIO_InitStructure.GPIO_Pin = USART_TX_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
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

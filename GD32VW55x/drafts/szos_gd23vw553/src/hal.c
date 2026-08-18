#include "board.h"
#include "sys_timer.h"
#include <trng.h>
#include <crc32.h>
#include <common_printf.h>

static void GPIOInit(void)
{
  /* enable the LED clock */
  rcu_periph_clock_enable(LED_TIMER_CLOCK);
  /* configure LED GPIO port */
  gpio_mode_set(LED_TIMER_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_TIMER_PIN);
}

void USARTInit(void)
{
  /* enable COM GPIO clock */
  USART_PORT_CLOCK_ENABLE
  /* enable USART clock */
  USART_CLOCK_ENABLE;

  /* connect port to USART TX */
  gpio_af_set(USART_TX_PORT, USART_TX_AF, USART_TX_PIN);
  /* connect port to USART RX */
  gpio_af_set(USART_RX_PORT, USART_RX_AF, USART_RX_PIN);

  /* configure USART Tx as alternate function push-pull */
  gpio_mode_set(USART_TX_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, USART_TX_PIN);
  gpio_output_options_set(USART_TX_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, USART_TX_PIN);

  /* configure USART Rx as alternate function push-pull */
  gpio_mode_set(USART_RX_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, USART_RX_PIN);
  gpio_output_options_set(USART_RX_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, USART_RX_PIN);

  /* USART configuration */
  usart_deinit(USART_INST);
  usart_word_length_set(USART_INST, USART_WL_8BIT);
  usart_stop_bit_set(USART_INST, USART_STB_1BIT);
  usart_parity_config(USART_INST, USART_PM_NONE);
  usart_baudrate_set(USART_INST, USART_BAUDRATE);
  usart_receive_config(USART_INST, USART_RECEIVE_ENABLE);
  usart_transmit_config(USART_INST, USART_TRANSMIT_ENABLE);

  /* USART interrupt configuration */
  eclic_irq_enable(USART_IRQn, 1, USART_INTERRUPT_PRIORITY);

  usart_interrupt_enable(USART_INST, USART_INT_RBNE);

  usart_enable(USART_INST);
}

void HalInit(void)
{
  sys_timer_init();
  GPIOInit();
  USARTInit();
  //trng_init();
  crc32_init();
}

void USART_IRQHandler(void)
{
  if(RESET != usart_interrupt_flag_get(USART_INST, USART_INT_FLAG_RBNE))
  {
    unsigned char c = usart_data_receive(USART_INST);
    *rx_buffer_write_p++ = c;
    if (rx_buffer_write_p == rx_buffer + RX_BUF_LEN)
      rx_buffer_write_p = rx_buffer;
  }
}

void usart_transmit(char c)
{
  while(RESET == usart_flag_get(USART_INST, USART_FLAG_TBE))
    ;
  usart_data_transmit(USART_INST, c);
}

void puts_(const char *s)
{
  for (;;)
  {
    char c = *s++;
    if (!c)
      break;
    usart_transmit(c);
    if (c == '\n')
      usart_transmit('\r');
  }
  while(RESET == usart_flag_get(USART_INST, USART_FLAG_TBE))
    ;
}
/*void puts_(const char *s)
{
  puts(s);
}*/

void __attribute__((noreturn)) reboot(void)
{
  eclic_system_reset();
  while (1)
    __WFI();
}
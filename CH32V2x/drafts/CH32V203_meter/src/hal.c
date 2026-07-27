#include "board.h"
#include <ch32v20x_gpio.h>
#include <ch32v20x_tim.h>
#include <ch32v20x_misc.h>
#include "debug.h"
#include "usb_cdc.h"
#include "hw_config.h"
#include "usb_init.h"
#include "usb_pwr.h"
#include <i2c_soft.h>
#include <spi_soft.h>
#include <ina226.h>
#include <mcp3421.h>
#include <ads1115.h>
#include "dev_si5351.h"

typedef struct
{
  GPIO_TypeDef *port;
  unsigned int pin;
} Pin;

typedef struct
{
  Pin pins[6];
} ModulePredefinedInfo;

static const ModulePredefinedInfo module_predefined_info[MAX_DEVICES] =
{
  {
    .pins = {
      {PORT_PIN_SDA1, PIN_SDA1},
      {PORT_PIN_SCL1, PIN_SCL1},
      {PORT_PIN_1_0, PIN_1_0},
      {PORT_PIN_1_1, PIN_1_1},
      {PORT_PIN_1_2, PIN_1_2},
      {PORT_PIN_1_3, PIN_1_3}
    }
  },
{
    .pins = {
      {PORT_PIN_SDA2, PIN_SDA2},
      {PORT_PIN_SCL2, PIN_SCL2},
      {PORT_PIN_2_0, PIN_2_0},
      {PORT_PIN_2_1, PIN_2_1},
      {PORT_PIN_2_2, PIN_2_2},
      {PORT_PIN_2_3, PIN_2_3}
    }
  },
{
    .pins = {
      {PORT_PIN_SDA3, PIN_SDA3},
      {PORT_PIN_SCL3, PIN_SCL3},
      {PORT_PIN_3_0, PIN_3_0},
      {PORT_PIN_3_1, PIN_3_1},
      {PORT_PIN_3_2, PIN_3_2},
      {PORT_PIN_3_3, PIN_3_3}
    }
  },
{
    .pins = {
      {PORT_PIN_SDA4, PIN_SDA4},
      {PORT_PIN_SCL4, PIN_SCL4},
      {PORT_PIN_4_0, PIN_4_0},
      {PORT_PIN_4_1, PIN_4_1},
      {PORT_PIN_4_2, PIN_4_2},
      {PORT_PIN_4_3, PIN_4_3}
    }
  },
{
    .pins = {
      {PORT_PIN_SDA5, PIN_SDA5},
      {PORT_PIN_SCL5, PIN_SCL5},
      {PORT_PIN_5_0, PIN_5_0},
      {PORT_PIN_5_1, PIN_5_1},
      {PORT_PIN_5_2, PIN_5_2},
      {PORT_PIN_5_3, PIN_5_3}
    }
  },
};

static int led_state;
static volatile unsigned int time_hi;

void __attribute__((interrupt("WCH-Interrupt-fast"))) TIM1_UP_IRQHandler(void)
{
  time_hi++;
  TIM1->INTFR = 0;
}

void blink_led(void)
{
  led_state = !led_state;
  GPIO_WriteBit(GPIOB, GPIO_Pin_2, led_state);
}

void change_channel(int channel)
{
}

void SCL_HIGH(int channel)
{
  const Pin *pin = &module_predefined_info[channel].pins[PIN_SCL];
  pin->port->BSHR = pin->pin;
}

void SCL_LOW(int channel)
{
  const Pin *pin = &module_predefined_info[channel].pins[PIN_SCL];
  pin->port->BCR = pin->pin;
}

void SDA_HIGH(int channel)
{
  const Pin *pin = &module_predefined_info[channel].pins[PIN_SDA];
  pin->port->BSHR = pin->pin;
}

void SDA_LOW(int channel)
{
  const Pin *pin = &module_predefined_info[channel].pins[PIN_SDA];
  pin->port->BCR = pin->pin;
}

int SDA_IN(int channel)
{
  const Pin *pin = &module_predefined_info[channel].pins[PIN_SDA];
  return pin->port->INDR & pin->pin;
}

int SCL_IN(int channel)
{
  const Pin *pin = &module_predefined_info[channel].pins[PIN_SCL];
  return pin->port->INDR & pin->pin;
}

int SPI_CHECK_MISO(int channel)
{
  const Pin *pin = &module_predefined_info[channel].pins[PIN_MISO];
  return pin->port->INDR & pin->pin;
}

void SPI_MOSI_SET(int channel)
{
  const Pin *pin = &module_predefined_info[channel].pins[PIN_MOSI];
  pin->port->BSHR = pin->pin;
}

void SPI_MOSI_CLR(int channel)
{
  const Pin *pin = &module_predefined_info[channel].pins[PIN_MOSI];
  pin->port->BCR = pin->pin;
}

void SPI_CS_SET(int channel)
{
  const Pin *pin = &module_predefined_info[channel].pins[PIN_NCS];
  pin->port->BSHR = pin->pin;
}

void SPI_CS_CLR(int channel)
{
  const Pin *pin = &module_predefined_info[channel].pins[PIN_NCS];
  pin->port->BCR = pin->pin;
}

void SPI_CLK_SET(int channel)
{
  const Pin *pin = &module_predefined_info[channel].pins[PIN_SCK];
  pin->port->BSHR = pin->pin;
}

void SPI_CLK_CLR(int channel)
{
  const Pin *pin = &module_predefined_info[channel].pins[PIN_SCK];
  pin->port->BCR = pin->pin;
}

static void TIM1Init(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;

  time_hi = 0;
  // 65ms interrupt
  TIM_TimeBaseInitStructure.TIM_Period = 0xFFFF;
  TIM_TimeBaseInitStructure.TIM_Prescaler = SystemCoreClock/1000000-1;
  TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 1;
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);

  TIM_ClearITPendingBit(TIM1, TIM_IT_Update);

  NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; //low priority
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
  TIM_Cmd(TIM1, ENABLE);
}

static void TIM2Init(void)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;

  TIM_TimeBaseInitStructure.TIM_Period = 0xFFFF;
  TIM_TimeBaseInitStructure.TIM_Prescaler = 0;
  TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);
}

static void TIM3Init(void)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;

  TIM_TimeBaseInitStructure.TIM_Period = 0xFFFF;
  TIM_TimeBaseInitStructure.TIM_Prescaler = 0;
  TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);

  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
  TIM_OC4Init(TIM3, &TIM_OCInitStructure);

  TIM_OC4PreloadConfig( TIM3, TIM_OCPreload_Enable );
  TIM_ARRPreloadConfig( TIM3, ENABLE );
}

static void TIM4Init(void)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;

  TIM_TimeBaseInitStructure.TIM_Period = 0xFFFF;
  TIM_TimeBaseInitStructure.TIM_Prescaler = 0;
  TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);

  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
  TIM_OC1Init(TIM4, &TIM_OCInitStructure);

  TIM_OC1PreloadConfig( TIM4, TIM_OCPreload_Enable );
  TIM_ARRPreloadConfig( TIM4, ENABLE );
}

unsigned long long int time_us(void)
{
  unsigned int time_low;
  unsigned int time_high;
  do
  {
    time_high = time_hi;
    time_low = TIM1->CNT;
  } while (time_high != time_hi);
  return ((unsigned long long int)time_high << 16) | time_low;
}

void delay_us(unsigned int us)
{
  Delay_Us(us);
}

void main_comm_port_write_bytes(const unsigned char *data, int len)
{
  USB_CDC_Transmit(data, len);
}

int main_comm_port_read_bytes(unsigned char *buffer, int buffer_size)
{
  return USB_CDC_Receive(buffer, buffer_size);
}

void release_reset(void)
{
  PORT_RESET->BSHR = PIN_RESET;
}

static void spi_rx(int module_id, unsigned char *rxdata, unsigned int num_bytes)
{
  const Pin *pin_ncs = &module_predefined_info[module_id].pins[PIN_NCS];
  pin_ncs->port->BCR = pin_ncs->pin;
  while (num_bytes--)
    *rxdata++ = spi_byte(module_id, 0, 0);
  pin_ncs->port->BSHR = pin_ncs->pin;
}

static void spi_tx(int module_id, unsigned char subdevice_id, const unsigned char *txdata, unsigned int num_bytes)
{
  const Pin *pin_ncs = &module_predefined_info[module_id].pins[PIN_NCS];
  pin_ncs->port->BCR = pin_ncs->pin;
  spi_byte(module_id, subdevice_id, 0);
  while (num_bytes--)
    spi_byte(module_id, *txdata++, 0);
  pin_ncs->port->BSHR = pin_ncs->pin;
}

int spi_transfer(struct _DeviceObject *o, const void *txdata, unsigned int txdatalength, void *rxdata,
                                       unsigned int rxdatalength)
{
  int tx = txdatalength && txdata;
  int rx = rxdatalength && rxdata;
  if (tx || rx)
  {
    if (tx)
      spi_tx(o->idx, o->subdevice, txdata, txdatalength);
    if (tx && rx)
      delay_us(10);
    if (rx)
      spi_rx(o->idx, rxdata, rxdatalength);
  }
  return 0;
}

void init_spi(int module_id)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  const Pin *pin = &module_predefined_info[module_id].pins[PIN_MISO];
  GPIO_InitStructure.GPIO_Pin = pin->pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(pin->port, &GPIO_InitStructure);
  pin = &module_predefined_info[module_id].pins[PIN_MOSI];
  GPIO_InitStructure.GPIO_Pin = pin->pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(pin->port, &GPIO_InitStructure);
  pin = &module_predefined_info[module_id].pins[PIN_SCK];
  GPIO_InitStructure.GPIO_Pin = pin->pin;
  GPIO_Init(pin->port, &GPIO_InitStructure);
  SPI_CS_SET(module_id);
  pin = &module_predefined_info[module_id].pins[PIN_NCS];
  GPIO_InitStructure.GPIO_Pin = pin->pin;
  GPIO_Init(pin->port, &GPIO_InitStructure);
}

void init_interrupt_pin(const struct _DeviceObject *o)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  const Pin *pin = &module_predefined_info[o->idx].pins[o->transfer == i2c_transfer ? 2 : 4];
  GPIO_InitStructure.GPIO_Pin = pin->pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(pin->port, &GPIO_InitStructure);
}

int get_interrupt_pin_level(const struct _DeviceObject *o)
{
  const Pin *pin = &module_predefined_info[o->idx].pins[o->transfer == i2c_transfer ? 2 : 4];
  return pin->port->INDR & pin->pin;
}

int pwm_enable(int module_id, int pin_id, int enable)
{
  if (!enable)
  {
    switch (module_id)
    {
      case 0:
        TIM3->CH4CVR = 0;
        break;
      case 2:
        TIM4->CH1CVR = 0;
        break;
      default:
        break;
    }
  }
  return 0;
}

int pwm_set_frequency_and_duty(int module_id, int pin_id, unsigned short prescaler, unsigned int frequency, unsigned int duty)
{
  switch (module_id)
  {
    case 0:
      TIM3->PSC = (unsigned short)(prescaler - 1);
      TIM3->ATRLR = (unsigned short)frequency;
      TIM3->CH4CVR = (unsigned short)duty;
      break;
    case 2:
      TIM4->PSC = (unsigned short)(prescaler - 1);
      TIM4->ATRLR = (unsigned short)frequency;
      TIM4->CH1CVR = (unsigned short)duty;
      break;
    default:
      break;
  }
  return 0;
}

static void GPIOInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  led_state = 0;

  GPIO_InitStructure.GPIO_Pin = PIN_RESET;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(PORT_RESET, &GPIO_InitStructure);
}

static void I2CInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;

  for (int i = 0; i < MAX_DEVICES; i++)
  {
    i2c_soft_init(i);
    const Pin *pin = &module_predefined_info[i].pins[PIN_SCL];
    GPIO_InitStructure.GPIO_Pin = pin->pin;
    GPIO_Init(pin->port, &GPIO_InitStructure);
    pin = &module_predefined_info[i].pins[PIN_SDA];
    GPIO_InitStructure.GPIO_Pin = pin->pin;
    GPIO_Init(pin->port, &GPIO_InitStructure);
  }
}

void configure_hal(void)
{
  fSuspendEnabled = FALSE;

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  Delay_Init();

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO | RCC_APB2Periph_TIM1, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4, ENABLE);

  GPIOInit();
  I2CInit();

  Set_USBConfig();
  USB_Init();
  USB_Interrupts_Config();

  USB_Endp_Init();

  TIM1Init();
  TIM2Init(); // frequency counter
  TIM3Init(); // pwm
  TIM4Init(); // pwm
}

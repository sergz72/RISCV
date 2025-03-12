#include "board.h"
#include <ch32v20x_gpio.h>
#include "debug.h"
#include "delay.h"
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

#define PIN_SDA 0
#define PIN_SCL 1

#define PIN_MOSI 0
#define PIN_SCK  1
#define PIN_MISO 2
#define PIN_NCS  3

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

static void GPIOInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  led_state = 0;
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

int I2C_SendAddress(int idx, int address)
{
  i2c_soft_start(idx);

  if (i2c_soft_tx(idx, address, I2C_TIMEOUT)) // no ack
  {
    i2c_soft_stop(idx);
    return 1;
  }

  return 0;
}

int I2CCheck(int idx, int device_id)
{
  int rc;

  rc = I2C_SendAddress(idx, device_id);
  if (!rc)
    i2c_soft_stop(idx);
  return rc == 0;
}

int inaReadRegister(int channel, unsigned char address, unsigned char reg, unsigned short *data)
{
  unsigned char d[2];
  int rc = i2c_soft_command(channel, address, &reg, 1, NULL, 0, d, 2, I2C_TIMEOUT);
  if (!rc)
    *data = (d[0] << 8) | d[1];
  return rc;
}

int ads1115ReadRegister(int channel, unsigned char address, unsigned char reg, unsigned short *data)
{
  return inaReadRegister(channel, address, reg, data);
}

int mcp3421Read(int channel, unsigned char address, unsigned char *data, unsigned int l)
{
  return i2c_soft_read(channel, address, data, l, I2C_TIMEOUT);
}

int inaWriteRegister(int channel, unsigned char address, unsigned char reg, unsigned short data)
{
  unsigned char d[2];
  d[0] = data >> 8;
  d[1] = data & 0xFF;
  return i2c_soft_command(channel, address, &reg, 1, d, 2, NULL, 0, I2C_TIMEOUT);
}

int ads1115WriteRegister(int channel, unsigned char address, unsigned char reg, unsigned short data)
{
  return inaWriteRegister(channel, address, reg, data);
}

int mcp3421Write(int channel, unsigned char address, unsigned char data)
{
  return i2c_soft_command(channel, address, NULL, 0, &data, 1, NULL, 0, I2C_TIMEOUT);
}

unsigned long long int time_us(void)
{
  //todo
  return 0;
}

void delay_us(unsigned int us)
{
  Delay_Us(us);
}

int si5351_write(unsigned char device_address, int channel, const unsigned char *data, unsigned int length)
{
  return i2c_soft_command(channel, device_address, NULL, 0, data, length, NULL, 0, I2C_TIMEOUT);
}

int mcp9600Read16(int channel, unsigned char address, unsigned char reg, unsigned short *data)
{
  return inaReadRegister(channel, address, reg, data);
}

int mcp9600Read8(int channel, unsigned char address, unsigned char reg, unsigned char *data)
{
  return i2c_soft_command(channel, address, &reg, 1, NULL, 0, data, 1, I2C_TIMEOUT);
}

int mcp9600Write(int channel, unsigned char address, unsigned char reg, unsigned char data)
{
  return i2c_soft_command(channel, address, &reg, 1, &data, 1, NULL, 0, I2C_TIMEOUT);
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

int i2c_transfer(struct _DeviceObject *o, const void *txdata, unsigned int txdatalength, void *rxdata,
                        unsigned int rxdatalength)
{
  return i2c_soft_command(o->idx, o->device->device_id, (unsigned char*)&o->subdevice, 1,
                    txdata, txdatalength, rxdata, rxdatalength, I2C_TIMEOUT);
}

int spi_transfer(struct _DeviceObject *o, const void *txdata, unsigned int txdatalength, void *rxdata,
                                       unsigned int rxdatalength)
{
  //todo
  return 0;
}

void init_spi(int module_id)
{
  //todo
}

void init_interrupt_pin(const struct _DeviceObject *o)
{
  //todo
}

int get_interrupt_pin_level(const struct _DeviceObject *o)
{
  //todo
  return 0;
}

void configure_hal(void)
{
  fSuspendEnabled = FALSE;

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  Delay_Init();

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

  GPIOInit();

  Set_USBConfig();
  USB_Init();
  USB_Interrupts_Config();

  USB_Endp_Init();
  //todo
}

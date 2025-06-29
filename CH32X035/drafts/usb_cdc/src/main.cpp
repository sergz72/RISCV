#include "board.h"
#include "delay.h"
#include <usb_device.h>
#include <usb_device_ch32x035.h>
#include <cdc_class.h>
#include <stdlib.h>

static const USBDeviceConfiguration configuration =
{
  .device_class = usb_class_misc, // composite device
  .vendor_id = 1155,
  .product_id =  22336,
  .manufacturer = "STMicroelectronics",
  .product = "STM32 Virtual ComPort",
  .serial_number = "00000000001A",
  .language_id = 1033
};

static const USBConfigurationDescriptor configuration_descriptor = {
  .configuration_name = "CDC Configuration",
  .self_powered = 1,
  .remote_wakeup = 0,
  .max_power = 100
};

int led_state;
static USB_Device_CH32X035 usb_device;
static USB_DeviceManager usb_device_manager(&configuration, &configuration_descriptor, &usb_device);
static USB_CDC_Class cdc_class(&usb_device_manager, 1024);
static unsigned char cdc_buffer[1024];

extern "C" {
  #if __GNUC__ > 13
  void __attribute__((naked)) void USBFS_IRQHandler(void)
  #else
  void __attribute__((interrupt("WCH-Interrupt-fast"))) USBFS_IRQHandler(void)
  #endif
  {
    usb_device.InterruptHandler();
  #if __GNUC__ > 13
    asm volatile ("mret");
  #endif
  }

  static void led_toggle(void)
  {
    led_state = !led_state;
    if (led_state)
      LED_ON;
    else
      LED_OFF;
  }

  void _GLOBAL__sub_I_led_state(void);

  int main(void)
  {
    _GLOBAL__sub_I_led_state();

    int cnt = 0;

    SysInit();

    led_state = 0;

    if (cdc_class.DescriptorBuilder(2) || usb_device_manager.Init() || !malloc(1))
    {
      LED_ON;
      while (1)
        Delay_Ms(1000);
    }

    while (1)
    {
      Delay_Ms(10);
      unsigned int length = cdc_class.GetPendingData(0, cdc_buffer, sizeof(cdc_buffer));
      if (length)
        cdc_class.Send(0, cdc_buffer, length);
      length = cdc_class.GetPendingData(1, cdc_buffer, sizeof(cdc_buffer));
      if (length)
        cdc_class.Send(1, cdc_buffer, length);
      cnt++;
      if (cnt == 100)
      {
        led_toggle();
        cnt = 0;
      }
    }
  }
}
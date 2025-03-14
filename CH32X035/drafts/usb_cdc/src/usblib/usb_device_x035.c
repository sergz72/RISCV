#include "board.h"
#include "usb_device_x035.h"
#include "ch32x035_usb.h"
#include "usb_device.h"

#define NUM_ENDPOINTS 8

#define USB_IOEN     0x80
#define USB_PHY_V33  0x40

#define UDP_PUE_MASK 0x0C
#define UDP_PUE_1K5  0x0C

#define UDM_PUE_MASK 0x03

const char x035_endpoints[USB_MAX_ENDPOINTS] = {1,1,1,1,0,1,1,1,0,0,0,0,0,0,0,0};

static __attribute__ ((aligned(4))) unsigned char endpoint_buffers[NUM_ENDPOINTS][USB_FS_MAX_PACKET_SIZE];

static void USBFS_Device_Endp_Init(void)
{
  USBFSD->UEP0_DMA = (unsigned int)endpoint_buffers[0];
  USBFSD->UEP1_DMA = (unsigned int)endpoint_buffers[1];
  USBFSD->UEP2_DMA = (unsigned int)endpoint_buffers[2];
  USBFSD->UEP3_DMA = (unsigned int)endpoint_buffers[3];
  USBFSD->UEP5_DMA = (unsigned int)endpoint_buffers[5];
  USBFSD->UEP6_DMA = (unsigned int)endpoint_buffers[6];
  USBFSD->UEP7_DMA = (unsigned int)endpoint_buffers[7];
}

void USBInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_APB2PeriphClockCmd( RCC_APB2Periph_AFIO, ENABLE );

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_16;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_17;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  // VDD = 3.3v, D+ line pullup = 1.5k
  AFIO->CTLR = (AFIO->CTLR & ~(UDP_PUE_MASK | UDM_PUE_MASK )) | USB_PHY_V33 | UDP_PUE_1K5 | USB_IOEN;

  USBFSD->BASE_CTRL = 0;
  USBFS_Device_Endp_Init();
  USBFSD->DEV_ADDR = 0;
  USBFSD->BASE_CTRL = USBFS_UC_DEV_PU_EN | USBFS_UC_INT_BUSY | USBFS_UC_DMA_EN;
  USBFSD->INT_FG = 0xFF;
  USBFSD->UDEV_CTRL = USBFS_UD_PD_DIS | USBFS_UD_PORT_EN;
  USBFSD->INT_EN = USBFS_UIE_SUSPEND | USBFS_UIE_BUS_RST | USBFS_UIE_TRANSFER;

  NVIC_EnableIRQ(USBFS_IRQn);
}

int USBReadInterruptEndpointNumber(void)
{
  //todo
  return -1;
}

int USBIsTransactionDirectionIN(int endpoint)
{
  //todo
  return 0;
}

int USBIsSetupTransaction(int endpoint)
{
  //todo
  return 0;
}

void USBEnableEndpoint(unsigned int endpoint)
{
  //todo
}

void USBActivateEndpoint(unsigned int endpoint, unsigned int length)
{
  //todo
}

void USBStallEndpoint(unsigned int endpoint)
{
  //todo
}

void *USBGetEndpointInBuffer(int endpoint)
{
  return endpoint_buffers[endpoint];
}

void *USBGetEndpointOutBuffer(int endpoint)
{
  return endpoint_buffers[endpoint];
}

void USBSetEndpointTransferType(int endpoint, USBEndpointTransferType transfer_type)
{
  //todo
}

void USBClearInterruptFlags(void)
{
  USBFSD->INT_FG = 0xFF;
}

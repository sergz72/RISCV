#include "board.h"
#include "debug.h"
#include "ch32v30x_usbfs_device.h"
#include "delay.h"
#include <usb_cdc.h>
#include <stdarg.h>
#include <shell.h>
#include <getstring.h>
#include <eth_driver.h>

static unsigned char usb_cdc_buffer[USB_CDC_RX_BUFFER_SIZE];
static char command_line[200];
static volatile unsigned int cdc_length;
static unsigned char *cdc_buffer_p;
static int led_state;
static unsigned char socket_buf[RECE_BUF_LEN];

void puts_(const char *s)
{
  CDC_Transmit((unsigned char*)s, strlen(s));
}

int usb_printf(const char *format, ...)
{
  static char buffer[PRINTF_BUFFER_LENGTH], buffer2[PRINTF_BUFFER_LENGTH];
  char *p, *p2;
  va_list vArgs;
  int rc;

  va_start(vArgs, format);
  rc = vsnprintf(buffer, sizeof(buffer), format, vArgs);
  va_end(vArgs);
  p = buffer;
  p2 = buffer2;
  while (*p)
  {
    if (*p == '\n')
      *p2++ = '\r';
    *p2++ = *p++;
  }
  *p2 = 0;
  puts_(buffer2);
  return rc;
}

static int getch_(void)
{
  if (!cdc_length)
    return EOF;
  cdc_length--;
  return *cdc_buffer_p++;
}

static void LEDTimerToggle(void)
{
  led_state = !led_state;
  if (led_state)
    LED_BLUE_ON;
  else
    LED_BLUE_OFF;
}

/*********************************************************************
 * @fn      WCHNET_HandleSockInt
 *
 * @brief   Socket Interrupt Handle
 *
 * @param   socketid - socket id.
 *          intstat - interrupt status
 *
 * @return  none
 */
void WCHNET_HandleSockInt(u8 socketid,u8 intstat)
{
  u32 len;

  if(intstat & SINT_STAT_RECV)                           //receive data
  {
    len = WCHNET_SocketRecvLen(socketid,NULL);         //get socket buffer data length
    WCHNET_SocketRecv(socketid,socket_buf,&len);            //Read the data of the receive buffer into MyBuf
  }
  if(intstat & SINT_STAT_CONNECT)                        //connect successfully
  {
    /***/
  }
  if(intstat & SINT_STAT_DISCONNECT)                     //disconnect
  {
    /***/
  }
  if(intstat & SINT_STAT_TIM_OUT)                        //timeout disconnect
  {
    /***/
  }
}

/*********************************************************************
 * @fn      WCHNET_HandleGlobalInt
 *
 * @brief   Global Interrupt Handle
 *
 * @return  none
 */
void WCHNET_HandleGlobalInt(void)
{
  u8 intstat;
  u16 i;
  u8 socketint;

  intstat = WCHNET_GetGlobalInt();                              //get global interrupt flag
  if (intstat & GINT_STAT_UNREACH)                              //Unreachable interrupt
  {
    usb_printf("GINT_STAT_UNREACH\n");
  }
  if (intstat & GINT_STAT_IP_CONFLI)                            //IP conflict
  {
    usb_printf("GINT_STAT_IP_CONFLI\n");
  }
  if (intstat & GINT_STAT_PHY_CHANGE)                           //PHY status change
  {
    i = WCHNET_GetPHYStatus();
    if(i&PHY_Linked_Status){
      usb_printf("PHY Link Success\n");
    }
  }
  if (intstat & GINT_STAT_SOCKET) {                             //socket related interrupt
    for (i = 0; i < WCHNET_MAX_SOCKET_NUM; i++) {
      socketint = WCHNET_GetSocketInt(i);
      if (socketint)
        WCHNET_HandleSockInt(i, socketint);
    }
  }
}

int main(void)
{
  int rc;

  led_state = 0;

  HalInit();

  USBFS_RCC_Init( );
  USBFS_Device_Init( ENABLE );

  shell_init(usb_printf, NULL);

  getstring_init(command_line, sizeof(command_line), getch_, puts_);

  while (1)
  {
    /*Ethernet library main task function,
     * which needs to be called cyclically*/
    WCHNET_MainTask();
    /*Query the Ethernet global interrupt,
     * if there is an interrupt, call the global interrupt handler*/
    if(WCHNET_QueryGlobalInt())
    {
      WCHNET_HandleGlobalInt();
    }

    if (timer_interrupt)
    {
      timer_interrupt = 0;
      if ((timeCnt & 32) == 0)
        LEDTimerToggle();
      cdc_length = CDC_Receive(usb_cdc_buffer, sizeof(usb_cdc_buffer));
      cdc_buffer_p = usb_cdc_buffer;
      while (cdc_length)
      {
        if (!getstring_next())
        {
          switch (command_line[0])
          {
          case SHELL_UP_KEY:
            puts_("\r\33[2K$ ");
            getstring_buffer_init(shell_get_prev_from_history());
            break;
          case SHELL_DOWN_KEY:
            puts_("\r\33[2K$ ");
            getstring_buffer_init(shell_get_next_from_history());
            break;
          default:
            rc = shell_execute(command_line);
            if (rc == 0)
              puts_("OK\r\n$ ");
            else if (rc < 0)
              puts_("Invalid command line\r\n$ ");
            else
              usb_printf("shell_execute returned %d\n$ ", rc);
            break;
          }
        }
      }
    }
  }
}

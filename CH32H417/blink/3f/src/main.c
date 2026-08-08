#include <ch32h417.h>
#include <delay.h>
#include <core_riscv.h>
#include <debug.h>

#define LED1_PORT GPIOC
#define LED1_PIN  GPIO_Pin_2
#define LED1_OFF  LED1_PORT->BSHR = LED1_PIN
#define LED1_ON   LED1_PORT->BCR  = LED1_PIN

#define LED2_PORT GPIOC
#define LED2_PIN  GPIO_Pin_3
#define LED2_OFF  LED2_PORT->BSHR = LED2_PIN
#define LED2_ON   LED2_PORT->BCR  = LED2_PIN

#define LCD_DC_PORT GPIOD
#define LCD_DC_PIN  GPIO_Pin_9
#define LCD_DC_SET  LCD_DC_PORT->BSHR = LCD_DC_PIN
#define LCD_DC_CLR  LCD_DC_PORT->BCR  = LCD_DC_PIN

#define LCD_CS_PORT GPIOB
#define LCD_CS_PIN  GPIO_Pin_12
#define LCD_CS_SET  LCD_CS_PORT->BSHR = LCD_CS_PIN
#define LCD_CS_CLR  LCD_CS_PORT->BCR  = LCD_CS_PIN

#define LCD_CLK_PORT GPIOB
#define LCD_CLK_PIN  GPIO_Pin_13
#define LCD_CLK_SET  LCD_CLK_PORT->BSHR = LCD_CLK_PIN
#define LCD_CLK_CLR  LCD_CLK_PORT->BCR  = LCD_CLK_PIN

#define LCD_MOSI_PORT GPIOB
#define LCD_MOSI_PIN  GPIO_Pin_15
#define LCD_MOSI_SET  LCD_MOSI_PORT->BSHR = LCD_MOSI_PIN
#define LCD_MOSI_CLR  LCD_MOSI_PORT->BCR  = LCD_MOSI_PIN

#define LCD_RST_PORT GPIOD
#define LCD_RST_PIN  GPIO_Pin_8
#define LCD_RST_SET  LCD_RST_PORT->BSHR = LCD_RST_PIN
#define LCD_RST_CLR  LCD_RST_PORT->BCR  = LCD_RST_PIN

#define LED_CLOCK_ENABLE RCC_HB2PeriphClockCmd(RCC_HB2Periph_GPIOC, ENABLE)
#define LCD_CLOCK_ENABLE RCC_HB2PeriphClockCmd(RCC_HB2Periph_GPIOB|RCC_HB2Periph_GPIOD, ENABLE)

void GPIOInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  LED_CLOCK_ENABLE;

  LED1_OFF;
  GPIO_InitStructure.GPIO_Pin = LED1_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Low;
  GPIO_Init(LED1_PORT, &GPIO_InitStructure);

  LED2_OFF;
  GPIO_InitStructure.GPIO_Pin = LED2_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Low;
  GPIO_Init(LED2_PORT, &GPIO_InitStructure);

  LCD_CLOCK_ENABLE;

  GPIO_InitStructure.GPIO_Pin = LCD_DC_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Very_High;
  GPIO_Init(LCD_DC_PORT, &GPIO_InitStructure);

  LCD_CS_SET;
  GPIO_InitStructure.GPIO_Pin = LCD_CS_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Very_High;
  GPIO_Init(LCD_CS_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = LCD_CLK_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Very_High;
  GPIO_Init(LCD_CLK_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = LCD_MOSI_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Very_High;
  GPIO_Init(LCD_MOSI_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = LCD_RST_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Very_High;
  GPIO_Init(LCD_RST_PORT, &GPIO_InitStructure);
}

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void)
{
  SystemInit();
  SystemAndCoreClockUpdate();
  Delay_Init();

  GPIOInit();

  NVIC_WakeUp_V5F(Core_V5F_StartAddr); //wake up V5

  bool state = false;

  while (1)
  {
    if (state)
      LED1_ON;
    else
      LED1_OFF;
    state = !state;
    Delay_Ms(500);
  }
}

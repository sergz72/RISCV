#ifndef _BOARD_H
#define _BOARD_H

#ifndef NULL
#define NULL 0
#endif

#include <ch32v00x.h>

#define CPU_CLOCK_LOW  3000000
#define CPU_CLOCK_HIGH 24000000

#define I2C_TIMEOUT 10000
#define I2C_INST I2C1
#define I2C_CLOCK RCC_APB1Periph_I2C1
/*
 *PC4 = PWM
 */
#define PWM_FREQUENCY 100000
#define PWM_PIN GPIO_Pin_7
#define PWM_PORT GPIOC
#define PWM_PERIOD (CPU_CLOCK_HIGH/PWM_FREQUENCY)
#define PWM_OCINIT TIM_OC2Init
#define PWM_OCPRELOADCONFIG TIM_OC2PreloadConfig
#define PWM_TIMER_CLOCK RCC_APB2Periph_TIM1
#define PWM_TIM TIM1

/*
 *PD6 = LED_TIMER
 */
#define LED_TIMER_PIN GPIO_Pin_3
#define LED_TIMER_PORT GPIOC
#define LED_TIMER_OFF GPIOC->BCR = LED_TIMER_PIN
#define LED_TIMER_ON GPIOC->BSHR = LED_TIMER_PIN

#define VBAT_PIN     GPIO_Pin_4
#define VBAT_PORT    GPIOC
#define VBAT_CHANNEL ADC_Channel_2

/*
 *PD6 = LED_BATTERY
 */
#define LED_BATTERY_PIN GPIO_Pin_6
#define LED_BATTERY_PORT GPIOC
#define LED_BATTERY_OFF GPIOC->BCR = LED_BATTERY_PIN
#define LED_BATTERY_ON GPIOC->BSHR = LED_BATTERY_PIN

#define TIMER_EVENT_FREQUENCY 512

#define TIM_TIMER TIM2
#define TIM_TIMER_CLOCK RCC_APB1Periph_TIM2
#define TIM_TIMER_HANDLER TIM2_IRQHandler
#define TIM_TIMER_IRQ TIM2_IRQn
#define TIM_TIMER_PRESCALER 63
#define TIM_TIMER_PERIOD_LOW (CPU_CLOCK_LOW/(TIM_TIMER_PRESCALER+1)/TIMER_EVENT_FREQUENCY - 1)
#define TIM_TIMER_PERIOD_HIGH (CPU_CLOCK_HIGH/(TIM_TIMER_PRESCALER+1)/TIMER_EVENT_FREQUENCY - 1)

#define OPA_IN_PORT  GPIOA
#define OPA_P_PIN    GPIO_Pin_2
#define OPA_N_PIN    GPIO_Pin_1
#define OPA_OUT_PORT GPIOD
#define OPA_OUT_PIN  GPIO_Pin_4

#define USART_TX_PIN      GPIO_Pin_5
#define USART_RX_PIN      GPIO_Pin_6
#define USART_PORT        GPIOD
#define USART_BAUDRATE    4800
#define USART_BUFFER_SIZE 128

#define COMMMAND_LINE_SIZE 50
#define PRINTF_BUFFER_LENGTH 100
#define USE_MYVSPRINTF

#define TIMER_INTERRUPT_PRIORITY 0
#define USART_INTERRUPT_PRIORITY 1

#define PIR_SENSOR_FILTER_THRESHOLD               15
#define PIR_SENSOR_AVERAGING_FILTER_SAMPLES_COUNT TIMER_EVENT_FREQUENCY
//#define PIR_SENSOR_FILTER_CRS                     1

#define LIGHT_SENSOR_HIGH_THRESHOLD 30
#define LIGHT_SENSOR_LOW_THRESHOLD  10

#define VBAT_3V0 726 // 3000 mv
#define VBAT_2V9 702 // 2900 mv
#define VBAT_BELOW_3V0(v) (v < VBAT_3V0)
#define VBAT_BELOW_2V9(v) (v < VBAT_2V9)
#define VBAT_TO_DUTY(v) (185+(((871-v)*226)>>10))

#define IWDG_PRESCALER IWDG_Prescaler_32
#define IWDG_RELOAD    (40000/TIMER_EVENT_FREQUENCY)

//#define USART_ENABLED
#define SENSOR_ENABLED

#ifdef USART_ENABLED
#define MOTION_DETECTOR_ON_TIME 2
#else
#define MOTION_DETECTOR_ON_TIME 10
#endif

void SysInit(void);
//void disable_i2c(void);
//void enable_i2c(void);
void enable_pwm(uint16_t pulse);
void disable_pwm(void);
void disable_adc(void);
void enable_adc(void);
unsigned short adc_get(void);
void set_low_system_clock(void);
void set_high_system_clock(void);
void usart_transmit(char c);
void delayms(int ms);
int i2c_write(unsigned char address, unsigned char *data, unsigned int l, unsigned int timeout, bool stop);
int i2c_read(unsigned char address, unsigned char *data, unsigned int l, unsigned int timeout);
int light_sensor_init(void);
int light_sensor_read(unsigned short *result);
unsigned short get_vbat(void);
void pwm_on(unsigned short duty);
void pwm_off(void);
unsigned short pwm_set_duty(unsigned short duty);
void pwm_auto(unsigned int vbat);
void iwdg_init(void);

extern volatile bool timer_interrupt;
extern volatile char command_line[COMMMAND_LINE_SIZE];
extern volatile char *command_line_p, *command_line_echo_p;
extern volatile bool command_ready;

#include "light_sensor_veml7700.h"

#endif

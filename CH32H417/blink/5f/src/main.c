#include <ch32h417.h>
#include <delay.h>

#define EVAL
#ifdef NANO
#define LED2_PORT GPIOC
#define LED2_PIN  GPIO_Pin_3
#endif

#ifdef EVAL
#define LED2_PORT GPIOE
#define LED2_PIN  GPIO_Pin_3
#endif

#define LED2_OFF  LED2_PORT->BSHR = LED2_PIN
#define LED2_ON   LED2_PORT->BCR  = LED2_PIN

int main(void)
{
    SystemAndCoreClockUpdate();
    Delay_Init();

    bool state = false;

    while(1)
    {
        if (state)
            LED2_OFF;
        else
            LED2_ON;
        state = !state;
        Delay_Ms(1000);
    }
}

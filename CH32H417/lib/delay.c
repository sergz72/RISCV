#include "ch32h417.h"
#include <delay.h>

static unsigned short  p_us = 0;
static unsigned int p_ms = 0;

/*********************************************************************
 * @fn      Delay_Init
 *
 * @brief   Initializes Delay Funcation.
 *
 * @return  none
 */
void Delay_Init(void)
{
    p_us = HCLKClock / 1000000;
    p_ms = (unsigned short)p_us * 1000;
}

/*********************************************************************
 * @fn      Delay_Us
 *
 * @brief   Microsecond Delay Time.
 *
 * @param   n - Microsecond number.
 *
 * @return  none
 */
void Delay_Us(unsigned int n)
{
    unsigned int i;
#ifdef Core_V3F
    SysTick0->ISR &= ~(1 << 0);
    i = (unsigned int)n * p_us;

    SysTick0->CNT = 0;
    SysTick0->CMP = i;
    SysTick0->CTLR = (1 << 2);
    SysTick0->CTLR |= (1 << 0);

    while((SysTick0->ISR & (1 << 0)) != (1 << 0))
        ;
    SysTick0->CTLR &= ~(1 << 0);

#elif defined(Core_V5F)
    SysTick0->ISR &= ~(1 << 1);
    i = (unsigned int)n * p_us;

    SysTick1->CNT = 0;
    SysTick1->CMP = i;
    SysTick1->CTLR = (1 << 2);
    SysTick1->CTLR |= (1 << 0);

    while((SysTick0->ISR & (1 << 1)) != (1 << 1))
        ;
    SysTick1->CTLR &= ~(1 << 0);
#endif
}

/*********************************************************************
 * @fn      Delay_Ms
 *
 * @brief   Millisecond Delay Time.
 *
 * @param   n - Millisecond number.
 *
 * @return  none
 */
void Delay_Ms(unsigned int n)
{
    unsigned int i;
#ifdef Core_V3F
    SysTick0->ISR &= ~(1 << 0);
    i = (unsigned int)n * p_ms;

    SysTick0->CNT = 0;
    SysTick0->CMP = i;
    SysTick0->CTLR = (1 << 2);
    SysTick0->CTLR |= (1 << 0);

    while((SysTick0->ISR & (1 << 0)) != (1 << 0))
        ;
    SysTick0->CTLR &= ~(1 << 0);
#elif defined(Core_V5F)
    SysTick0->ISR &= ~(1 << 1);
    i = (unsigned int)n * p_ms;

    SysTick1->CNT = 0;
    SysTick1->CMP = i;
    SysTick1->CTLR = (1 << 2);
    SysTick1->CTLR |= (1 << 0);

    while((SysTick0->ISR & (1 << 1)) != (1 << 1))
        ;
    SysTick1->CTLR &= ~(1 << 0);
#endif
}

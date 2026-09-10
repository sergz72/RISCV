const rcc = @import("rcc");
const gpio = @import("gpio");
const afio = @import("afio");
const usart = @import("usart");
const cpu = @import("cpu");
const system_timer = @import("system_timer");
const pfic = @import("pfic");

const LED_PIN: u32 = 0;
const LED_PIN_MASK: u32 = 1 << LED_PIN;
const LED_PORT = gpio.gpiod;

const TX_PIN: u32 = 6;
const TX_PIN_MASK: u32 = 1 << TX_PIN;
const TX_PORT = gpio.gpiob;

const RX_PIN: u32 = 7;
const RX_PIN_MASK: u32 = 1 << RX_PIN;
const RX_PORT = gpio.gpiob;

export fn USART1_IRQHandler() callconv(.naked) void {
    if (usart.usart1.statr.rxne) {
        const data = usart.usart1.datar;
        usart.usart1.datar = data;
    }
    asm volatile("mret");
}

export fn main() callconv(.c) noreturn {
    system_timer.delay_init();
    rcc.rcc.apb2pcenr = rcc.RccCfgrApb2pcEnr{.iopden = true, .iopben = true, .afioen = true, .usart1en = true};
    afio.afio.pcfr1 = afio.AfioPcfr1{.pd01_rm = true, .usart1_rm = true};
    LED_PORT.Init(LED_PIN_MASK, gpio.GpioModeOutputSlowSpeed | gpio.GpioCnfOutputPushPull);
    TX_PORT.Init(TX_PIN_MASK, gpio.GpioModeOutputFastSpeed | gpio.GpioCnfAlternatePushPull);
    RX_PORT.bshr = RX_PIN_MASK; // pullup
    RX_PORT.Init(RX_PIN_MASK, gpio.GpioCnfInputPullupPulldown);
    usart.usart1.init(115200, cpu.cpu.current_frequency);
    pfic.pfic.interrupt_enable(pfic.Interrupt.USART1);
    while (true) {
        gpio.gpiod.bshr = LED_PIN_MASK;
        system_timer.delayms(1000);
        gpio.gpiod.bcr = LED_PIN_MASK;
        system_timer.delayms(1000);
    }
}

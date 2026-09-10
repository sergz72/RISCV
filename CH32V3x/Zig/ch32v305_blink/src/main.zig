const rcc = @import("rcc");
const gpio = @import("gpio");
const afio = @import("afio");
const system_timer = @import("system_timer");

const LED_PIN: u32 = 0;
const LED_PIN_MASK: u32 = 1 << LED_PIN;

export fn main() callconv(.c) noreturn {
    system_timer.delay_init();
    rcc.rcc.apb2pcenr.iopden = true;
    rcc.rcc.apb2pcenr.afioen = true;
    afio.afio.pcfr1.pd01_rm = true;
    gpio.gpiod.Init(LED_PIN_MASK, gpio.GpioModeOutputSlowSpeed | gpio.GpioCnfOutputPushPull);
    while (true) {
        gpio.gpiod.bshr = LED_PIN_MASK;
        system_timer.delayms(1000);
        gpio.gpiod.bcr = LED_PIN_MASK;
        system_timer.delayms(1000);
    }
}

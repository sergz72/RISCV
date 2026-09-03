const sio = @import("sio");
const resets = @import("resets");

comptime {
    _ = @import("interrupts");
}

fn delay(ms: usize) void {
    for (0..ms*2398) |_| {
        asm volatile("nop");
    }
}

export fn main() callconv(.c) noreturn {
    resets.unreset(resets.ResetFields{.io_bank0=true, .pads_bank0=true});
    sio.sio.gpioLowOutputEnable(sio.DEFAULT_LED_PIN_MASK);
    sio.io_bank0.gpioFunctionSet(sio.DEFAULT_LED_PIN, sio.GpioFunc.sio_0);
    sio.pads_bank0.gpioIsolationRemove(sio.DEFAULT_LED_PIN);

    while (true) {
        sio.sio.gpioLowToggle(sio.DEFAULT_LED_PIN_MASK);
        delay(250);
    }
}

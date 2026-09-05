const std = @import("std");
const sio = @import("sio");
const resets = @import("resets");
const interrupts = @import("interrupts");

const CPU_FREQUENCY = 12000000;

var mtimecmp_next: u64 = undefined;
var time_ms: u64 = undefined;

export fn MachineTimerInterruptHandler() callconv(.c) void {
    mtimecmp_next += CPU_FREQUENCY / 1000;
    sio.sio.set_mtimecmp(mtimecmp_next);
    time_ms += 1;
}

fn delay(ms: usize) void {
    time_ms = 0;
    while (time_ms < ms) {
        asm volatile("wfi");
    }
}

export fn main_cpu0() callconv(.c) noreturn {
    resets.unreset(resets.ResetFields{.io_bank0=true, .pads_bank0=true});
    sio.sio.gpioLowOutputEnable(sio.DEFAULT_LED_PIN_MASK);
    sio.io_bank0.gpioFunctionSet(sio.DEFAULT_LED_PIN, sio.GpioFunc.sio_0);
    sio.pads_bank0.gpioIsolationRemove(sio.DEFAULT_LED_PIN);

    sio.sio.mtime_ctrl.fullspeed = true;

    interrupts.interrupt_enable(interrupts.MieRegister{.meie = true, .mtie = true});
    interrupts.global_interrupt_enable();

    mtimecmp_next = sio.sio.get_mtime() + CPU_FREQUENCY / 1000;
    sio.sio.set_mtimecmp(mtimecmp_next);

    while (true) {
        sio.sio.gpioLowToggle(sio.DEFAULT_LED_PIN_MASK);
        delay(250);
    }
}

export fn main_cpu1() callconv(.c) noreturn {
    mtimecmp_next = sio.sio.get_mtime() + CPU_FREQUENCY / 1000;

    sio.sio.mtime_ctrl.fullspeed = true;

    interrupts.interrupt_enable(interrupts.MieRegister{.meie = true, .mtie = true});
    interrupts.global_interrupt_enable();

    while (true) {
        //sio.sio.gpioLowToggle(sio.DEFAULT_LED_PIN_MASK);
        delay(250);
    }
}

pub fn panic(msg: []const u8, error_return_trace: ?*std.builtin.StackTrace, ret_addr: ?usize) noreturn {
    _ = msg;
    _ = error_return_trace;
    _ = ret_addr;
    // For embedded, loop infinitely or trigger a hardware reset
    while (true) {}
}
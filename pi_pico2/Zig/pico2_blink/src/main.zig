const std = @import("std");
const sio = @import("sio");
const resets = @import("resets");
const interrupts = @import("interrupts");
const uart = @import("uart");

const CPU_FREQUENCY = 12000000;

const LED2_PIN: u32 = 21;
const LED2_PIN_MASK: u32 = 1 << LED2_PIN;

extern var __CORE1_STACK_TOP: anyopaque;
extern fn _VectoredInterruptVectorTable() callconv(.c) void;
extern fn cpu1_reset_handler() callconv(.c) void;

var mtimecmp_next: u64 = undefined;
var time_ms: u32 = undefined;

export fn MachineTimerInterruptHandler() callconv(.c) void {
    mtimecmp_next += CPU_FREQUENCY / 1000;
    sio.sio.set_mtimecmp(mtimecmp_next);
    time_ms += 1;
}

fn delay(ms: usize) void {
    const start = time_ms;
    while (time_ms - start < ms) {
        asm volatile("wfi");
    }
}

export fn main_cpu0() callconv(.c) noreturn {
    time_ms = 0;
    resets.unreset(resets.ResetFields{.io_bank0=true, .pads_bank0=true});
    sio.sio.gpioLowOutputEnable(sio.DEFAULT_LED_PIN_MASK);
    sio.io_bank0.gpioFunctionSet(sio.DEFAULT_LED_PIN, sio.GpioFunc.sio_0);
    sio.pads_bank0.gpioIsolationRemove(sio.DEFAULT_LED_PIN);

    sio.sio.gpioLowOutputEnable(LED2_PIN_MASK);
    sio.io_bank0.gpioFunctionSet(LED2_PIN, sio.GpioFunc.sio_0);
    sio.pads_bank0.gpioIsolationRemove(LED2_PIN);

    sio.sio.mtime_ctrl.fullspeed = true;

    interrupts.interrupt_enable(interrupts.MieRegister{.meie = true, .mtie = true});
    interrupts.global_interrupt_enable();

    mtimecmp_next = sio.sio.get_mtime() + CPU_FREQUENCY / 1000;
    sio.sio.set_mtimecmp(mtimecmp_next);

    uart.uart_writer.print("Hello from Embedded Zig!\n", .{}) catch {};

    sio.sio.launch_core1(@intFromPtr(&_VectoredInterruptVectorTable), @intFromPtr(&__CORE1_STACK_TOP), @intFromPtr(&cpu1_reset_handler));

    while (true) {
        sio.sio.gpioLowToggle(sio.DEFAULT_LED_PIN_MASK);
        delay(250);
    }
}

export fn main_cpu1() callconv(.c) noreturn {
    interrupts.interrupt_enable(interrupts.MieRegister{.meie = true, .mtie = true});
    interrupts.global_interrupt_enable();

    mtimecmp_next = sio.sio.get_mtime() + CPU_FREQUENCY / 1000;
    sio.sio.set_mtimecmp(mtimecmp_next);

    while (true) {
        sio.sio.gpioLowToggle(LED2_PIN_MASK);
        delay(1000);
    }
}

// pub fn panic(msg: []const u8, error_return_trace: ?*std.builtin.StackTrace, ret_addr: ?usize) noreturn {
//     _ = msg;
//     _ = error_return_trace;
//     _ = ret_addr;
//     // For embedded, loop infinitely or trigger a hardware reset
//     while (true) {}
// }
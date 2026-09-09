const cpu = @import("cpu");
const pfic = @import("pfic");

const SYSTICK_BASE: usize = 0xE000F000;

var p_us: u32 = undefined;
var p_ms: u32 = undefined;
var systick_interrupt: bool = undefined;

const SystickCtlr = packed struct(u32) {
    ste: bool = false,
    stie: bool = false,
    stclk_hclk: bool = false,
    stre: bool = false,
    reserved: u27 = 0,
    swie: bool = false
};

const Systick = extern struct {
    ctlr: SystickCtlr,
    sr: u32,
    cntr: u32,
    reserved: u32,
    cmpr: u32
};

const systick: *volatile Systick = @ptrFromInt(SYSTICK_BASE);

pub fn delay_init() void {
    p_ms = cpu.cpu.current_frequency / 1000;
    p_us = p_ms / 1000;
    pfic.pfic.interrupt_enable(pfic.Interrupt.SysTick);
}

export fn SysTick_Handler() callconv(.naked) void {
    systick_interrupt = true;
    systick.sr = 0;
    asm volatile("mret");
}

fn delay(n: u32) void {
    systick.ctlr = SystickCtlr{};
    systick_interrupt = false;
    systick.cntr = 0;
    systick.cmpr = n;
    systick.ctlr = SystickCtlr{.ste = true, .stie = true, .stclk_hclk = true};
    while (!systick_interrupt) {
        asm volatile ("wfi");
    }
    systick.ctlr = SystickCtlr{};
}

pub fn delayms(ms: usize) void {
    delay(ms * p_ms);
}

pub fn delayus(us: usize) void {
    delay(us * p_us);
}

const TICKS_BASE: u32 = 0x40108000;

pub const TickCtrl = packed struct(u32) {
    enable: bool,
    running: bool,
    _reserved: u30
};

pub const TickCycles = packed struct(u32) {
    value: u9,
    _reserved: u23
};

pub const Tick = extern struct {
    ctrl: TickCtrl,
    cycles: TickCycles,
    count: TickCycles
};

pub const Ticks = extern struct {
    proc:     [2]Tick,
    timer:    [2]Tick,
    watchdog: Tick,
    riscv:    Tick
};

pub const ticks: *volatile Ticks = @ptrFromInt(TICKS_BASE);

test "sizeof test" {
    const std = @import("std");
    try std.testing.expectEqual(0x48, @sizeOf(Ticks));
}

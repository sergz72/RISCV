export fn SystemInit() callconv(.c) void {
}

pub const Cpu = struct {
    current_frequency: usize
};

pub var cpu = Cpu{.current_frequency = 8000000};

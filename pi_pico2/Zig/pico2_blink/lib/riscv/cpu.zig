const interrupts = @import("interrupts");

pub const Cpu = struct {
    current_frequency: usize,

    pub fn sev() void {
        asm volatile ("slt x0, x0, x1"
            :
            :
            : .{ .memory = true });
    }

    pub fn wfe() void {
        asm volatile ("slt x0, x0, x0"
            :
            :
            : .{ .memory = true });
    }

    pub fn interrupts_enable() void {
        interrupts.interrupt_enable(interrupts.MieRegister{.meie = true, .mtie = true});
        interrupts.global_interrupt_enable();
    }
};

pub var cpu = Cpu{.current_frequency = 11000000};

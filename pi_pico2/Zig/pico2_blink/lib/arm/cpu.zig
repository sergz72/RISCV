pub const Cpu = struct {
    current_frequency: usize,

    pub fn sev() void {
        asm volatile ("sev");
    }

    pub fn wfe() void {
        asm volatile ("wfe");
    }

    pub fn interrupts_enable() void {
        //todo
    }
};

pub var cpu = Cpu{.current_frequency = 11000000};

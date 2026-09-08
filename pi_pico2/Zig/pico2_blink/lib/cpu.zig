pub const Cpu = struct {
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
};

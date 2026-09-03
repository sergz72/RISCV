const WRITE_NORMAL: u32 = 0x0000;   // normal read write access
const WRITE_XOR: u32    = 0x1000;   // atomic XOR on write
const WRITE_SET: u32    = 0x2000;   // atomic bitmask set on write
const WRITE_CLR: u32    = 0x3000;   // atomic bitmask clear on write

pub fn IORegisters(comptime T: type) type {
    return struct {
        const Self = @This();

        normal: *volatile T,
        xor: *volatile T,
        set: *volatile T,
        clr: *volatile T,

        // 2. Add an initialization method
        pub fn init(address: u32) Self {
            return Self{
                .normal = @ptrFromInt(address),
                .xor = @ptrFromInt(address + WRITE_XOR),
                .set = @ptrFromInt(address + WRITE_SET),
                .clr = @ptrFromInt(address + WRITE_CLR)
            };
        }
    };
}

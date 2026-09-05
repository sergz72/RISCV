const SIO_BASE: u32         = 0xd0000000;
const IO_BANK0_BASE: u32    = 0x40028000;
const PADS_BANK0_BASE: u32  = 0x40038000;

pub const DEFAULT_LED_PIN: u32 = 25;
pub const DEFAULT_LED_PIN_MASK: u32 = 1 << DEFAULT_LED_PIN;

const Interpolator = extern struct {
    accum: [2]u32,
    base: [3]u32,
    pop_lane: [2]u32,
    pop_full: u32,
    peek_lane: [2]u32,
    peek_full: u32,
    ctrl_lane: [2]u32,
    accum_add: [2]u32,
    interp0_base_1and0: u32
};

const SioMask = extern struct {
    low: u32,
    high: u32
};

pub const RiscvSoftIrq = packed struct(u32) {
    core0_set: bool = false,
    core1_set: bool = false,
    core0_clr: bool = false,
    core1_clr: bool = false,
    _reserved: u28 = 0
};

pub const MTimeCtrl = packed struct(u32) {
    en: bool = false,
    fullspeed: bool = false,
    dbgpause_core0: bool = false,
    dbgpause_core1: bool = false,
    _reserved: u28 = 0
};

pub const Sio = extern struct {
    // 0x000
    cpuid: u32,
    // 0x004
    gpio_in: SioMask,

    // 0x00c - reserved
    _reserved_00c: u32,

    // 0x010
    gpio_out: SioMask,
    // 0x018
    gpio_out_set: SioMask,
    // 0x020
    gpio_out_clr: SioMask,
    // 0x028
    gpio_out_xor: SioMask,

    // 0x030
    gpio_oe: SioMask,
    // 0x038
    gpio_oe_set: SioMask,
    // 0x040
    gpio_oe_clr: SioMask,
    // 0x048
    gpio_oe_xor: SioMask,

    // 0x050
    fifo_st: u32,
    // 0x054
    fifo_wr: u32,
    // 0x058
    fifo_rd: u32,
    // 0x05c
    spinlock_st: u32,

    // 0x060 - 0x07c reserved
    _reserved_060: [8]u32,

    // Interpolators
    interpolator: [2]Interpolator,

    // Spinlocks 0..31
    // 0x100
    spinlock: [32]u32,

    doorbell_out_set: u32,
    doorbell_out_clr: u32,
    doorbell_in_set: u32,
    doorbell_in_clr: u32,

    peri_nonsec: u32,

    _reserved_194: [3]u32,

    riscv_softirq: RiscvSoftIrq,

    mtime_ctrl: MTimeCtrl,
    _reserved_1a8: [2]u32,
    mtime: u32,
    mtimeh: u32,
    mtimecmp: u32,
    mtimecmph: u32,

    tmds_ctrl: u32,
    tmds_wdata: u32,
    tmds_peek_single: u32,
    tmds_pop_single: u32,
    tmds_peek_double_l0: u32,
    tmds_pop_double_l0: u32,
    tmds_peek_double_l1: u32,
    tmds_pop_double_l1: u32,
    tmds_peek_double_l2: u32,
    tmds_pop_double_l2: u32,

    pub inline fn gpioSet(self: *volatile Sio, mask: SioMask) void {
        self.gpio_out_set = mask;
    }

    pub inline fn gpioLowSet(self: *volatile Sio, mask: u32) void {
        self.gpio_out_set.low = mask;
    }

    pub inline fn gpioHighSet(self: *volatile Sio, mask: u32) void {
        self.gpio_out_set.high = mask;
    }

    pub inline fn gpioClear(self: *volatile Sio, mask: SioMask) void {
        self.gpio_out_clr = mask;
    }

    pub inline fn gpioLowClear(self: *volatile Sio, mask: u32) void {
        self.gpio_out_clr.low = mask;
    }

    pub inline fn gpioHighClear(self: *volatile Sio, mask: u32) void {
        self.gpio_out_clr.high = mask;
    }

    pub inline fn gpioToggle(self: *volatile Sio, mask: SioMask) void {
        self.gpio_out_xor = mask;
    }

    pub inline fn gpioLowToggle(self: *volatile Sio, mask: u32) void {
        self.gpio_out_xor.low = mask;
    }

    pub inline fn gpioHighToggle(self: *volatile Sio, mask: u32) void {
        self.gpio_out_xor.high = mask;
    }

    pub inline fn gpioRead(self: *volatile Sio) SioMask {
        return self.gpio_in;
    }

    pub inline fn gpioLowRead(self: *volatile Sio) u32 {
        return self.gpio_in.low;
    }

    pub inline fn gpioHighRead(self: *volatile Sio) u32 {
        return self.gpio_in.high;
    }

    pub inline fn gpioOutputEnable(self: *volatile Sio, mask: SioMask) void {
        self.gpio_oe_set = mask;
    }

    pub inline fn gpioLowOutputEnable(self: *volatile Sio, mask: u32) void {
        self.gpio_oe_set.low = mask;
    }

    pub inline fn gpioHighOutputEnable(self: *volatile Sio, mask: u32) void {
        self.gpio_oe_set.high = mask;
    }

    pub inline fn gpioOutputDisable(self: *volatile Sio, mask: SioMask) void {
        self.gpio_oe_clr = mask;
    }

    pub inline fn gpioLowOutputDisable(self: *volatile Sio, mask: u32) void {
        self.gpio_oe_clr.low = mask;
    }

    pub inline fn gpioHighOutputDisable(self: *volatile Sio, mask: u32) void {
        self.gpio_oe_clr.high = mask;
    }

    pub inline fn gpioWrite(self: *volatile Sio, mask: SioMask, value: bool) void {
        if (value) {
            self.gpio_out_set = mask;
        } else {
            self.gpio_out_clr = mask;
        }
    }

    pub inline fn gpioLowWrite(self: *volatile Sio, mask: u32, value: bool) void {
        if (value) {
            self.gpio_out_set.low = mask;
        } else {
            self.gpio_out_clr.low = mask;
        }
    }

    pub inline fn gpioHighWrite(self: *volatile Sio, mask: u32, value: bool) void {
        if (value) {
            self.gpio_out_set.high = mask;
        } else {
            self.gpio_out_clr.high = mask;
        }
    }

    pub fn get_mtime(self: *volatile Sio) u64 {
        while (true) {
            const high = self.mtimeh;
            const low = self.mtime;
            const high2 = self.mtimeh;
            if (high == high2)
                return @as(u64, low) | (@as(u64, high) << 32);
        }
    }

    pub fn set_mtimecmp(self: *volatile Sio, value: u64) void {
        self.mtimecmp = 0xFFFFFFFF;
        self.mtimecmph = @intCast(value >> 32);
        self.mtimecmp = @intCast(value & 0xFFFFFFFF);
    }
};

/// Bitfields for the GPIO Status Register (GPIOx_STATUS)
const GpioStatus = packed struct(u32) {
    _reserved0: u9 = 0,               // Bits 0-8
    outtopad: bool,                   // Bit 9
    _reserved1: u3 = 0,               // Bits 10-12
    oe_to_pad: bool,                  // Bit 13
    _reserved2: u3 = 0,               // Bits 14-16
    infrompad: bool,                  // Bit 17
    _reserved3: u8 = 0,               // Bits 18-25
    irq_to_proc: bool,                // Bit 26
    _reserved4: u5 = 0                // Bits 27-31
};

pub const GpioFunc = enum(u5) {
    jtag_tck                    = 0x00,
    spi0_rx                     = 0x01,
    uart0_tx                    = 0x02,
    i2c0_sda                    = 0x03,
    pwm_a_0                     = 0x04,
    sio_0                       = 0x05,
    pio0_0                      = 0x06,
    pio1_0                      = 0x07,
    pio2_0                      = 0x08,
    xip_ss_n_1                  = 0x09,
    usb_muxing_overcurr_detect  = 0x0a,
    /// Disabled / Disconnected state
    null                        = 0x1f
};

/// Bitfields for the GPIO Control Register (GPIOx_CTRL)
const GpioCtrl = packed struct(u32) {
    /// Function select (0-31). Check datasheet for specific pin mux maps.
    funcsel: GpioFunc,                // Bits 0-4
    _reserved0: u7 = 0,               // Bits 5-11
    /// Out override (0: normal, 1: invert, 2: low, 3: high)
    outover: u2,                      // Bits 12-13
    /// Output enable override (0: normal, 1: invert, 2: disable, 3: enable)
    oeover: u2,                       // Bits 14-15
    /// Input override (0: normal, 1: invert, 2: low, 3: high)
    inover: u2,                       // Bits 16-17
    _reserved1: u10 = 0,              // Bits 18-27
    /// Interrupt override (0: normal, 1: invert, 2: low, 3: high)
    irqover: u2,                      // Bits 28-29
    _reserved2: u2 = 0                // Bits 30-31
};

/// Individual Pin Channel
const GpioChannel = extern struct {
    status: GpioStatus,
    ctrl: GpioCtrl,
};

const IrqSummary = extern struct {
    secure:    [2]u32,
    nonsecure: [2]u32
};

const InterruptControl = extern struct {
    enable: [6]u32,
    force:  [6]u32,
    status: [6]u32
};

pub const IoBank = extern struct {
    /// Array matching registers from GPIO0 through GPIO47
    gpio:             [48]GpioChannel,            // Offsets: 0x000 to 0x178
    _reserved:        [32]u32,
    irq_summary_proc: [2]IrqSummary,
    irq_summary_coma: IrqSummary,
    intr:             [6]u32,
    proc_intr:        [2]InterruptControl,
    dormant_wake:     InterruptControl,

    pub inline fn gpioFunctionSet(self: *volatile IoBank, gpio_num: u32, func: GpioFunc) void {
        self.gpio[gpio_num].ctrl.funcsel = func;
    }
};

const PadsVoltageSelect = packed struct(u32) {
    voltage_select_1v8: bool,
    _reserved: u31 = 0
};

const PadsGpio = packed struct(u32) {
    slewfast: bool,
    schmitt: bool,
    pde: bool,
    pue: bool,
    drive: u2,
    ie: bool,
    od: bool,
    iso: bool,
    _reserved: u23 = 0
};

pub const PadsBank = extern struct {
    voltage_select: PadsVoltageSelect,
    gpio: [48]PadsGpio,
    swclk: PadsGpio,
    swd: PadsGpio,

    pub inline fn gpioIsolationRemove(self: *volatile PadsBank, gpio_num: u32) void {
        self.gpio[gpio_num].iso = false;
    }
};

pub const sio: *volatile Sio = @ptrFromInt(SIO_BASE);
pub const io_bank0: *volatile IoBank = @ptrFromInt(IO_BANK0_BASE);
pub const pads_bank0: *volatile PadsBank = @ptrFromInt(PADS_BANK0_BASE);

test "sizeof test" {
    const std = @import("std");
    try std.testing.expectEqual(0x1E8, @sizeOf(Sio));
    try std.testing.expectEqual(0x320, @sizeOf(IoBank));
    try std.testing.expectEqual(0xCC, @sizeOf(PadsBank));
}

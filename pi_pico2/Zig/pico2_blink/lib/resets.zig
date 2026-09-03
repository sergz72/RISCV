const io_registers = @import("io_registers");

const RESETS_BASE: u32 = 0x40020000;

pub const ResetFields = packed struct(u32) {
    adc: bool = false,
    busctrl: bool = false,
    dma: bool = false,
    hstx: bool = false,
    i2c0: bool = false,
    i2c1: bool = false,
    io_bank0: bool = false,
    io_qspi: bool = false,
    jtag: bool = false,
    pads_bank0: bool = false,
    pads_qspi: bool = false,
    pio0: bool = false,
    pio1: bool = false,
    pio2: bool = false,
    pll_sys: bool = false,
    pll_usb: bool = false,
    pwm: bool = false,
    sha256: bool = false,
    spi0: bool = false,
    spi1: bool = false,
    syscfg: bool = false,
    sysinfo: bool = false,
    tbman: bool = false,
    timer0: bool = false,
    timer1: bool = false,
    trng: bool = false,
    uart0: bool = false,
    uart1: bool = false,
    usbctrl: bool = false,
    _padding: u3 = 0,
};

const Resets = extern struct {
    reset: ResetFields,
    wdsel: ResetFields,
    reset_done: ResetFields,
};

pub const resets = io_registers.IORegisters(Resets).init(RESETS_BASE);

pub fn unreset(mask: ResetFields) void {
    resets.clr.reset = mask;
    const mask_u32 = @as(u32, @bitCast(mask));
    while ((@as(u32, @bitCast(resets.normal.reset_done)) & mask_u32) != mask_u32) {
    }
}

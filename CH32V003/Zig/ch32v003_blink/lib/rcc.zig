const RCC_BASE: usize = 0x40021000;

pub const RccCtlr = packed struct(u32) {
    hsion: bool,
    hsirdy: bool,
    reserved: u2,
    hsitrim: u5,
    hsical: u8,
    hseon: bool,
    hsebyp: bool,
    csson: bool,
    reserved2: u4,
    pllon: bool,
    pllrdy: bool,
    reserved3: u6
};

pub const RccCfgr0Sw = enum(u2) {
    hsi = 0,
    hse = 1,
    pll = 2
};

pub const RccCfgrHpre = enum(u4) {
    off = 0,
    div2 = 1,
    div3 = 2,
    div4 = 3,
    div5 = 4,
    div6 = 5,
    div7 = 6,
    div8 = 7,
    div16 = 11,
    div32 = 12,
    div64 = 13,
    div128 = 14,
    div256 = 15
};

pub const RccCfgrAdcpre = enum(u5) {
    div2 = 0,
    div4 = 4,
    div6 = 0b10000,
    div8 = 12,
    div12 = 0b10100,
    div16 = 0b11100,
    div24 = 0b10101,
    div32 = 0b11101,
    div48 = 0b10110,
    div64 = 0b11110,
    div96 = 0b10111,
    div128 = 0b11111
};

pub const RccCfgrMco = enum(u3) {
    off = 0,
    sysclk = 4,
    hsi = 5,
    hse = 6,
    pll = 7
};

pub const RccCfgr0 = packed struct(u32) {
    sw: RccCfgr0Sw,
    sws: RccCfgr0Sw,
    hpre: RccCfgrHpre,
    reserved: u3 = 0,
    adcpre: RccCfgrAdcpre,
    pplsrc_hse: bool,
    resetved2: u7 = 0,
    mco: RccCfgrMco,
    reserved3: u5
};

pub const RccCfgrAhbpcEnr = packed struct(u32) {
    dme1en: bool = false,
    reserved: u1 = 0,
    sramen: bool = false,
    reserved2: u29 = 0
};

pub const RccCfgrApb2pcEnr = packed struct(u32) {
    afioen: bool = false,
    reserved: u1 = 0,
    iopaen: bool = false,
    reserved2: u1 = 0,
    iopcen: bool = false,
    iopden: bool = false,
    reserved3: u3 = 0,
    adc1en: bool = false,
    reserved4: u1 = 0,
    tim1en: bool = false,
    spi1en: bool = false,
    reserved5: u1 = 0,
    usart1en: bool = false,
    reserved6: u17 = 0
};

pub const RccCfgrApb1pcEnr = packed struct(u32) {
    tim2en: bool = false,
    reserved: u10 = 0,
    wwdgen: bool = false,
    reserved2: u9 = 0,
    i2c1en: bool = false,
    reserved3: u6 = 0,
    pwren: bool = false,
    reserved4: u3 = 0
};


pub const Rcc = extern struct {
    ctlr: RccCtlr,
    cfgr0: RccCfgr0,
    intr: u32,
    apb2prstr: u32,
    apb1prstr: u32,
    ahbpcenr: RccCfgrAhbpcEnr,
    apb2pcenr: RccCfgrApb2pcEnr,
    apb1pcenr: RccCfgrApb1pcEnr,
    reserved: u32,
    rstsckr: u32
};

pub const rcc: *volatile Rcc = @ptrFromInt(RCC_BASE);

test "sizeof test" {
    const std = @import("std");
    try std.testing.expectEqual(0x28, @sizeOf(Rcc));
}

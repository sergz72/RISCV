const XOSC_BASE: u32 = 0x40048000;

pub const XoscCtrlFreqRange = enum(u12) {
    one_to_15_mhz = 0xaa0,
    ten_to_30_mhz = 0xaa1,
    twenty_five_to_60_mhz = 0xaa2,
    forty_to_100_mhz = 0xaa3
};

pub const XoscEnable = enum(u12) {
    disable = 0xd1e,
    enable = 0xfab
};

pub const XoscCtrl = packed struct(u32) {
    freq_range: XoscCtrlFreqRange,
    enable: XoscEnable,
    _reserved: u8 = 0
};

pub const XoscStatusFreqRange = enum(u2) {
    one_to_15_mhz = 0,
    ten_to_30_mhz = 1,
    twenty_five_to_60_mhz = 2,
    forty_to_100_mhz = 3
};

pub const XoscStatus = packed struct(u32) {
    freq_range: XoscStatusFreqRange,
    _reserved: u10,
    enabled: bool,
    _reserved2: u11,
    badwrite: bool,
    _reserved3: u6,
    stable: bool
};

pub const XoscDormant = enum(u32) {
    dormant = 0x636f6d61,
    wake = 0x77616b65
};

pub const XoscStartup = packed struct(u32) {
    delay: u14,
    reserved: u6,
    x4: bool,
    reserved2: u11 = 0
};

pub const XoscCount = packed struct(u32) {
    counter: u16,
    reserved: u16 = 0
};

pub const Xosc = extern struct {
    ctrl: XoscCtrl,
    status: XoscStatus,
    dormant: XoscDormant,
    startup: XoscStartup,
    count: XoscCount,

    //todo: fix
    noinline fn get_status(self: *volatile Xosc) XoscStatus {
        return self.status;
    }

    pub fn init(self: *volatile Xosc, xosc_freq_range: XoscStatusFreqRange) void {
        const range_int: u12 = @intFromEnum(xosc_freq_range);
        const range: XoscCtrlFreqRange = @enumFromInt(range_int | 0xaa0);
        self.ctrl = XoscCtrl{.freq_range = range, .enable = XoscEnable.enable};
        while (true) {
            const status = self.get_status();
            if ((status.freq_range == xosc_freq_range) & status.enabled & status.stable) {
                break;
            }
        }
    }
};

pub const xosc: *volatile Xosc = @ptrFromInt(XOSC_BASE);
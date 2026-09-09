const cpu = @import("cpu");
const sio = @import("sio");

var mtimecmp_next: u64 = undefined;
var time_ms: u32 = undefined;

export fn MachineTimerInterruptHandler() callconv(.c) void {
    mtimecmp_next += cpu.cpu.current_frequency / 1000;
    sio.sio.set_mtimecmp(mtimecmp_next);
    time_ms += 1;
}

pub fn delay(ms: usize) void {
    const start = time_ms;
    while (time_ms - start < ms) {
        asm volatile("wfi");
    }
}

pub fn init_system_timer() void {
    time_ms = 0;

    sio.sio.mtime_ctrl.fullspeed = true;
}

pub fn start_system_timer() void {
    mtimecmp_next = sio.sio.get_mtime() + cpu.cpu.current_frequency / 1000;
    sio.sio.set_mtimecmp(mtimecmp_next);
}
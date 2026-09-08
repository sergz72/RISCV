const std = @import("std");

inline fn uartWrite(byte: u8) void {
    _ = byte;
    //todo
}

fn uartWriteByte(byte: u8) void {
    if (byte == '\n') {
        uartWrite('\r');
    }
    uartWrite(byte);
}

fn drain(w: *std.Io.Writer, data: []const []const u8, splat: usize) std.Io.Writer.Error!usize {
    _ = w;
    var n: usize = 0;
    for (data[0 .. data.len - 1]) |chunk| {
        for (chunk) |b| uartWriteByte(b);
        n += chunk.len;
    }
    const last = data[data.len - 1];
    for (0..splat) |_| {
        for (last) |b| uartWriteByte(b);
        n += last.len;
    }
    return n;
}

pub var uart_writer: std.Io.Writer = .{
    .vtable = &.{ .drain = drain },
    .buffer = &.{}, // zero-length -> every call goes straight to drain, unbuffered
};
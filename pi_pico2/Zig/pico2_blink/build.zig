const std = @import("std");

fn get_module_name(allocator: std.mem.Allocator, target_name: []const u8, module_name: []const u8) ![]u8 {
    return try std.fmt.allocPrint(allocator, "{s}_{s}", .{target_name, module_name});
}

fn build_target(b: *std.Build, target_name: []const u8, target: std.Build.ResolvedTarget,
                        optimize: std.builtin.OptimizeMode) !void {
    const interrupts = b.addModule(try get_module_name(b.allocator, target_name, "interrupts"), .{
        .root_source_file = b.path(try std.fmt.allocPrint(b.allocator, "lib/{s}/interrupts.zig", .{target_name})),
        .target = target,
        .optimize = optimize
    });

    const cpu = b.addModule(try get_module_name(b.allocator, target_name, "cpu"), .{
        .root_source_file = b.path(try std.fmt.allocPrint(b.allocator, "lib/{s}/cpu.zig", .{target_name})),
        .target = target,
        .optimize = optimize,
        .imports = &.{
            .{ .name = "interrupts", .module = interrupts }
        }
    });

    const sio = b.addModule(try get_module_name(b.allocator, target_name, "sio"), .{
        .root_source_file = b.path("lib/sio.zig"),
        .target = target,
        .optimize = optimize,
        .imports = &.{
            .{ .name = "cpu", .module = cpu }
        }
    });

    const system_timer = b.addModule(try get_module_name(b.allocator, target_name, "system_timer"), .{
        .root_source_file = b.path(try std.fmt.allocPrint(b.allocator, "lib/{s}/system_timer.zig", .{target_name})),
        .target = target,
        .optimize = optimize,
        .imports = &.{
            .{ .name = "cpu", .module = cpu },
        .{ .name = "sio", .module = sio }
        }
    });

    const xosc = b.addModule(try get_module_name(b.allocator, target_name, "xosc"), .{
        .root_source_file = b.path("lib/xosc.zig"),
        .target = target,
        .optimize = optimize
    });

    const uart = b.addModule(try get_module_name(b.allocator, target_name, "uart"), .{
        .root_source_file = b.path("lib/uart.zig"),
        .target = target,
        .optimize = optimize
    });

    const io_registers = b.addModule(try get_module_name(b.allocator, target_name, "io_registers"), .{
        .root_source_file = b.path("lib/io_registers.zig"),
        .target = target,
        .optimize = optimize
    });

    const ticks = b.addModule(try get_module_name(b.allocator, target_name, "ticks"), .{
        .root_source_file = b.path("lib/ticks.zig"),
        .target = target,
        .optimize = optimize
    });

    const resets = b.addModule(try get_module_name(b.allocator, target_name, "resets"), .{
        .root_source_file = b.path("lib/resets.zig"),
        .target = target,
        .optimize = optimize,
        .imports = &.{
            .{ .name = "io_registers", .module = io_registers }
        }
    });

    const riscv_exe = b.addExecutable(.{
        .name = try std.fmt.allocPrint(b.allocator, "pico2_blink_{s}.elf", .{target_name}),
        .root_module = b.createModule(.{
            .root_source_file = b.path("src/main.zig"),
            .target = target,
            .optimize = optimize,
            .imports = &.{
                .{ .name = "sio", .module = sio },
                .{ .name = "ticks", .module = ticks },
                .{ .name = "interrupts", .module = interrupts },
                .{ .name = "resets", .module = resets },
                .{ .name = "uart", .module = uart },
                .{ .name = "xosc", .module = xosc },
                .{ .name = "cpu", .module = cpu },
                .{ .name = "system_timer", .module = system_timer },
            },
        }),
    });

    riscv_exe.link_gc_sections = true;
    riscv_exe.link_function_sections = true;
    riscv_exe.link_data_sections = true;
    riscv_exe.lto = .full;                     // Whole-program optimization & inlining

    riscv_exe.root_module.addAssemblyFile(b.path(try std.fmt.allocPrint(b.allocator, "lib/{s}/start.s", .{target_name})));
    riscv_exe.root_module.addAssemblyFile(b.path(try std.fmt.allocPrint(b.allocator, "lib/{s}/interrupts.s", .{target_name})));

    riscv_exe.setLinkerScript(b.path("lib/Memory_Map.ld"));

    riscv_exe.entry = .{ .symbol_name = "_entry_point" };

    b.installArtifact(riscv_exe);

    const riscv_size_report = b.addSystemCommand(&.{ "llvm-size-22" });
    riscv_size_report.addArtifactArg(riscv_exe);

    b.getInstallStep().dependOn(&riscv_size_report.step);
}

pub fn build(b: *std.Build) !void {
    var riscv_features_add = std.Target.Cpu.Feature.Set.empty;
    const riscv_features = std.Target.riscv.Feature;

    riscv_features_add.addFeature(@intFromEnum(riscv_features.m));     // Multiply/Divide
    riscv_features_add.addFeature(@intFromEnum(riscv_features.a));     // Atomics
    riscv_features_add.addFeature(@intFromEnum(riscv_features.c));     // Compressed Instructions
    riscv_features_add.addFeature(@intFromEnum(riscv_features.zba));   // Address generation
    riscv_features_add.addFeature(@intFromEnum(riscv_features.zbb));   // Basic bit manipulation
    riscv_features_add.addFeature(@intFromEnum(riscv_features.zbs));   // Single-bit manipulation
    riscv_features_add.addFeature(@intFromEnum(riscv_features.zbkb));
    riscv_features_add.addFeature(@intFromEnum(riscv_features.zcb));
    riscv_features_add.addFeature(@intFromEnum(riscv_features.zcmp));
    riscv_features_add.addFeature(@intFromEnum(riscv_features.zicsr));
    riscv_features_add.addFeature(@intFromEnum(riscv_features.relax));

    var riscv_features_sub = std.Target.Cpu.Feature.Set.empty;
    riscv_features_sub.addFeature(@intFromEnum(riscv_features.f));
    riscv_features_sub.addFeature(@intFromEnum(riscv_features.d));
    riscv_features_sub.addFeature(@intFromEnum(riscv_features.zcf));

    const riscv_target = b.resolveTargetQuery(.{
        .cpu_arch = .riscv32,
        .os_tag = .freestanding,
        .abi = .ilp32,
        .cpu_features_add = riscv_features_add,
        .cpu_features_sub = riscv_features_sub
    });

    const arm_target = b.resolveTargetQuery(.{
        .cpu_arch = .thumb,
        .os_tag = .freestanding,
        .abi = .eabi,
        .cpu_model = .{ .explicit = &std.Target.arm.cpu.cortex_m33 }
    });

    const optimize = b.standardOptimizeOption(.{});

    try build_target(b, "riscv", riscv_target, optimize);
    try build_target(b, "arm", arm_target, optimize);
}

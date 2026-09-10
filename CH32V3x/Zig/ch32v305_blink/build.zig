const std = @import("std");

pub fn build(b: *std.Build) !void {
    var riscv_features_add = std.Target.Cpu.Feature.Set.empty;
    const riscv_features = std.Target.riscv.Feature;

    riscv_features_add.addFeature(@intFromEnum(riscv_features.i));
    riscv_features_add.addFeature(@intFromEnum(riscv_features.m));
    riscv_features_add.addFeature(@intFromEnum(riscv_features.a));
    riscv_features_add.addFeature(@intFromEnum(riscv_features.c));     // Compressed Instructions
    riscv_features_add.addFeature(@intFromEnum(riscv_features.f));
    riscv_features_add.addFeature(@intFromEnum(riscv_features.zicsr));
    riscv_features_add.addFeature(@intFromEnum(riscv_features.relax));

    var riscv_features_sub = std.Target.Cpu.Feature.Set.empty;
    riscv_features_sub.addFeature(@intFromEnum(riscv_features.d));
    riscv_features_sub.addFeature(@intFromEnum(riscv_features.zcf));

    const target = b.resolveTargetQuery(.{
        .cpu_arch = .riscv32,
        .os_tag = .freestanding,
        .abi = .ilp32,
        .cpu_features_add = riscv_features_add,
        .cpu_features_sub = riscv_features_sub
    });

    const optimize = b.standardOptimizeOption(.{});

    const flash = b.addModule("flash", .{
        .root_source_file = b.path("../lib/flash.zig"),
        .target = target,
        .optimize = optimize
    });

    const rcc = b.addModule("rcc", .{
        .root_source_file = b.path("../lib/rcc.zig"),
        .target = target,
        .optimize = optimize
    });

    const cpu = b.addModule("cpu", .{
        .root_source_file = b.path("../lib/cpu.zig"),
        .target = target,
        .optimize = optimize,
        .imports = &.{
            .{ .name = "flash", .module = flash },
            .{ .name = "rcc", .module = rcc },
        },
    });

    const gpio = b.addModule("gpio", .{
        .root_source_file = b.path("../lib/gpio.zig"),
        .target = target,
        .optimize = optimize
    });

    const afio = b.addModule("afio", .{
        .root_source_file = b.path("../lib/afio.zig"),
        .target = target,
        .optimize = optimize
    });

    const pfic = b.addModule("pfic", .{
        .root_source_file = b.path("../lib/pfic.zig"),
        .target = target,
        .optimize = optimize
    });

    const system_timer = b.addModule("system_timer", .{
        .root_source_file = b.path("../lib/system_timer.zig"),
        .target = target,
        .optimize = optimize,
        .imports = &.{
            .{ .name = "cpu", .module = cpu },
            .{ .name = "pfic", .module = pfic }
        },
    });

    const riscv_exe = b.addExecutable(.{
        .name = "ch32v305_blink.elf",
        .root_module = b.createModule(.{
            .root_source_file = b.path("src/main.zig"),
            .target = target,
            .optimize = optimize,
            .imports = &.{
                .{ .name = "rcc", .module = rcc },
                .{ .name = "gpio", .module = gpio },
                .{ .name = "afio", .module = afio },
                .{ .name = "system_timer", .module = system_timer },
            },
        }),
    });

    riscv_exe.link_gc_sections = true;
    riscv_exe.link_function_sections = true;
    riscv_exe.link_data_sections = true;
    riscv_exe.lto = .full;                     // Whole-program optimization & inlining

    riscv_exe.root_module.addAssemblyFile(b.path("../startup_ch32v30x_D8C.S"));

    riscv_exe.setLinkerScript(b.path("../Link.ld"));

    b.installArtifact(riscv_exe);

    const riscv_size_report = b.addSystemCommand(&.{ "llvm-size-22" });
    riscv_size_report.addArtifactArg(riscv_exe);

    b.getInstallStep().dependOn(&riscv_size_report.step);
}

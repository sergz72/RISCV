const std = @import("std");

pub fn build(b: *std.Build) void {
    var features_add = std.Target.Cpu.Feature.Set.empty;
    const features = std.Target.riscv.Feature;

    features_add.addFeature(@intFromEnum(features.m));     // Multiply/Divide
    features_add.addFeature(@intFromEnum(features.a));     // Atomics
    features_add.addFeature(@intFromEnum(features.c));     // Compressed Instructions
    features_add.addFeature(@intFromEnum(features.zba));   // Address generation
    features_add.addFeature(@intFromEnum(features.zbb));   // Basic bit manipulation
    features_add.addFeature(@intFromEnum(features.zbs));   // Single-bit manipulation
    features_add.addFeature(@intFromEnum(features.zbkb));
    features_add.addFeature(@intFromEnum(features.zcb));
    features_add.addFeature(@intFromEnum(features.zcmp));
    features_add.addFeature(@intFromEnum(features.zicsr));
    features_add.addFeature(@intFromEnum(features.relax));

    var features_sub = std.Target.Cpu.Feature.Set.empty;
    features_sub.addFeature(@intFromEnum(features.f));
    features_sub.addFeature(@intFromEnum(features.d));
    features_sub.addFeature(@intFromEnum(features.zcf));

    const target = b.resolveTargetQuery(.{
        .cpu_arch = .riscv32,
        .os_tag = .freestanding,
        .abi = .ilp32,
        .cpu_features_add = features_add,
        .cpu_features_sub = features_sub
    });

    const optimize = b.standardOptimizeOption(.{});

    const io_registers = b.addModule("io_registers", .{
        .root_source_file = b.path("lib/io_registers.zig"),
        .target = target,
        .optimize = optimize
    });

    const sio = b.addModule("sio", .{
        .root_source_file = b.path("lib/sio.zig"),
        .target = target,
        .optimize = optimize
    });

    const ticks = b.addModule("ticks", .{
        .root_source_file = b.path("lib/ticks.zig"),
        .target = target,
        .optimize = optimize
    });

    const interrupts = b.addModule("interrupts", .{
        .root_source_file = b.path("lib/interrupts.zig"),
        .target = target,
        .optimize = optimize
    });

    const resets = b.addModule("resets", .{
        .root_source_file = b.path("lib/resets.zig"),
        .target = target,
        .optimize = optimize,
        .imports = &.{
            .{ .name = "io_registers", .module = io_registers }
        }
    });

    // const rp2350 = b.addTranslateC(.{
    //    .root_source_file = b.path("lib/RP2350.h"),
    //    .target = target,
    //    .optimize = optimize,
    //    .link_libc = false
    // });

    // rp2350.defineCMacro("CORE_FAMILY_RISC_V", null);

    const exe = b.addExecutable(.{
        .name = "pico2_blink.elf",
        .root_module = b.createModule(.{
            .root_source_file = b.path("src/main.zig"),
            .target = target,
            .optimize = optimize,
            .imports = &.{
                .{ .name = "sio", .module = sio },
                .{ .name = "ticks", .module = ticks },
                .{ .name = "interrupts", .module = interrupts },
                .{ .name = "resets", .module = resets },
            },
        }),
    });

    exe.link_gc_sections = true;
    exe.link_function_sections = true;
    exe.link_data_sections = true;
    exe.lto = .full;                     // Whole-program optimization & inlining

    // exe.root_module.addCSourceFile(.{
    //     .file = b.path("lib/image_definition_block.c"),
    //     // Pass any required compiler flags, or empty slice if none
    //     .flags = &.{ "-Wall", "-std=c23" },
    // });

    // exe.root_module.addCSourceFile(.{
    //     .file = b.path("lib/Startup/Core/RISC-V/image_definition_block.c"),
    //     // Pass any required compiler flags, or empty slice if none
    //     .flags = &.{ "-Wall", "-std=c23" },
    // });

    // exe.root_module.addCSourceFile(.{
    //     .file = b.path("lib/Startup/Startup.c"),
    //     // Pass any required compiler flags, or empty slice if none
    //     .flags = &.{ "-Wall", "-std=c23" },
    // });

    exe.root_module.addAssemblyFile(b.path("lib/start.s"));
    exe.root_module.addAssemblyFile(b.path("lib/interrupts.s"));

    exe.setLinkerScript(b.path("lib/Memory_Map.ld"));

    // Prevent Zig from filling in standard OS/libc entry points
    //exe.linker_allow_shlib_undefined = true;

    exe.entry = .{ .symbol_name = "_entry_point" };

    b.installArtifact(exe);

    const size_report = b.addSystemCommand(&.{ "llvm-size-22" });
    size_report.addArtifactArg(exe);

    // 3. Make the main install step depend on your size report
    b.getInstallStep().dependOn(&size_report.step);
}

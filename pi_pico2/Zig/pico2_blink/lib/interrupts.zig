const NUM_IRQS = 50;

const IRQHandler = *const fn () void;

var irq_handlers = [NUM_IRQS]IRQHandler{
    DefaultIRQHandler,
    DefaultIRQHandler,
    DefaultIRQHandler,
    DefaultIRQHandler,
    DefaultIRQHandler,
    DefaultIRQHandler,
    DefaultIRQHandler,
    DefaultIRQHandler,
    DefaultIRQHandler,
    DefaultIRQHandler,
    DefaultIRQHandler,
    DefaultIRQHandler,
    DefaultIRQHandler,
    DefaultIRQHandler,
    DefaultIRQHandler,
    DefaultIRQHandler,
    DefaultIRQHandler,
    DefaultIRQHandler,
    DefaultIRQHandler,
    DefaultIRQHandler,
    DefaultIRQHandler,
    DefaultIRQHandler,
    DefaultIRQHandler,
    DefaultIRQHandler,
    DefaultIRQHandler,
    DefaultIRQHandler,
    DefaultIRQHandler,
    DefaultIRQHandler,
    DefaultIRQHandler,
    DefaultIRQHandler,
    DefaultIRQHandler,
    DefaultIRQHandler,
    DefaultIRQHandler,
    DefaultIRQHandler,
    DefaultIRQHandler,
    DefaultIRQHandler,
    DefaultIRQHandler,
    DefaultIRQHandler,
    DefaultIRQHandler,
    DefaultIRQHandler,
    DefaultIRQHandler,
    DefaultIRQHandler,
    DefaultIRQHandler,
    DefaultIRQHandler,
    DefaultIRQHandler,
    DefaultIRQHandler,
    DefaultIRQHandler,
    DefaultIRQHandler,
    DefaultIRQHandler,
    DefaultIRQHandler,
};

fn DefaultIRQHandler() void {
    while (true) {
        asm volatile ("wfi");
    }
}

export fn MachineSoftwareInterruptHandler() void {

}

export fn MachineTimerInterruptHandler() void {

}

export fn MachineExternalInterruptHandler() void {
    var value: u32 = undefined;

    // read meinext
    asm volatile ("csrrsi %[value], 0xbe4, 1"
        : [value] "=r" (value),
    );
    while ((value & 0x80000000) == 0) {
        const irq_no = value >> 2;
        irq_handlers[irq_no]();
        // read meinext
        asm volatile ("csrrsi %[value], 0xbe4, 1"
            : [value] "=r" (value),
        );
    }
}

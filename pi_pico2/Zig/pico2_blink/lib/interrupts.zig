const NUM_IRQS = 52;

pub const IrqNum = enum(u32) {
    timer0_irq_0 = 0,
    timer0_irq_1 = 1,
    timer0_irq_2 = 2,
    timer0_irq_3 = 3,
    timer1_irq_0 = 4,
    timer1_irq_1 = 5,
    timer1_irq_2 = 6,
    timer1_irq_3 = 7,
    pwm_irq_wrap_0 = 8,
    pwm_irq_wrap_1 = 9,
    dma_irq_0 = 10,
    dma_irq_1 = 11,
    dma_irq_2 = 12,
    dma_irq_3 = 13,
    usbctrl_irq = 14,
    pio0_irq_0 = 15,
    pio0_irq_1 = 16,
    pio1_irq_0 = 17,
    pio1_irq_1 = 18,
    pio2_irq_0 = 19,
    pio2_irq_1 = 20,
    io_irq_bank0 = 21,
    io_irq_bank0_ns = 22,
    io_irq_qspi = 23,
    io_irq_qspi_ns = 24,
    sio_irq_fifo = 25,
    sio_irq_bell = 26,
    sio_irq_fifo_ns = 27,
    sio_irq_bell_ns = 28,
    sio_irq_mtimecmp = 29,
    clocks_irq = 30,
    spi0_irq = 31,
    spi1_irq = 32,
    uart0_irq = 33,
    uart1_irq = 34,
    adc_irq_fifo = 35,
    i2c0_irq = 36,
    i2c1_irq = 37,
    otp_irq = 38,
    trng_irq = 39,
    proc0_irq_cti = 40,
    proc1_irq_cti = 41,
    pll_sys_irq = 42,
    pll_usb_irq = 43,
    powman_irq_pow = 44,
    powman_irq_timer = 45,
    spare_irq_0 = 46,
    spare_irq_1 = 47,
    spare_irq_2 = 48,
    spare_irq_3 = 49,
    spare_irq_4 = 50,
    spare_irq_5 = 51
};

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

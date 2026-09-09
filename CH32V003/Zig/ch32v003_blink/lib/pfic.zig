const PFIC_BASE: usize = 0xE000E000;

pub const Interrupt = enum(u32) {
    NonMaskableInt = 2, // 2 Non Maskable Interrupt                             
    EXC = 3,            // 3 Exception Interrupt                                
    SysTick = 12,       // 12 System timer Interrupt                            
    Software = 14,      // 14 software Interrupt                                

    WWDG = 16,          // Window WatchDog Interrupt                            
    PVD = 17,           // PVD through EXTI Line detection Interrupt            
    FLASH = 18,         // FLASH global Interrupt                               
    RCC = 19,           // RCC global Interrupt                                 
    EXTI7_0 = 20,       // External Line[7:0] Interrupts                        
    AWU = 21,           // AWU global Interrupt                                 
    DMA1_Channel1 = 22, // DMA1 Channel 1 global Interrupt                      
    DMA1_Channel2 = 23, // DMA1 Channel 2 global Interrupt                      
    DMA1_Channel3 = 24, // DMA1 Channel 3 global Interrupt                      
    DMA1_Channel4 = 25, // DMA1 Channel 4 global Interrupt                      
    DMA1_Channel5 = 26, // DMA1 Channel 5 global Interrupt                      
    DMA1_Channel6 = 27, // DMA1 Channel 6 global Interrupt                      
    DMA1_Channel7 = 28, // DMA1 Channel 7 global Interrupt                      
    ADC = 29,           // ADC global Interrupt                                 
    I2C1_EV = 30,       // I2C1 Event Interrupt                                 
    I2C1_ER = 31,       // I2C1 Error Interrupt                                 
    USART1 = 32,        // USART1 global Interrupt                              
    SPI1 = 33,          // SPI1 global Interrupt                                
    TIM1_BRK = 34,      // TIM1 Break Interrupt                                 
    TIM1_UP = 35,       // TIM1 Update Interrupt                                
    TIM1_TRG_COM = 36,  // TIM1 Trigger and Commutation Interrupt               
    TIM1_CC = 37,       // TIM1 Capture Compare Interrupt                       
    TIM2 = 38           // TIM2 global Interrupt                                
};

pub const Pfic = extern struct {
    isr: [8]u32,
    ipr: [8]u32,
    ithresdr: u32,
    reserved: u32,
    cfgr: u32,
    gisr: u32,
    vtfidr: u32,
    reserved9: [3]u32,
    vtfaddrr: [4]u32,
    reserved2: [36]u32,
    ienr: [8]u32,
    reserved3: [24]u32,
    irer: [8]u32,
    reserved4: [24]u32,
    ipsr: [8]u32,
    reserved5: [24]u32,
    iprr: [8]u32,
    reserved6: [24]u32,
    iactr: [8]u32,
    reserved7: [56]u32,
    iprior: [16]u32,
    reserved8: [564]u32,
    sctlr: u32,

    pub fn interrupt_enable(self: *volatile Pfic, interrupt: Interrupt) void {
        const v = @intFromEnum(interrupt);
        const shift: u5 = @truncate(v);
        self.ienr[v >> 5] = @as(u32, 1) << shift;
    }

    pub fn interrupt_disable(self: *volatile Pfic, interrupt: Interrupt) void {
        const v = @intFromEnum(interrupt);
        const shift: u5 = @truncate(v);
        self.irer[v >> 5] = @as(u32, 1) << shift;
    }
};

pub const pfic: *volatile Pfic = @ptrFromInt(PFIC_BASE);

test "sizeof test" {
    const std = @import("std");
    try std.testing.expectEqual(0xD14, @sizeOf(Pfic));
}

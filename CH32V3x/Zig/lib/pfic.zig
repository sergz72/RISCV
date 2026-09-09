const PFIC_BASE: usize = 0xE000E000;

pub const Interrupt = enum(u32) {
    NonMaskableInt         = 2,       // 2 Non Maskable Interrupt                             
    EXC                    = 3,       // 3 Exception Interrupt                                
    Ecall_M_Mode           = 5,       // 5 Ecall M Mode Interrupt                             
    Ecall_U_Mode           = 8,       // 8 Ecall U Mode Interrupt                             
    Break_Point            = 9,       // 9 Break Point Interrupt                              
    SysTick                = 12,      // 12 System timer Interrupt                            
    Software               = 14,      // 14 software Interrupt                                

    WWDG                   = 16,      // Window WatchDog Interrupt
    PVD                    = 17,      // PVD through EXTI Line detection Interrupt            
    TAMPER                 = 18,      // Tamper Interrupt                                     
    RTC                    = 19,      // RTC global Interrupt                                 
    FLASH                  = 20,      // FLASH global Interrupt                               
    RCC                    = 21,      // RCC global Interrupt                                 
    EXTI0                  = 22,      // EXTI Line0 Interrupt                                 
    EXTI1                  = 23,      // EXTI Line1 Interrupt                                 
    EXTI2                  = 24,      // EXTI Line2 Interrupt                                 
    EXTI3                  = 25,      // EXTI Line3 Interrupt                                 
    EXTI4                  = 26,      // EXTI Line4 Interrupt                                 
    DMA1_Channel1          = 27,      // DMA1 Channel 1 global Interrupt                      
    DMA1_Channel2          = 28,      // DMA1 Channel 2 global Interrupt                      
    DMA1_Channel3          = 29,      // DMA1 Channel 3 global Interrupt                      
    DMA1_Channel4          = 30,      // DMA1 Channel 4 global Interrupt                      
    DMA1_Channel5          = 31,      // DMA1 Channel 5 global Interrupt                      
    DMA1_Channel6          = 32,      // DMA1 Channel 6 global Interrupt                      
    DMA1_Channel7          = 33,      // DMA1 Channel 7 global Interrupt                      
    ADC                    = 34,      // ADC1 and ADC2 global Interrupt                       
    USB_HP_CAN1_TX         = 35,      // USB Device High Priority or CAN1 TX Interrupts       
    USB_LP_CAN1_RX0        = 36,      // USB Device Low Priority or CAN1 RX0 Interrupts       
    CAN1_RX1               = 37,      // CAN1 RX1 Interrupt                                   
    CAN1_SCE               = 38,      // CAN1 SCE Interrupt                                   
    EXTI9_5                = 39,      // External Line[9:5] Interrupts                        
    TIM1_BRK               = 40,      // TIM1 Break Interrupt                                 
    TIM1_UP                = 41,      // TIM1 Update Interrupt                                
    TIM1_TRG_COM           = 42,      // TIM1 Trigger and Commutation Interrupt               
    TIM1_CC                = 43,      // TIM1 Capture Compare Interrupt                       
    TIM2                   = 44,      // TIM2 global Interrupt                                
    TIM3                   = 45,      // TIM3 global Interrupt                                
    TIM4                   = 46,      // TIM4 global Interrupt                                
    I2C1_EV                = 47,      // I2C1 Event Interrupt                                 
    I2C1_ER                = 48,      // I2C1 Error Interrupt                                 
    I2C2_EV                = 49,      // I2C2 Event Interrupt                                 
    I2C2_ER                = 50,      // I2C2 Error Interrupt                                 
    SPI1                   = 51,      // SPI1 global Interrupt                                
    SPI2                   = 52,      // SPI2 global Interrupt                                
    USART1                 = 53,      // USART1 global Interrupt                              
    USART2                 = 54,      // USART2 global Interrupt                              
    USART3                 = 55,      // USART3 global Interrupt                              
    EXTI15_10              = 56,      // External Line[15:10] Interrupts                      
    RTCAlarm               = 57,      // RTC Alarm through EXTI Line Interrupt                

    USBWakeUp              = 58,      // USB Device WakeUp from suspend through EXTI Line Interrupt

    TIM8_BRK               = 59,      // TIM8 Break Interrupt
    TIM8_UP                = 60,      // TIM8 Update Interrupt                                
    TIM8_TRG_COM           = 61,      // TIM8 Trigger and Commutation Interrupt               
    TIM8_CC                = 62,      // TIM8 Capture Compare Interrupt                       
    RNG                    = 63,      // RNG global Interrupt                                 
    SDIO                   = 65,      // SDIO global Interrupt                                
    TIM5                   = 66,      // TIM5 global Interrupt                                
    SPI3                   = 67,      // SPI3 global Interrupt                                
    UART4                  = 68,      // UART4 global Interrupt                               
    UART5                  = 69,      // UART5 global Interrupt                               
    TIM6                   = 70,      // TIM6 global Interrupt                                
    TIM7                   = 71,      // TIM7 global Interrupt                                
    DMA2_Channel1          = 72,      // DMA2 Channel 1 global Interrupt                      
    DMA2_Channel2          = 73,      // DMA2 Channel 2 global Interrupt                      
    DMA2_Channel3          = 74,      // DMA2 Channel 3 global Interrupt                      
    DMA2_Channel4          = 75,      // DMA2 Channel 4 global Interrupt                      
    DMA2_Channel5          = 76,      // DMA2 Channel 5 global Interrupt                      
    ETH                    = 77,      // ETH global Interrupt
    ETH_WKUP               = 78,      // ETH WakeUp Interrupt
    CAN2_TX                = 79,      // CAN2 TX Interrupts
    CAN2_RX0               = 80,      // CAN2 RX0 Interrupts
    CAN2_RX1               = 81,      // CAN2 RX1 Interrupt
    CAN2_SCE               = 82,      // CAN2 SCE Interrupt
    USBFS                  = 83,      // USBFS global Interrupt
    USBHSWakeup            = 84,      // USBHS WakeUp Interrupt
    USBHS                  = 85,      // USBHS global Interrupt
    DVP                    = 86,      // DVP global Interrupt
    UART6                  = 87,      // UART6 global Interrupt
    UART7                  = 88,      // UART7 global Interrupt                               
    UART8                  = 89,      // UART8 global Interrupt                               
    TIM9_BRK               = 90,      // TIM9 Break Interrupt                                 
    TIM9_UP                = 91,      // TIM9 Update Interrupt                                
    TIM9_TRG_COM           = 92,      // TIM9 Trigger and Commutation Interrupt               
    TIM9_CC                = 93,      // TIM9 Capture Compare Interrupt                       
    TIM10_BRK              = 94,      // TIM10 Break Interrupt                                
    TIM10_UP               = 95,      // TIM10 Update Interrupt                               
    TIM10_TRG_COM          = 96,      // TIM10 Trigger and Commutation Interrupt              
    TIM10_CC               = 97,      // TIM10 Capture Compare Interrupt                      
    DMA2_Channel6          = 98,      // DMA2 Channel 6 global Interrupt                      
    DMA2_Channel7          = 99,      // DMA2 Channel 7 global Interrupt                      
    DMA2_Channel8          = 100,     // DMA2 Channel 8 global Interrupt                      
    DMA2_Channel9          = 101,     // DMA2 Channel 9 global Interrupt                      
    DMA2_Channel10         = 102,     // DMA2 Channel 10 global Interrupt                     
    DMA2_Channel11         = 103      // DMA2 Channel 11 global Interrupt
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

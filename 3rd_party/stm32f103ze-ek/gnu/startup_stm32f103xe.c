/* File: startup_stm32f103xe.c
 * Purpose: startup file for stm32f103xe Cortex-M3 device.
 *          Should be used with GCC 'GNU Tools ARM Embedded'
 * Version: CMSIS 5.0.1
 * Date:    2017-09-13
 *
 * Created from the CMSIS template for the specified device
 * Quantum Leaps, www.state-machine.com
 *
 * NOTE:
 * The function assert_failed defined at the end of this file defines
 * the error/assertion handling policy for the application and might
 * need to be customized for each project. This function is defined in
 * assembly to re-set the stack pointer, in case it is corrupted by the
 * time assert_failed is called.
 */
/* Copyright (c) 2011 - 2014 ARM LIMITED

   All rights reserved.
   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
   - Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   - Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
   - Neither the name of ARM nor the names of its contributors may be used
     to endorse or promote products derived from this software without
     specific prior written permission.
   *
   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
 ---------------------------------------------------------------------------*/

/* start and end of stack defined in the linker script ---------------------*/
/*extern int __stack_start__;*/
extern int __stack_end__;

/* Weak prototypes for error handlers --------------------------------------*/
/**
* \note
* The function assert_failed defined at the end of this file defines
* the error/assertion handling policy for the application and might
* need to be customized for each project. This function is defined in
* assembly to avoid accessing the stack, which might be corrupted by
* the time assert_failed is called.
*/
__attribute__ ((naked, noreturn))
void assert_failed(char const *module, int loc);

/* Function prototypes -----------------------------------------------------*/
void Default_Handler(void);  /* Default empty handler */
void Reset_Handler(void);    /* Reset Handler */
void SystemInit(void);       /* CMSIS system initialization */
__attribute__ ((noreturn))
void Q_onAssert(char const *module, int loc); /* QP assertion handler */

/*----------------------------------------------------------------------------
* weak aliases for each Exception handler to the Default_Handler.
* Any function with the same name will override these definitions.
*/
/* Cortex-M Processor fault exceptions... */
void NMI_Handler             (void) __attribute__ ((weak));
void HardFault_Handler       (void) __attribute__ ((weak));
void MemManage_Handler       (void) __attribute__ ((weak));
void BusFault_Handler        (void) __attribute__ ((weak));
void UsageFault_Handler      (void) __attribute__ ((weak));

/* Cortex-M Processor non-fault exceptions... */
void SVC_Handler             (void) __attribute__ ((weak, alias("Default_Handler")));
void DebugMon_Handler        (void) __attribute__ ((weak, alias("Default_Handler")));
void PendSV_Handler          (void) __attribute__ ((weak, alias("Default_Handler")));
void SysTick_Handler         (void) __attribute__ ((weak, alias("Default_Handler")));

/* external interrupts...   */
void WWDG_IRQHandler         (void) __attribute__ ((weak, alias("Default_Handler")));
void PVD_IRQHandler          (void) __attribute__ ((weak, alias("Default_Handler")));
void TAMPER_IRQHandler       (void) __attribute__ ((weak, alias("Default_Handler")));
void RTC_IRQHandler          (void) __attribute__ ((weak, alias("Default_Handler")));
void FLASH_IRQHandler        (void) __attribute__ ((weak, alias("Default_Handler")));
void RCC_IRQHandler          (void) __attribute__ ((weak, alias("Default_Handler")));
void EXTI0_IRQHandler        (void) __attribute__ ((weak, alias("Default_Handler")));
void EXTI1_IRQHandler        (void) __attribute__ ((weak, alias("Default_Handler")));
void EXTI2_IRQHandler        (void) __attribute__ ((weak, alias("Default_Handler")));
void EXTI3_IRQHandler        (void) __attribute__ ((weak, alias("Default_Handler")));
void EXTI4_IRQHandler        (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Channel1_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Channel2_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Channel3_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Channel4_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Channel5_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Channel6_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA1_Channel7_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void ADC1_2_IRQHandler       (void) __attribute__ ((weak, alias("Default_Handler")));
void USB_HP_CAN1_TX_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void USB_LP_CAN1_RX0_IRQHandler(void) __attribute__ ((weak, alias("Default_Handler")));
void CAN1_RX1_IRQHandler     (void) __attribute__ ((weak, alias("Default_Handler")));
void CAN1_SCE_IRQHandler     (void) __attribute__ ((weak, alias("Default_Handler")));
void EXTI9_5_IRQHandler      (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM1_BRK_IRQHandler     (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM1_UP_IRQHandler      (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM1_TRG_COM_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM1_CC_IRQHandler      (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM2_IRQHandler         (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM3_IRQHandler         (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM4_IRQHandler         (void) __attribute__ ((weak, alias("Default_Handler")));
void I2C1_EV_IRQHandler      (void) __attribute__ ((weak, alias("Default_Handler")));
void I2C1_ER_IRQHandler      (void) __attribute__ ((weak, alias("Default_Handler")));
void I2C2_EV_IRQHandler      (void) __attribute__ ((weak, alias("Default_Handler")));
void I2C2_ER_IRQHandler      (void) __attribute__ ((weak, alias("Default_Handler")));
void SPI1_IRQHandler         (void) __attribute__ ((weak, alias("Default_Handler")));
void SPI2_IRQHandler         (void) __attribute__ ((weak, alias("Default_Handler")));
void USART1_IRQHandler       (void) __attribute__ ((weak, alias("Default_Handler")));
void USART2_IRQHandler       (void) __attribute__ ((weak, alias("Default_Handler")));
void USART3_IRQHandler       (void) __attribute__ ((weak, alias("Default_Handler")));
void EXTI15_10_IRQHandler    (void) __attribute__ ((weak, alias("Default_Handler")));
void RTC_Alarm_IRQHandler    (void) __attribute__ ((weak, alias("Default_Handler")));
void USBWakeUp_IRQHandler    (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM8_BRK_IRQHandler     (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM8_UP_IRQHandler      (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM8_TRG_COM_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM8_CC_IRQHandler      (void) __attribute__ ((weak, alias("Default_Handler")));
void ADC3_IRQHandler         (void) __attribute__ ((weak, alias("Default_Handler")));
void FSMC_IRQHandler         (void) __attribute__ ((weak, alias("Default_Handler")));
void SDIO_IRQHandler         (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM5_IRQHandler         (void) __attribute__ ((weak, alias("Default_Handler")));
void SPI3_IRQHandler         (void) __attribute__ ((weak, alias("Default_Handler")));
void UART4_IRQHandler        (void) __attribute__ ((weak, alias("Default_Handler")));
void UART5_IRQHandler        (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM6_IRQHandler         (void) __attribute__ ((weak, alias("Default_Handler")));
void TIM7_IRQHandler         (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_Channel1_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_Channel2_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_Channel3_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));
void DMA2_Channel4_5_IRQHandler (void) __attribute__ ((weak, alias("Default_Handler")));

/*..........................................................................*/
__attribute__ ((section(".isr_vector")))
int const g_pfnVectors[] = {
    (int)&__stack_end__,            /* Top of Stack                    */
    (int)&Reset_Handler,            /* Reset Handler                   */
    (int)&NMI_Handler,              /* NMI Handler                     */
    (int)&HardFault_Handler,        /* Hard Fault Handler              */
    (int)&MemManage_Handler,        /* The MPU fault handler           */
    (int)&BusFault_Handler,         /* The bus fault handler           */
    (int)&UsageFault_Handler,       /* The usage fault handler         */
    0,                              /* Reserved                        */
    0,                              /* Reserved                        */
    0,                              /* Reserved                        */
    0,                              /* Reserved                        */
    (int)&SVC_Handler,              /* SVCall handler                  */
    (int)&DebugMon_Handler,         /* Debug monitor handler           */
    0,                              /* Reserved                        */
    (int)&PendSV_Handler,           /* The PendSV handler              */
    (int)&SysTick_Handler,          /* The SysTick handler             */

    /*IRQ handlers... */
    (int)&WWDG_IRQHandler,             /* Window WatchDog                  */
    (int)&PVD_IRQHandler,              /* PVD through EXTI Line detection  */
    (int)&TAMPER_IRQHandler,           /* Tamper and TimeStamps through the EXTI line EXTI line */
    (int)&RTC_IRQHandler,              /* RTC Wakeup through the EXTI line */
    (int)&FLASH_IRQHandler,            /* FLASH                            */
    (int)&RCC_IRQHandler,              /* RCC                              */
    (int)&EXTI0_IRQHandler,            /* EXTI Line0                       */
    (int)&EXTI1_IRQHandler,            /* EXTI Line1                       */
    (int)&EXTI2_IRQHandler,            /* EXTI Line2                       */
    (int)&EXTI3_IRQHandler,            /* EXTI Line3                       */
    (int)&EXTI4_IRQHandler,            /* EXTI Line4                       */
    (int)&DMA1_Channel1_IRQHandler,    /* DMA1 Channel 1                   */
    (int)&DMA1_Channel2_IRQHandler,    /* DMA1 Channel 2                   */
    (int)&DMA1_Channel3_IRQHandler,    /* DMA1 Channel 3                   */
    (int)&DMA1_Channel4_IRQHandler,    /* DMA1 Channel 4                   */
    (int)&DMA1_Channel5_IRQHandler,    /* DMA1 Channel 5                   */
    (int)&DMA1_Channel6_IRQHandler,    /* DMA1 Channel 6                   */
    (int)&DMA1_Channel7_IRQHandler,    /* DMA1 Channel 7                   */
    (int)&ADC1_2_IRQHandler,           /* ADC1, ADC2 and                   */
    (int)&USB_HP_CAN1_TX_IRQHandler,   /* CAN1 TX                          */
    (int)&USB_LP_CAN1_RX0_IRQHandler,  /* CAN1 RX0                         */
    (int)&CAN1_RX1_IRQHandler,         /* CAN1 RX1                         */
    (int)&CAN1_SCE_IRQHandler,         /* CAN1 SCE                         */
    (int)&EXTI9_5_IRQHandler,          /* External Line[9:5]               */
    (int)&TIM1_BRK_IRQHandler,         /* TIM1 Break                       */
    (int)&TIM1_UP_IRQHandler,          /* TIM1 Update                      */
    (int)&TIM1_TRG_COM_IRQHandler,     /* TIM1 Trigger and Commutation and TIM11 and TIM11 */
    (int)&TIM1_CC_IRQHandler,          /* TIM1 Capture Compare             */
    (int)&TIM2_IRQHandler,             /* TIM2                             */
    (int)&TIM3_IRQHandler,             /* TIM3                             */
    (int)&TIM4_IRQHandler,             /* TIM4                             */
    (int)&I2C1_EV_IRQHandler,          /* I2C1 Event                       */
    (int)&I2C1_ER_IRQHandler,          /* I2C1 Error                       */
    (int)&I2C2_EV_IRQHandler,          /* I2C2 Event                       */
    (int)&I2C2_ER_IRQHandler,          /* I2C2 Error                       */
    (int)&SPI1_IRQHandler,             /* SPI1                             */
    (int)&SPI2_IRQHandler,             /* SPI2                             */
    (int)&USART1_IRQHandler,           /* USART1                           */
    (int)&USART2_IRQHandler,           /* USART2                           */
    (int)&USART3_IRQHandler,           /* USART3                           */
    (int)&EXTI15_10_IRQHandler,        /* External Line[15:10]s*/
    (int)&RTC_Alarm_IRQHandler,        /* RTC Alarm (A and B) through EXTI LineLine */
    (int)&USBWakeUp_IRQHandler,        /* USB OTG FS Wakeup through EXTI linene   */
    (int)&TIM8_BRK_IRQHandler,         /* TIM8 Break                       */
    (int)&TIM8_UP_IRQHandler,          /* TIM8 Update                      */
    (int)&TIM8_TRG_COM_IRQHandler,     /* TIM8 Trigger and Commutation and TIM14 and TIM14 */
    (int)&TIM8_CC_IRQHandler,          /* TIM8 Capture Compare             */
    (int)&ADC3_IRQHandler,             /* ADC3                             */
    (int)&FSMC_IRQHandler,             /* FSMC                             */
    (int)&SDIO_IRQHandler,             /* SDIO                             */
    (int)&TIM5_IRQHandler,             /* TIM5                             */
    (int)&SPI3_IRQHandler,             /* SPI3                             */
    (int)&UART4_IRQHandler,            /* UART4                            */
    (int)&UART5_IRQHandler,            /* UART5                            */
    (int)&TIM6_IRQHandler,             /* TIM6                             */
    (int)&TIM7_IRQHandler,             /* TIM7                             */
    (int)&DMA2_Channel1_IRQHandler,    /* DMA2 Channel 1                   */
    (int)&DMA2_Channel2_IRQHandler,    /* DMA2 Channel 2                   */
    (int)&DMA2_Channel3_IRQHandler,    /* DMA2 Channel 3                   */
    (int)&DMA2_Channel4_5_IRQHandler   /* DMA2 Channel 4 and 5             */
};


/* reset handler -----------------------------------------------------------*/
__attribute__((naked)) void Reset_Handler(void);
void Reset_Handler(void) {
    extern int main(void);
    extern int __libc_init_array(void);
    extern unsigned __data_start;  /* start of .data in the linker script */
    extern unsigned __data_end__;  /* end of .data in the linker script */
    extern unsigned const __data_load; /* initialization values for .data  */
    extern unsigned __bss_start__; /* start of .bss in the linker script */
    extern unsigned __bss_end__;   /* end of .bss in the linker script */
    extern void software_init_hook(void) __attribute__((weak));

    unsigned const *src;
    unsigned *dst;

    SystemInit(); /* CMSIS system initialization */

    /* copy the data segment initializers from flash to RAM... */
    src = &__data_load;
    for (dst = &__data_start; dst < &__data_end__; ++dst, ++src) {
        *dst = *src;
    }

    /* zero fill the .bss segment in RAM... */
    for (dst = &__bss_start__; dst < &__bss_end__; ++dst) {
        *dst = 0;
    }

    /* init hook provided? */
    if (&software_init_hook != (void (*)(void))(0)) {
        /* give control to the RTOS */
        software_init_hook(); /* this will also call __libc_init_array */
    }
    else {
        /* call all static constructors in C++ (comment out in C programs) */
        //__libc_init_array();
        (void)main(); /* application's entry point; should never return! */
    }

    /* the previous code should not return, but assert just in case... */
    assert_failed("Reset_Handler", __LINE__);
}


/* fault exception handlers ------------------------------------------------*/
__attribute__((naked)) void NMI_Handler(void);
void NMI_Handler(void) {
    __asm volatile (
        "    ldr r0,=str_nmi\n\t"
        "    mov r1,#1\n\t"
        "    b assert_failed\n\t"
        "str_nmi: .asciz \"NMI\"\n\t"
        "  .align 2\n\t"
    );
}
/*..........................................................................*/
__attribute__((naked)) void MemManage_Handler(void);
void MemManage_Handler(void) {
    __asm volatile (
        "    ldr r0,=str_mem\n\t"
        "    mov r1,#1\n\t"
        "    b assert_failed\n\t"
        "str_mem: .asciz \"MemManage\"\n\t"
        "  .align 2\n\t"
    );
}
/*..........................................................................*/
__attribute__((naked)) void HardFault_Handler(void);
void HardFault_Handler(void) {
    __asm volatile (
        "    ldr r0,=str_hrd\n\t"
        "    mov r1,#1\n\t"
        "    b assert_failed\n\t"
        "str_hrd: .asciz \"HardFault\"\n\t"
        "  .align 2\n\t"
    );
}
/*..........................................................................*/
__attribute__((naked)) void BusFault_Handler(void);
void BusFault_Handler(void) {
    __asm volatile (
        "    ldr r0,=str_bus\n\t"
        "    mov r1,#1\n\t"
        "    b assert_failed\n\t"
        "str_bus: .asciz \"BusFault\"\n\t"
        "  .align 2\n\t"
    );
}
/*..........................................................................*/
__attribute__((naked)) void UsageFault_Handler(void);
void UsageFault_Handler(void) {
    __asm volatile (
        "    ldr r0,=str_usage\n\t"
        "    mov r1,#1\n\t"
        "    b assert_failed\n\t"
        "str_usage: .asciz \"UsageFault\"\n\t"
        "  .align 2\n\t"
    );
}
/*..........................................................................*/
__attribute__((naked)) void Default_Handler(void);
void Default_Handler(void) {
    __asm volatile (
        "    ldr r0,=str_dflt\n\t"
        "    mov r1,#1\n\t"
        "    b assert_failed\n\t"
        "str_dflt: .asciz \"Default\"\n\t"
        "  .align 2\n\t"
    );
}


/*****************************************************************************
* The function assert_failed defines the error/assertion handling policy
* for the application. After making sure that the stack is OK, this function
* calls Q_onAssert, which should NOT return (typically reset the CPU).
*
* NOTE: the function Q_onAssert should NOT return.
*****************************************************************************/
__attribute__ ((naked, noreturn))
void assert_failed(char const *module, int loc) {
    /* re-set the SP in case of stack overflow */
    __asm volatile (
        "  MOV sp,%0\n\t"
        : : "r" (&__stack_end__));

    Q_onAssert(module, loc); /* call the application-specific QP handler */

    for (;;) { /* should not be reached, but just in case loop forever... */
    }
}

/****** End Of File *********************************************************/

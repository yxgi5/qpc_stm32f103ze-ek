/*****************************************************************************
* Product: DPP example, NUCLEO-L053R8 board, uC/OS-II RTOS
* Last updated for version 6.9.1
* Last updated on  2020-09-22
*
*                    Q u a n t u m  L e a P s
*                    ------------------------
*                    Modern Embedded Software
*
* Copyright (C) 2005-2020 Quantum Leaps, LLC. All rights reserved.
*
* This program is open source software: you can redistribute it and/or
* modify it under the terms of the GNU General Public License as published
* by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Alternatively, this program may be distributed and modified under the
* terms of Quantum Leaps commercial licenses, which expressly supersede
* the GNU General Public License and are specifically designed for
* licensees interested in retaining the proprietary status of their code.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program. If not, see <www.gnu.org/licenses/>.
*
* Contact information:
* <www.state-machine.com/licensing>
* <info@state-machine.com>
*****************************************************************************/
#include "qpc.h"
#include "dpp.h"
#include "bsp.h"

/* STM32Cube include files */
#include "stm32f1xx_hal.h"
#include "stm32f103ze_ek.h"
/* add other drivers if necessary... */

Q_DEFINE_THIS_FILE

/* Local-scope defines -----------------------------------------------------*/
/* LED pins available on the board (just one user LED LD2--Green on PA.5) */

static uint32_t l_rnd;  /* random seed */

#ifdef Q_SPY
    QSTimeCtr QS_tickTime_;
    QSTimeCtr QS_tickPeriod_;

    /* event-source identifiers used for tracing */
    static uint8_t l_tickHook;
    static uint8_t l_GPIO_EVEN_IRQHandler = 0U;
    //static uint8_t l_EXTI0_IRQHandler = 0U;
    static UART_HandleTypeDef l_uartHandle;
	
    enum AppRecords { /* application-specific trace records */
        PHILO_STAT = QS_USER,
        ON_CONTEXT_SW
    };

#endif

/* ISRs used in the application ==========================================*/
/* uCOS-II application hooks ===============================================*/
void App_TaskCreateHook (OS_TCB *ptcb) { (void)ptcb; }
void App_TaskDelHook    (OS_TCB *ptcb) { (void)ptcb; }
/*..........................................................................*/
void App_TaskIdleHook(void) {
#if OS_CRITICAL_METHOD == 3u  /* Allocate storage for CPU status register */
    OS_CPU_SR cpu_sr;
#endif

    /* toggle LED1 on and then off, see NOTE01 */
    OS_ENTER_CRITICAL();
	BSP_LED_On(LED4);
	BSP_LED_Off(LED4);
    OS_EXIT_CRITICAL();

#ifdef Q_SPY
    if ((l_uartHandle.Instance->ISR & UART_FLAG_TXE) != 0U) {  /* is TXE empty? */
        uint16_t b;

        OS_ENTER_CRITICAL();
        b = QS_getByte();
        OS_EXIT_CRITICAL();

        if (b != QS_EOD) {  /* not End-Of-Data? */
            l_uartHandle.Instance->TDR = (b & 0xFFU);  /* put into the TDR register */
        }
    }
#elif defined NDEBUG
    /* Put the CPU and peripherals to the low-power mode.
    * you might need to customize the clock management for your application,
    * see the datasheet for your particular Cortex-M3 MCU.
    */
    __WFI(); /* Wait-For-Interrupt */
#endif
}
/*..........................................................................*/
void App_TaskReturnHook (OS_TCB *ptcb) { (void)ptcb; }
void App_TaskStatHook   (void)         {}
void App_TaskSwHook     (void)         {}
void App_TCBInitHook    (OS_TCB *ptcb) { (void)ptcb; }
/*..........................................................................*/
void App_TimeTickHook(void) {
    /* state of the button debouncing, see below */
    static struct ButtonsDebouncing {
        uint32_t depressed;
        uint32_t previous;
    } buttons = { 0U, 0U };
    uint32_t current;
    uint32_t tmp;

#ifdef Q_SPY
    {
        tmp = SysTick->CTRL; /* clear SysTick_CTRL_COUNTFLAG */
        QS_tickTime_ += QS_tickPeriod_; /* account for the clock rollover */
    }
#endif

    QF_TICK_X(0U, &l_tickHook); /* process time events for rate 0 */
    //QACTIVE_POST(the_Ticker0, 0, &l_tickHook); /* post to Ticker0 */

    /* Perform the debouncing of buttons. The algorithm for debouncing
    * adapted from the book "Embedded Systems Dictionary" by Jack Ganssle
    * and Michael Barr, page 71.
    */
    current = BSP_PB_GetState(BUTTON_KEY); /* read the Key button */
    tmp = buttons.depressed; /* save the debounced depressed buttons */
    buttons.depressed |= (buttons.previous & current); /* set depressed */
    buttons.depressed &= (buttons.previous | current); /* clear released */
    buttons.previous   = current; /* update the history */
    tmp ^= buttons.depressed;     /* changed debounced depressed */
    if (tmp != 0U) {  /* debounced Key state changed? */
        if (buttons.depressed != 0U) { /* is G8 depressed? */
            static QEvt const pauseEvt = { PAUSE_SIG, 0U, 0U};
            QF_PUBLISH(&pauseEvt, &l_tickHook);
        }
        else {            /* the button is released */
            static QEvt const serveEvt = { SERVE_SIG, 0U, 0U};
            QF_PUBLISH(&serveEvt, &l_tickHook);
        }
    }
}

/* BSP functions ===========================================================*/
void BSP_init(void) {
    /* NOTE: SystemInit() has been already called from the startup code
    *  but SystemCoreClock needs to be updated
    */
    SystemCoreClockUpdate();

    /* Configure LED */
    BSP_LED_Init(LED1);
	BSP_LED_Init(LED2);
	BSP_LED_Init(LED3);
	BSP_LED_Init(LED4);


    /* Configure the User Button in GPIO Mode */
    BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_GPIO);

    BSP_randomSeed(1234U); /* seed the random number generator */

    /* initialize the QS software tracing... */
    if (QS_INIT((void *)0) == 0U) {
        Q_ERROR();
    }
    QS_OBJ_DICTIONARY(&l_tickHook);
    QS_OBJ_DICTIONARY(&l_GPIO_EVEN_IRQHandler);
    QS_USR_DICTIONARY(PHILO_STAT);
    QS_USR_DICTIONARY(ON_CONTEXT_SW);

    /* setup the QS filters... */
    QS_GLB_FILTER(QS_SM_RECORDS);
    QS_GLB_FILTER(QS_UA_RECORDS);
}
/*..........................................................................*/
void BSP_displayPhilStat(uint8_t n, char const *stat) {
    /* exercise the FPU with some floating point computations */
    /* NOTE: this code can be only called from a task that created with
    * the option OS_TASK_OPT_SAVE_FP.
    */
    //float volatile x;
    //x = 3.1415926F;
    //x = x + 2.7182818F;
	
    if (stat[0] == 'h') {
        BSP_LED_On(LED1);  /* turn LED on  */
    }
    else {
        BSP_LED_Off(LED1);  /* turn LED off */
    }

    QS_BEGIN_ID(PHILO_STAT, AO_Philo[n]->prio) /* app-specific record */
        QS_U8(1, n);                  /* Philosopher number */
        QS_STR(stat);                 /* Philosopher status */
    QS_END()
}
/*..........................................................................*/
void BSP_displayPaused(uint8_t paused) {
    /* not enough LEDs to implement this feature */
    if (paused != 0U) {
        BSP_LED_On(LED2);  /* turn LED[n] on  */
    }
    else {
        BSP_LED_Off(LED2);  /* turn LED[n] off */
    }
}
/*..........................................................................*/
uint32_t BSP_random(void) { /* a very cheap pseudo-random-number generator */
    /* "Super-Duper" Linear Congruential Generator (LCG)
    * LCG(2^32, 3*7*11*13*23, 0, seed)
    */
    l_rnd = l_rnd * (3U*7U*11U*13U*23U);
    return l_rnd >> 8;
}
/*..........................................................................*/
void BSP_randomSeed(uint32_t seed) {
    l_rnd = seed;
}
/*..........................................................................*/
void BSP_terminate(int16_t result) {
    (void)result;
}

/* QF callbacks ============================================================*/
void QF_onStartup(void) {
    /* set up the SysTick timer to fire at BSP_TICKS_PER_SEC rate
    * NOTE: do NOT call OS_CPU_SysTickInit() from uC/OS-II
    */
    SysTick_Config(SystemCoreClock / BSP_TICKS_PER_SEC);

    /* set priorities of ALL ISRs used in the system */
    NVIC_SetPriority(SysTick_IRQn,  1U);
    /* ... */

    /* enable IRQs in the NVIC... */
    /* ... */
}
/*..........................................................................*/
void QF_onCleanup(void) {
}
/*..........................................................................*/
Q_NORETURN Q_onAssert(char_t const * const module, int_t const loc) {
    /*
    * NOTE: add here your application-specific error handling
    */
    (void)module;
    (void)loc;
    QS_ASSERTION(module, loc, 10000U); /* report assertion to QS */

#ifndef NDEBUG
    /* light all both LEDs */
    BSP_LED_On(LED1);
	BSP_LED_On(LED2);
	BSP_LED_On(LED3);
	BSP_LED_On(LED4);
    /* for debugging, hang on in an endless loop until SW1 is pressed... */
    while ((BSP_PB_GetState(BUTTON_KEY)) != 0U) {
        BSP_LED_On(LED3);        /* turn LED1 on  */
        BSP_LED_Off(LED3);       /* turn LED1 off */
    }
#endif

    NVIC_SystemReset();
}

/* QS callbacks ============================================================*/
#ifdef Q_SPY
/*..........................................................................*/
#define __DIV(__PCLK, __BAUD)       (((__PCLK / 4) *25)/(__BAUD))
#define __DIVMANT(__PCLK, __BAUD)   (__DIV(__PCLK, __BAUD)/100)
#define __DIVFRAQ(__PCLK, __BAUD)   \
    (((__DIV(__PCLK, __BAUD) - (__DIVMANT(__PCLK, __BAUD) * 100)) \
        * 16 + 50) / 100)
#define __USART_BRR(__PCLK, __BAUD) \
    ((__DIVMANT(__PCLK, __BAUD) << 4)|(__DIVFRAQ(__PCLK, __BAUD) & 0x0F))

/*..........................................................................*/
uint8_t QS_onStartup(void const *arg) {
    static uint8_t qsBuf[2*1024]; /* buffer for Quantum Spy */

    (void)arg; /* avoid the "unused parameter" compiler warning */
    QS_initBuf(qsBuf, sizeof(qsBuf));

  GPIO_InitTypeDef gpio_init_structure;

  /* Enable GPIO clock */
  COMx_TX_GPIO_CLK_ENABLE(COM);
  COMx_RX_GPIO_CLK_ENABLE(COM);

  /* Enable USART clock */
  COMx_CLK_ENABLE(COM);

  /* Configure USART Tx as alternate function */
  gpio_init_structure.Pin = COM_TX_PIN[COM];
  gpio_init_structure.Mode = GPIO_MODE_AF_PP;
  //gpio_init_structure.Speed = GPIO_SPEED_FAST;
  gpio_init_structure.Speed      = GPIO_SPEED_FREQ_HIGH;
  gpio_init_structure.Pull = GPIO_PULLUP;
  //gpio_init_structure.Alternate = COM_TX_AF[COM];
  HAL_GPIO_Init(COM_TX_PORT[COM], &gpio_init_structure);

  /* Configure USART Rx as alternate function */
  gpio_init_structure.Mode = GPIO_MODE_INPUT;
  gpio_init_structure.Pin = COM_RX_PIN[COM];
  //gpio_init_structure.Mode = GPIO_MODE_AF_PP;
  //gpio_init_structure.Alternate = COM_RX_AF[COM];
  HAL_GPIO_Init(COM_RX_PORT[COM], &gpio_init_structure);

    l_uartHandle.Instance        = USART1;
    l_uartHandle.Init.BaudRate   = 115200;
    l_uartHandle.Init.WordLength = UART_WORDLENGTH_8B;
    l_uartHandle.Init.StopBits   = UART_STOPBITS_1;
    l_uartHandle.Init.Parity     = UART_PARITY_NONE;
    l_uartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    l_uartHandle.Init.Mode       = UART_MODE_TX_RX;
    l_uartHandle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&l_uartHandle) != HAL_OK) {
        return 0U; /* return failure */
    }
    /* Set UART to receive 1 byte at a time via interrupt */
    //HAL_UART_Receive_IT(&l_uartHandle, (uint8_t *)qsRxBuf, 1);

    QS_tickPeriod_ = SystemCoreClock / BSP_TICKS_PER_SEC;
    QS_tickTime_ = QS_tickPeriod_; /* to start the timestamp at zero */

    return 1U; /* return success */
}
/*..........................................................................*/
void QS_onCleanup(void) {
}
/*..........................................................................*/
QSTimeCtr QS_onGetTime(void) {  /* NOTE: invoked with interrupts DISABLED */
    if ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0) { /* not set? */
        return QS_tickTime_ - (QSTimeCtr)SysTick->VAL;
    }
    else { /* the rollover occured, but the SysTick_ISR did not run yet */
        return QS_tickTime_ + QS_tickPeriod_ - (QSTimeCtr)SysTick->VAL;
    }
}
/*..........................................................................*/
void QS_onFlush(void) {
    uint16_t b;
#if OS_CRITICAL_METHOD == 3u  /* Allocate storage for CPU status register */
    OS_CPU_SR cpu_sr;
#endif

    OS_ENTER_CRITICAL();
    while ((b = QS_getByte()) != QS_EOD) {    /* while not End-Of-Data... */
        OS_EXIT_CRITICAL();
        while ((l_uartHandle.Instance->ISR & UART_FLAG_TXE) == 0U) { /* while TXE not empty */
        }
        l_uartHandle.Instance->TDR = (b & 0xFFU);  /* put into the DR register */
        OS_ENTER_CRITICAL();
    }
    OS_EXIT_CRITICAL();
}
/*..........................................................................*/
/*! callback function to reset the target (to be implemented in the BSP) */
void QS_onReset(void) {
    NVIC_SystemReset();
}
/*..........................................................................*/
/*! callback function to execute a user command (to be implemented in BSP) */
void QS_onCommand(uint8_t cmdId,
                  uint32_t param1, uint32_t param2, uint32_t param3)
{
    (void)cmdId;
    (void)param1;
    (void)param2;
    (void)param3;
}

#endif /* Q_SPY */
/*--------------------------------------------------------------------------*/

/*****************************************************************************
* NOTE01:
* Usually, one of the LEDs is used to visualize the idle loop activity.
* However, the board has not enough LEDs (only one, actually), so this
* feature is disabled.
*/

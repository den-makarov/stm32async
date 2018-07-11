/*******************************************************************************
 * stm32async: Asynchronous I/O C++ library for STM32
 * Test unit for the development board: STM32F405RGT6
 * *****************************************************************************
 * Copyright (C) 2016-2017 Mikhail Kulesh
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 ******************************************************************************/

// Peripherie used in this projects
#include "stm32async/HardwareLayout/Dma1.h"
#include "stm32async/HardwareLayout/Dma2.h"
#include "stm32async/HardwareLayout/PortA.h"
#include "stm32async/HardwareLayout/PortB.h"
#include "stm32async/HardwareLayout/PortC.h"
#include "stm32async/HardwareLayout/PortH.h"
#include "stm32async/HardwareLayout/Usart6.h"

// Used devices
#include "stm32async/SystemClock.h"
#include "stm32async/Rtc.h"
#include "stm32async/IOPort.h"
#include "stm32async/UsartLogger.h"

// Common includes
#include <functional>

using namespace Stm32async;

#define USART_DEBUG_MODULE "Main: "

class MyApplication : public Rtc::EventHandler
{
private:

    // Used ports
    HardwareLayout::PortA portA;
    HardwareLayout::PortB portB;
    HardwareLayout::PortC portC;
    HardwareLayout::PortH portH;

    // DMA
    HardwareLayout::Dma1 dma1;
    HardwareLayout::Dma2 dma2;

    // System and MCO
    SystemClock sysClock;
    Rtc rtc;
    MCO mco;

    // LEDs
    IOPort ledBlue, ledRed;

    // USART logger
    HardwareLayout::Usart6 usart6;
    UsartLogger usartLogger;

public:

    MyApplication () :
        // System, RTC and MCO
        sysClock { HardwareLayout::Interrupt { SysTick_IRQn, 0, 0 } },
        rtc { HardwareLayout::Interrupt { RTC_WKUP_IRQn, 10, 0 } },
        mco { portA, GPIO_PIN_8, RCC_MCO1SOURCE_PLLCLK, RCC_MCODIV_5 },
        // LEDs
        ledBlue { portC, GPIO_PIN_2, GPIO_MODE_OUTPUT_PP },
        ledRed { portC, GPIO_PIN_3, GPIO_MODE_OUTPUT_PP },
        // USART logger
        usart6 { portC, GPIO_PIN_6, portC, GPIO_PIN_7, /*remapped=*/ true, NULL,
                 HardwareLayout::Interrupt { USART6_IRQn, 1, 0 },
                 HardwareLayout::DmaStream { &dma2, DMA2_Stream7, DMA_CHANNEL_5,
                                             HardwareLayout::Interrupt { DMA2_Stream7_IRQn, 2, 0 } },
                 HardwareLayout::DmaStream { &dma2, DMA2_Stream2, DMA_CHANNEL_5,
                                             HardwareLayout::Interrupt { DMA2_Stream2_IRQn, 2, 0 } } },
        usartLogger { usart6, 115200 }
    {
        // External oscillators use system pins
        sysClock.setHSE(portH, GPIO_PIN_0 | GPIO_PIN_1);
        sysClock.setLSE(portC, GPIO_PIN_14 | GPIO_PIN_15);
    }

    virtual ~MyApplication ()
    {
        // empty
    }

    void initClock (uint32_t pllp)
    {
        SystemClock::getInstance()->setRTC();
        SystemClock::getInstance()->setSysClockSource(RCC_SYSCLKSOURCE_PLLCLK);        
        HardwareLayout::SystemPllFactors pllConfig;
        pllConfig.PLLM = 16;
        pllConfig.PLLN = 336;
        pllConfig.PLLP = pllp;
        pllConfig.PLLQ = 7;
        SystemClock::getInstance()->setPLL(&pllConfig);
        SystemClock::getInstance()->setAHB(RCC_SYSCLK_DIV1, RCC_HCLK_DIV8, RCC_HCLK_DIV8);
        SystemClock::getInstance()->setLatency(FLASH_LATENCY_3);
        SystemClock::getInstance()->start();
    }

    void run (uint32_t runId, uint32_t pllp)
    {
        initClock(pllp);
        usartLogger.initInstance();

        USART_DEBUG("--------------------------------------------------------");
        USART_DEBUG("Oscillator frequency: " << SystemClock::getInstance()->getHSEFreq()
                    << ", MCU frequency: " << SystemClock::getInstance()->getMcuFreq() << UsartLogger::ENDL
                    << UsartLogger::TAB << "runId=" << runId << UsartLogger::ENDL
                    << UsartLogger::TAB << "pllp=" << pllp);

        // For RTC, it is necessary to reset the state since it will not be
        // automatically reset after MCU programming.
        rtc.stop();
        rtc.setHandler(this);
        do
        {
            Rtc::Start::Status status = rtc.start(8 * 2047 + 7, RTC_WAKEUPCLOCK_RTCCLK_DIV2);
            USART_DEBUG("RTC status: " << Rtc::Start::asString(status) << " (" << rtc.getHalStatus() << ")");
        }
        while (rtc.getHalStatus() != HAL_OK);

        mco.start();
        ledBlue.start();
        ledRed.start();

        ledRed.setHigh();
        HAL_Delay(500);
        ledRed.setLow();

        for (int i = 0; i < 15; ++i)
        {
            // main loop empty for 15 sec
            HAL_Delay(1000);
        }

        ledBlue.stop();
        ledRed.stop();
        mco.stop();
        rtc.stop();

        // Log resource occupations after all devices (expect USART1 for logging, HSE, LSE) are stopped
        // Desired: two at portB and DMA2 (USART1), one for portC (LSE), one for portH (HSE)
        USART_DEBUG("Resource occupations: " << UsartLogger::ENDL
                    << UsartLogger::TAB << "portA=" << portA.getObjectsCount() << UsartLogger::ENDL
                    << UsartLogger::TAB << "portB=" << portB.getObjectsCount() << UsartLogger::ENDL
                    << UsartLogger::TAB << "portC=" << portC.getObjectsCount() << UsartLogger::ENDL
                    << UsartLogger::TAB << "portH=" << portH.getObjectsCount() << UsartLogger::ENDL
                    << UsartLogger::TAB << "dma1=" << dma1.getObjectsCount() << UsartLogger::ENDL
                    << UsartLogger::TAB << "dma2=" << dma2.getObjectsCount() << UsartLogger::ENDL);
        usartLogger.clearInstance();

        sysClock.stop();
    }

    void onRtcWakeUp ()
    {
        USART_DEBUG("time=" << Rtc::getInstance()->getTimeSec());
        ledBlue.toggle();
    }

    inline AsyncUsart & getLoggerUsart ()
    {
        return usartLogger.getUsart();
    }
};

MyApplication * appPtr = NULL;

int main (void)
{
    // Note: check the Value of the External oscillator mounted in PCB
    // and set this value in the file stm32f4xx_hal_conf.h
    HAL_Init();

    MyApplication app;
    appPtr = &app;

    uint32_t pllp = 2, runId = 0;
    while (true)
    {
        app.run(++runId, pllp);
        pllp += 2;
        if (pllp > 8)
        {
            pllp = 2;
        }
    }
}

extern "C"
{

// System
void SysTick_Handler (void)
{
    HAL_IncTick();
}

void RTC_WKUP_IRQHandler ()
{
    if (Rtc::getInstance() != NULL)
    {
        Rtc::getInstance()->onSecondInterrupt();
    }
}

// UART: uses both USART and DMA interrupts
void DMA2_Stream7_IRQHandler (void)
{
    appPtr->getLoggerUsart().processDmaTxInterrupt();
}

void USART6_IRQHandler (void)
{
    appPtr->getLoggerUsart().processInterrupt();
}

void HAL_UART_TxCpltCallback (UART_HandleTypeDef * /*channel*/)
{
    appPtr->getLoggerUsart().processCallback(SharedDevice::State::TX_CMPL);
}

void HAL_UART_ErrorCallback (UART_HandleTypeDef * /*channel*/)
{
    appPtr->getLoggerUsart().processCallback(SharedDevice::State::ERROR);
}

}


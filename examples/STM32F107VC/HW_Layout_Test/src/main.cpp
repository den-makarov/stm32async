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
#include "HardwareLayout/Dma1.h"
#include "HardwareLayout/Dma2.h"
#include "HardwareLayout/PortB.h"
#include "HardwareLayout/PortD.h"
//#include "HardwareLayout/PortH.h"
#include "HardwareLayout/Usart1.h"

// Used devices
#include "SystemClock.h"
#include "IOPort.h"
#include "UsartLogger.h"

// Common includes
#include <functional>

using namespace Stm32async;

#define USART_DEBUG_MODULE "Main: "

class MyApplication
{
public:

    typedef typename std::pair<uint32_t, uint32_t> FreqSettings;

private:
    
    // Used ports
    HardwareLayout::PortB portB;
    HardwareLayout::PortD portD;
    //HardwareLayout::PortH portH;

    // DMA
    HardwareLayout::Dma1 dma1;
    HardwareLayout::Dma2 dma2;

    // System and MCO
    SystemClock sysClock;
    //MCO mco;

    // LED
    IOPort led;

    // USART logger
    HardwareLayout::Usart1 usart1;
    UsartLogger usartLogger;

public:
    
    MyApplication () :
        // System and MCO
        sysClock{HardwareLayout::Interrupt{SysTick_IRQn, 0, 0}},
        //mco{portA, GPIO_PIN_8},
        led{portD, GPIO_PIN_13, GPIO_MODE_OUTPUT_PP},
        usart1 { portB, GPIO_PIN_6, portB, GPIO_PIN_7,
                 HardwareLayout::Interrupt { USART1_IRQn, 1, 0 },
                 HardwareLayout::DmaStream { &dma2, DMA2_Channel5, 0 },
                 HardwareLayout::Interrupt { DMA2_Channel5_IRQn, 2, 0 },
                 HardwareLayout::DmaStream { &dma2, DMA2_Channel2, 0 },
                 HardwareLayout::Interrupt { DMA2_Channel2_IRQn, 2, 0 }},
        usartLogger { usart1, 115200 }
    {
        // empty
    }
    
    virtual ~MyApplication ()
    {
        // empty
    }
    
    void initClock ()
    {
        SystemClock::getInstance()->setPLL(nullptr);
        SystemClock::getInstance()->setAHB(RCC_SYSCLK_DIV1, RCC_HCLK_DIV2, RCC_HCLK_DIV1);
        SystemClock::getInstance()->setLatency(FLASH_LATENCY_2);
        SystemClock::getInstance()->start();
    }

    void run (uint32_t runId)
    {
        initClock();

        usartLogger.initInstance();

        USART_DEBUG("--------------------------------------------------------");
        USART_DEBUG("Oscillator frequency: " << SystemClock::getInstance()->getHSEFreq()
                    << ", MCU frequency: " << SystemClock::getInstance()->getMcuFreq() << UsartLogger::ENDL
                    << UsartLogger::TAB << "runId=" << runId << UsartLogger::ENDL);

        //mco.start(RCC_MCO1SOURCE_PLLCLK, RCC_MCODIV_5);
        led.start();
        led.setHigh();
        HAL_Delay(500);
        led.setLow();

        //for (int i = 0; i < 15; ++i)
        {
            // main loop empty for 15 sec
            HAL_Delay(15000);
        }

        led.stop();
        //mco.stop();

        // Log resource occupations after all devices (expect USART1 for logging, HSE, LSE) are stopped
                // Desired: two at portB and DMA2 (USART1), one for portC (LSE), one for portH (HSE)
        USART_DEBUG("Resource occupations: " << UsartLogger::ENDL
                    << UsartLogger::TAB << "portB=" << portB.getObjectsCount() << UsartLogger::ENDL
                    << UsartLogger::TAB << "portD=" << portD.getObjectsCount() << UsartLogger::ENDL
                    << UsartLogger::TAB << "dma1=" << dma1.getObjectsCount() << UsartLogger::ENDL
                    << UsartLogger::TAB << "dma2=" << dma2.getObjectsCount() << UsartLogger::ENDL);
        usartLogger.clearInstance();

        SystemClock::getInstance()->stop();
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
    
    uint32_t runId = 0;
    while (true)
    {
        app.run(++runId);
    }
}


extern "C"
{
// System
void SysTick_Handler (void)
{
    HAL_IncTick();
}

/*void RTC_WKUP_IRQHandler ()
{
    if (Rtc::getInstance() != NULL)
    {
        Rtc::getInstance()->onSecondInterrupt();
    }
}*/

// UART: uses both USART and DMA interrupts
void DMA2_Channel5_IRQHandler (void)
{
    appPtr->getLoggerUsart().processDmaTxInterrupt();
}

void USART1_IRQHandler (void)
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

} /* extern "C" */

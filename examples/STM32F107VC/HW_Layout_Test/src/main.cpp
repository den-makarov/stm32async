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
#include "HardwareLayout/PortA.h"
#include "HardwareLayout/PortD.h"
#include "HardwareLayout/Usart2.h"

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
private:
    
    // Used ports
    HardwareLayout::PortD portA;
    HardwareLayout::PortD portD;

    // DMA
    HardwareLayout::Dma1 dma1;

    // System and MCO
    SystemClock sysClock;
    MCO mco;

    // LED
    IOPort ledOrange;
    IOPort ledGreen;
    IOPort ledRed;
    IOPort ledBlue;
    IOPort ** leds;

    // USART logger
    HardwareLayout::Usart2 usart2;
    UsartLogger usartLogger;

public:

    MyApplication () :
        // System and MCO
        sysClock{HardwareLayout::Interrupt{SysTick_IRQn, 0, 0}},
        mco{portA, GPIO_PIN_8, RCC_MCO1SOURCE_PLLCLK, 0},
        // Leds
        ledOrange{portD, GPIO_PIN_13, GPIO_MODE_OUTPUT_PP},
        ledGreen{portD, GPIO_PIN_7, GPIO_MODE_OUTPUT_PP},
        ledRed{portD, GPIO_PIN_3, GPIO_MODE_OUTPUT_PP},
        ledBlue{portD, GPIO_PIN_4, GPIO_MODE_OUTPUT_PP},
        leds{new IOPort * [4]},
        // Serial Port
        usart2 { portD, GPIO_PIN_5, portD, GPIO_PIN_6, true,
                 HardwareLayout::Interrupt { USART2_IRQn, 1, 0 },
                 HardwareLayout::DmaStream { &dma1, DMA1_Channel7, 0 },
                 HardwareLayout::Interrupt { DMA1_Channel7_IRQn, 2, 0 },
                 HardwareLayout::DmaStream { &dma1, DMA1_Channel6, 0 },
                 HardwareLayout::Interrupt { DMA1_Channel6_IRQn, 2, 0 }},
        // Output stream
        usartLogger { usart2, 115200 }
    {
        leds[0] = &ledRed;
        leds[1] = &ledOrange;
        leds[2] = &ledGreen;
        leds[3] = &ledBlue;
    }
    
    virtual ~MyApplication ()
    {
        delete [] leds;
    }
    
    void initClock ()
    {
        SystemClock::getInstance()->setSysClockSource(RCC_SYSCLKSOURCE_PLLCLK);
        HardwareLayout::SystemPllFactors pllConfig;

        // Next parameters configure SystemClock to 72 MHz
        pllConfig.Prediv1Source = RCC_PREDIV1_SOURCE_PLL2;
        pllConfig.HSEPredivValue = RCC_HSE_PREDIV_DIV5;
        pllConfig.HSEPrediv2Value = RCC_HSE_PREDIV2_DIV5;
        pllConfig.PLLMUL = RCC_PLL_MUL9;
        pllConfig.PLL2MUL = RCC_PLL2_MUL8;

        SystemClock::getInstance()->setPLL(&pllConfig);
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

        mco.start();
        for(auto i = 0; i < 4; i++)
        {
            leds[i]->start();
        }

        leds[0]->setHigh();
        HAL_Delay(500);
        leds[0]->setLow();

        for (int i = 0; i < 15; ++i)
        {
            // main loop empty for 15 sec
            HAL_Delay(333);
            leds[1]->toggle();
            HAL_Delay(333);
            leds[2]->toggle();
            HAL_Delay(333);
            leds[3]->toggle();
        }

        for(auto i = 0; i < 4; i++)
        {
            leds[i]->stop();
        }
        mco.stop();

        // Log resource occupations after all devices (expect USART2 for logging) are stopped
        // Desired: two at portD, two DMA2 (USART2), null for portA
        USART_DEBUG("Resource occupations: " << UsartLogger::ENDL
                    << UsartLogger::TAB << "portA=" << portA.getObjectsCount() << UsartLogger::ENDL
                    << UsartLogger::TAB << "portD=" << portD.getObjectsCount() << UsartLogger::ENDL
                    << UsartLogger::TAB << "dma1=" << dma1.getObjectsCount() << UsartLogger::ENDL);
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
void DMA1_Channel7_IRQHandler (void)
{
    appPtr->getLoggerUsart().processDmaTxInterrupt();
}

void DMA1_Channel6_IRQHandler (void)
{
    appPtr->getLoggerUsart().processDmaRxInterrupt();
}

void USART2_IRQHandler (void)
{
    appPtr->getLoggerUsart().processInterrupt();
}

void HAL_UART_TxCpltCallback (UART_HandleTypeDef * huart)
{
    UNUSED(huart);
    appPtr->getLoggerUsart().processCallback(SharedDevice::State::TX_CMPL);
}

void HAL_UART_ErrorCallback (UART_HandleTypeDef * huart)
{
    UNUSED(huart);
    appPtr->getLoggerUsart().processCallback(SharedDevice::State::ERROR);
}

} /* extern "C" */

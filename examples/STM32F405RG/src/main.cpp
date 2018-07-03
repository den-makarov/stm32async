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
#include "stm32async/HardwareLayout/PortA.h"
#include "stm32async/HardwareLayout/PortB.h"
#include "stm32async/HardwareLayout/PortC.h"
#include "stm32async/HardwareLayout/PortH.h"
#include "stm32async/HardwareLayout/Usart1.h"

// Used devices
#include "stm32async/SystemClock.h"
#include "stm32async/IOPort.h"
#include "stm32async/AsyncUsart.h"

// Common includes
#include <functional>
#include <cstring>

using namespace Stm32async;

class MyApplication
{
private:

    // Used ports
    HardwareLayout::PortA portA;
    HardwareLayout::PortB portB;
    HardwareLayout::PortC portC;
    HardwareLayout::PortH portH;

    // System and MCO
    SystemClock sysClock;
    MCO mco;

    // LED
    IOPort ledBlue, ledRed;

    // USART
    HardwareLayout::Usart1 usart1;
    AsyncUsart loggerUsart;

public:

    MyApplication () :
        // System and MCO
        sysClock { HardwareLayout::Interrupt { SysTick_IRQn, 0, 0 } },
        mco { portA, GPIO_PIN_8 },
        ledBlue { portC, GPIO_PIN_2, GPIO_MODE_OUTPUT_PP },
        ledRed { portC, GPIO_PIN_3, GPIO_MODE_OUTPUT_PP },
        usart1 { portB, GPIO_PIN_6, portB, GPIO_PIN_7, HardwareLayout::Interrupt {USART1_IRQn, 1, 0} },
        loggerUsart { usart1 }
    {
        // External resonators use system pins
        sysClock.setHSE(portH, GPIO_PIN_0 | GPIO_PIN_1);
        sysClock.setLSE(portC, GPIO_PIN_14 | GPIO_PIN_15);
    }

    virtual ~MyApplication ()
    {
        // empty
    }

    void initClock (uint32_t pllp)
    {
        SystemClock::getInstance()->setPLL(16, 336, pllp, 7);
        SystemClock::getInstance()->setAHB(RCC_SYSCLK_DIV1, RCC_HCLK_DIV8, RCC_HCLK_DIV8);
        SystemClock::getInstance()->setLatency(FLASH_LATENCY_3);
        SystemClock::getInstance()->start();
    }

    void run (uint32_t pllp)
    {
        initClock(pllp);
        mco.start(RCC_MCO1SOURCE_PLLCLK, RCC_MCODIV_5);
        ledBlue.start();
        ledRed.start();
        ledRed.setLow();

        ledRed.setHigh();
        HAL_Delay(500);
        ledRed.setLow();

        loggerUsart.setTimeout(1000);
        if (loggerUsart.start(UART_MODE_TX, 115200, UART_WORDLENGTH_8B, UART_STOPBITS_1, UART_PARITY_NONE) == HAL_OK)
        {
            const char * h = "Hello world!\n\r";
            loggerUsart.transmit(NULL, h, ::strlen(h));
            loggerUsart.waitForRelease();
        }

        for (int i = 0; i < 30; ++i)
        {
            // main loop
            ledBlue.toggle();
            HAL_Delay(500);
        }

        loggerUsart.stop();
        ledBlue.stop();
        mco.stop();
        sysClock.stop();
    }

    inline AsyncUsart & getLoggerUsart ()
    {
        return loggerUsart;
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

    uint32_t pllp = 2;
    while (true)
    {
        app.run(pllp);
        ++pllp;
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

// UART
void USART1_IRQHandler(void)
{
    appPtr->getLoggerUsart().processInterrupt();
}

void HAL_UART_TxCpltCallback (UART_HandleTypeDef * channel)
{
    appPtr->getLoggerUsart().processCallback(SharedDevice::State::TX_CMPL);
}

void HAL_UART_ErrorCallback (UART_HandleTypeDef * channel)
{
    appPtr->getLoggerUsart().processCallback(SharedDevice::State::ERROR);
}


}


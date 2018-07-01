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
#include "stm32async/HardwareLayout/PortC.h"
#include "stm32async/HardwareLayout/PortH.h"

// Used devices
#include "stm32async/System.h"
#include "stm32async/IOPort.h"

// Common includes
#include <functional>

using namespace Stm32async;

class MyApplication
{
public:

    typedef typename std::pair<uint32_t, uint32_t> FreqSettings;

private:

    // Used ports
    HardwareLayout::PortA portA;
    HardwareLayout::PortC portC;
    HardwareLayout::PortH portH;

    // System and MCO
    System sys;
    MCO mco;

    // LED
    IOPort ledBlue, ledRed;

public:

    MyApplication () :
        // System and MCO
        sys { HardwareLayout::Interrupt { SysTick_IRQn, 0, 0 } },
        mco { portA, GPIO_PIN_8 },
        ledBlue { portC, GPIO_PIN_2, GPIO_MODE_OUTPUT_PP },
        ledRed { portC, GPIO_PIN_3, GPIO_MODE_OUTPUT_PP }
    {
        sys.initHSE(portH, GPIO_PIN_0 | GPIO_PIN_1);
        sys.initLSE(portC, GPIO_PIN_14 | GPIO_PIN_15);
    }

    virtual ~MyApplication ()
    {
        // empty
    }

    void initClock (uint32_t pllp)
    {
        sys.initInstance();
        sys.initPLL(16, 336, pllp, 7);
        sys.initAHB(RCC_SYSCLK_DIV1, RCC_HCLK_DIV8, RCC_HCLK_DIV8);
        sys.start(FLASH_LATENCY_3);
    }

    void run (uint32_t pllp)
    {
        initClock(pllp);
        mco.start(RCC_MCO1SOURCE_PLLCLK, RCC_MCODIV_5);
        ledBlue.start();
        ledRed.start();
        ledRed.setHigh();
        HAL_Delay(500);
        ledRed.setLow();

        for (int i = 0; i < 30; ++i)
        {
            // main loop
            ledBlue.toggle();
            HAL_Delay(500);
        }

        ledBlue.stop();
        mco.stop();
        sys.stop();
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

}


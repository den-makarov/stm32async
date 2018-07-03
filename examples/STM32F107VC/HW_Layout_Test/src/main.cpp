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
//#include "HardwareLayout/PortA.h"
#include "HardwareLayout/PortD.h"
//#include "HardwareLayout/PortH.h"

// Used devices
#include "SystemClock.h"
#include "IOPort.h"

// Common includes
#include <functional>

using namespace Stm32async;

class MyApplication
{
public:

    typedef typename std::pair<uint32_t, uint32_t> FreqSettings;

private:
    
    // Used ports
    //HardwareLayout::PortA portA;
    HardwareLayout::PortD portD;
    //HardwareLayout::PortH portH;

    // System and MCO
    SystemClock sys;
    //MCO mco;

    // LED
    IOPort led;

public:
    
    MyApplication () :
        // System and MCO
        sys{HardwareLayout::Interrupt{SysTick_IRQn, 0, 0}},
        //mco{portA, GPIO_PIN_8},
        led{portD, GPIO_PIN_13, GPIO_MODE_OUTPUT_PP}
    {
        // empty
    }
    
    virtual ~MyApplication ()
    {
        // empty
    }
    
    void initClock ()
    {
        sys.setPLL();
        sys.setAHB(RCC_SYSCLK_DIV1, RCC_HCLK_DIV2, RCC_HCLK_DIV1);
        sys.start();
    }

    void run ()
    {
        initClock();
        //mco.start(RCC_MCO1SOURCE_PLLCLK, RCC_MCODIV_5);
        led.start();

        for (int i = 0; i < 30; ++i)
        {
            // main loop
        	led.toggle();
            HAL_Delay(500);
        }

        led.stop();
        //mco.stop();
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
    
    while (true)
    {
        app.run();
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


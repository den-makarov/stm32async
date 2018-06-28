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

#include "stm32async/System.h"

#include <functional>

using namespace Stm32async;

class MyApplication
{
public:

    typedef typename std::pair<uint32_t, uint32_t> FreqSettings;

private:
    
    System sys;
    GPIO_InitTypeDef mcoParameters;
    GPIO_InitTypeDef ledParameters;

public:
    
    MyApplication () :
        // system
        sys{Interrupt{SysTick_IRQn, 0, 0}}
    {
        // empt
    }
    
    virtual ~MyApplication ()
    {
        // empty
    }
    
    void initClock (uint32_t pllp)
    {
        // Set system frequency to 168MHz
        sys.initHSE(/*external=*/ true);
        sys.initPLL(16, 336, pllp, 7);
        sys.initLSE(/*external=*/ true);
        sys.initAHB(RCC_SYSCLK_DIV1, RCC_HCLK_DIV8, RCC_HCLK_DIV8);
        sys.start(FLASH_LATENCY_3);
        sys.initInstance();
    }

    void activateClockOutput (uint32_t source, uint32_t div)
    {
        __GPIOA_CLK_ENABLE();
        mcoParameters.Pin = GPIO_PIN_8;
        mcoParameters.Mode = GPIO_MODE_AF_PP;
        mcoParameters.Pull = GPIO_NOPULL;
        mcoParameters.Speed = GPIO_SPEED_FREQ_HIGH;
        mcoParameters.Alternate = GPIO_AF0_MCO;
        HAL_GPIO_Init(GPIOA, &mcoParameters);
        HAL_RCC_MCOConfig(RCC_MCO1, source, div);
    }

    void stopClockOutput ()
    {
        HAL_GPIO_DeInit(GPIOA, mcoParameters.Pin);
        __GPIOA_CLK_DISABLE();
    }

    void activateLED ()
    {
        __GPIOC_CLK_ENABLE();
        ledParameters.Pin = GPIO_PIN_2;
        ledParameters.Mode = GPIO_MODE_OUTPUT_PP;
        ledParameters.Pull = GPIO_NOPULL;
        ledParameters.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(GPIOC, &ledParameters);
    }

    void stopLED ()
    {
        HAL_GPIO_DeInit(GPIOC, ledParameters.Pin);
        __GPIOC_CLK_DISABLE();
    }

    void run (uint32_t pllp)
    {
        initClock(pllp);
        activateClockOutput(RCC_MCO1SOURCE_PLLCLK, RCC_MCODIV_5);
        activateLED();

        bool ledValue = true;
        for (int i = 0; i < 30; ++i)
        {
            // main loop
            HAL_GPIO_WritePin(GPIOC, ledParameters.Pin, (GPIO_PinState)ledValue);
            ledValue = !ledValue;
            HAL_Delay(500);
        }

        stopLED();
        stopClockOutput();
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


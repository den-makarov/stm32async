/*******************************************************************************
 * stm32async: Asynchronous I/O C++ library for STM32
 * *****************************************************************************
 * Copyright (C) 2018 Mikhail Kulesh, Denis Makarov
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

#include "System.h"

using namespace Stm32async;

/************************************************************************
 * Class System
 ************************************************************************/

System * System::instance = NULL;

System::System (Interrupt && _sysTickIrq):
    sysTickIrq{std::move(_sysTickIrq)},
    mcuFreq{0}
{
    oscParams.OscillatorType = RCC_OSCILLATORTYPE_NONE;
    oscParams.HSEState = RCC_HSE_OFF;
    oscParams.HSIState = RCC_HSI_OFF;
    oscParams.LSEState = RCC_LSE_OFF;
    oscParams.LSIState = RCC_LSI_OFF;
    clkParams.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
}


void System::initHSE (bool external)
{
    if (external)
    {
        oscParams.OscillatorType |= RCC_OSCILLATORTYPE_HSE;
        oscParams.HSEState = RCC_HSE_ON;
        clkParams.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
    }
    else
    {
        oscParams.OscillatorType |= RCC_OSCILLATORTYPE_HSI;
        oscParams.HSIState = RCC_HSI_ON;
        clkParams.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    }
}


void System::initLSE (bool external)
{
    if (external)
    {
        oscParams.OscillatorType |= RCC_OSCILLATORTYPE_LSE;
        oscParams.LSEState = RCC_LSE_ON;
    }
    else
    {
        oscParams.OscillatorType |= RCC_OSCILLATORTYPE_LSI;
        oscParams.LSIState = RCC_LSI_ON;
    }
}


void System::initPLL (uint32_t PLLM, uint32_t PLLN, uint32_t PLLP, uint32_t PLLQ, uint32_t PLLR)
{
    oscParams.PLL.PLLState = RCC_PLL_ON;
    if (oscParams.HSEState == RCC_HSE_ON)
    {

        oscParams.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    }
    else
    {
        oscParams.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    }
    oscParams.PLL.PLLM = PLLM;
    oscParams.PLL.PLLN = PLLN;
    oscParams.PLL.PLLP = PLLP;
    oscParams.PLL.PLLQ = PLLQ;
    #ifdef STM32F410Rx
    oscParams.PLL.PLLR = PLLR;
    #endif
    clkParams.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
}


void System::initAHB (uint32_t AHBCLKDivider, uint32_t APB1CLKDivider, uint32_t APB2CLKDivider)
{
    clkParams.AHBCLKDivider =  AHBCLKDivider;
    clkParams.APB1CLKDivider = APB1CLKDivider;
    clkParams.APB2CLKDivider = APB2CLKDivider;
}


void System::initRTC ()
{
    if (oscParams.LSIState == RCC_LSI_ON)
    {
        periphClkParams.PeriphClockSelection |= RCC_PERIPHCLK_RTC;
        periphClkParams.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
    }
    if (oscParams.LSEState == RCC_LSE_ON)
    {
        periphClkParams.PeriphClockSelection |= RCC_PERIPHCLK_RTC;
        periphClkParams.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
    }
}


void System::initI2S (uint32_t PLLI2SN, uint32_t PLLI2SR)
{
    periphClkParams.PeriphClockSelection |= RCC_PERIPHCLK_I2S;
    periphClkParams.PLLI2S.PLLI2SN = PLLI2SN;
    periphClkParams.PLLI2S.PLLI2SR = PLLI2SR;
}


void System::start (uint32_t fLatency, int32_t msAdjustment)
{
    __HAL_RCC_PWR_CLK_ENABLE();
    if (oscParams.LSEState == RCC_LSE_ON)
    {
        __HAL_RCC_GPIOC_CLK_ENABLE();
    }
    if (oscParams.HSEState == RCC_HSE_ON)
    {
        __HAL_RCC_GPIOH_CLK_ENABLE();
    }
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    HAL_RCC_OscConfig(&oscParams);
    HAL_RCC_ClockConfig(&clkParams, fLatency);

    /* STM32F405x/407x/415x/417x Revision Z devices: prefetch is supported  */
    if (HAL_GetREVID() == 0x1001)
    {
    /* Enable the Flash prefetch */
    __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
    }

    HAL_RCCEx_PeriphCLKConfig(&periphClkParams);

    mcuFreq = HAL_RCC_GetHCLKFreq();
    HAL_SYSTICK_Config(mcuFreq/1000 + msAdjustment);
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /* SysTick_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(sysTickIrq.irqn, sysTickIrq.prio, sysTickIrq.subPrio);
}

void System::stop ()
{
    HAL_RCC_DeInit();
    if (oscParams.HSEState == RCC_HSE_ON)
    {
        __HAL_RCC_GPIOH_CLK_DISABLE();
    }
    if (oscParams.LSEState == RCC_LSE_ON)
    {
        __HAL_RCC_GPIOC_CLK_DISABLE();
    }
    __HAL_RCC_PWR_CLK_DISABLE();
}

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

System::System (HardwareLayout::Interrupt && _sysTickIrq) :
    sysTickIrq { std::move(_sysTickIrq) },
    hsePort { NULL },
    lsePort { NULL },
    mcuFreq { 0 }
{
    oscParameters.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    oscParameters.HSEState = RCC_HSE_OFF;
    // By default at least next one should be enabled
    oscParameters.HSIState = RCC_HSI_ON;
    oscParameters.LSEState = RCC_LSE_OFF;
    oscParameters.LSIState = RCC_LSI_OFF;

    clkParameters.ClockType = RCC_CLOCKTYPE_HCLK | 
                              RCC_CLOCKTYPE_SYSCLK | 
                              RCC_CLOCKTYPE_PCLK1 | 
                              RCC_CLOCKTYPE_PCLK2;
}

void System::initHSE (const HardwareLayout::Port & _port, uint32_t /*pin*/)
{
    hsePort = &_port;
    oscParameters.OscillatorType &= ~RCC_OSCILLATORTYPE_HSI;
    oscParameters.OscillatorType |= RCC_OSCILLATORTYPE_HSE;
    oscParameters.HSEState = RCC_HSE_ON;
    oscParameters.HSIState = RCC_HSI_OFF;
    clkParameters.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
}

void System::initHSI ()
{
    oscParameters.OscillatorType &= ~RCC_OSCILLATORTYPE_HSE;
    oscParameters.OscillatorType |= RCC_OSCILLATORTYPE_HSI;
    oscParameters.HSEState = RCC_HSE_OFF;
    oscParameters.HSIState = RCC_HSI_ON;
    clkParameters.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
}

void System::initLSE (const HardwareLayout::Port & _port, uint32_t /*pin*/)
{
    lsePort = &_port;
    oscParameters.OscillatorType &= ~RCC_OSCILLATORTYPE_LSI;
    oscParameters.OscillatorType |= RCC_OSCILLATORTYPE_LSE;
    oscParameters.LSEState = RCC_LSE_ON;
    oscParameters.LSIState = RCC_LSI_OFF;
}

void System::initLSI ()
{
    oscParameters.OscillatorType &= ~RCC_OSCILLATORTYPE_LSE;
    oscParameters.OscillatorType |= RCC_OSCILLATORTYPE_LSI;
    oscParameters.LSIState = RCC_LSI_ON;
    oscParameters.LSEState = RCC_LSE_OFF;
}

void System::initPLL (uint32_t PLLM, uint32_t PLLN, uint32_t PLLP, uint32_t PLLQ, uint32_t PLLR)
{
    oscParameters.PLL.PLLState = RCC_PLL_ON;
    if (oscParameters.HSEState == RCC_HSE_ON)
    {
        oscParameters.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    }
    else
    {
        oscParameters.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    }
    oscParameters.PLL.PLLM = PLLM;
    oscParameters.PLL.PLLN = PLLN;
    oscParameters.PLL.PLLP = PLLP;
    oscParameters.PLL.PLLQ = PLLQ;
#ifdef STM32F410Rx
    oscParameters.PLL.PLLR = PLLR;
#endif
    clkParameters.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
}

void System::initAHB (uint32_t AHBCLKDivider, uint32_t APB1CLKDivider, uint32_t APB2CLKDivider)
{
    clkParameters.AHBCLKDivider = AHBCLKDivider;
    clkParameters.APB1CLKDivider = APB1CLKDivider;
    clkParameters.APB2CLKDivider = APB2CLKDivider;
}

void System::initRTC ()
{
    if (oscParameters.LSIState == RCC_LSI_ON)
    {
        periphClkParameters.PeriphClockSelection |= RCC_PERIPHCLK_RTC;
        periphClkParameters.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
    }
    if (oscParameters.LSEState == RCC_LSE_ON)
    {
        periphClkParameters.PeriphClockSelection |= RCC_PERIPHCLK_RTC;
        periphClkParameters.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
    }
}

//TODO: Hmm, what to do?
/*void System::initI2S (uint32_t PLLI2SN, uint32_t PLLI2SR)
{
    periphClkParameters.PeriphClockSelection |= RCC_PERIPHCLK_I2S;
    periphClkParameters.PLLI2S.PLLI2SN = PLLI2SN;
    periphClkParameters.PLLI2S.PLLI2SR = PLLI2SR;
}*/

void System::start (uint32_t fLatency, int32_t msAdjustment)
{
    __HAL_RCC_PWR_CLK_ENABLE();
    if (hsePort != NULL)
    {
        hsePort->enableClock();
    }
    if (lsePort != NULL)
    {
        lsePort->enableClock();
    }
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    HAL_RCC_OscConfig(&oscParameters);
    HAL_RCC_ClockConfig(&clkParameters, fLatency);

    /* STM32F405x/407x/415x/417x Revision Z devices: prefetch is supported  */
    if (HAL_GetREVID() == 0x1001)
    {
        /* Enable the Flash prefetch */
        __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
    }

    HAL_RCCEx_PeriphCLKConfig(&periphClkParameters);

    mcuFreq = HAL_RCC_GetHCLKFreq();
    HAL_SYSTICK_Config(mcuFreq / 1000 + msAdjustment);
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /* SysTick_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(sysTickIrq.irqn, sysTickIrq.prio, sysTickIrq.subPrio);
}

void System::stop ()
{
    HAL_RCC_DeInit();
    if (hsePort != NULL)
    {
        hsePort->disableClock();
    }
    if (lsePort != NULL)
    {
        lsePort->disableClock();
    }
    __HAL_RCC_PWR_CLK_DISABLE();
}

/************************************************************************
 * Class MCO
 ************************************************************************/

MCO::MCO (const HardwareLayout::Port & _port, uint32_t pin) :
    port { _port }
{
    parameters.Pin = pin;
    parameters.Mode = GPIO_MODE_AF_PP;
    parameters.Pull = GPIO_NOPULL;
    parameters.Speed = GPIO_SPEED_FREQ_HIGH;
    parameters.Alternate = GPIO_AF0_MCO;
}

void MCO::start (uint32_t source, uint32_t div)
{
    port.enableClock();
    HAL_GPIO_Init(port.instance, &parameters);
    HAL_RCC_MCOConfig(RCC_MCO1, source, div);
}

void MCO::stop ()
{
    HAL_GPIO_DeInit(port.instance, parameters.Pin);
    port.disableClock();
}


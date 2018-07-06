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

#include "SystemClock.h"

using namespace Stm32async;

/************************************************************************
 * Class System
 ************************************************************************/

SystemClock * SystemClock::instance = NULL;

SystemClock::SystemClock (HardwareLayout::Interrupt && _sysTickIrq) :
    sysTickIrq { std::move(_sysTickIrq) },
    hsePort { NULL },
    lsePort { NULL },
    mcuFreq { 0 },
#if defined(STM32F4)
    fLatency { FLASH_LATENCY_7 }
#elif defined(STM32F1)
    fLatency { FLASH_LATENCY_2 }
#endif
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
    instance = this;
}

void SystemClock::setHSE (const HardwareLayout::Port & _port, uint32_t /*pin*/)
{
    hsePort = &_port;
    oscParameters.OscillatorType &= ~RCC_OSCILLATORTYPE_HSI;
    oscParameters.OscillatorType |= RCC_OSCILLATORTYPE_HSE;
    oscParameters.HSEState = RCC_HSE_ON;
    oscParameters.HSIState = RCC_HSI_OFF;
    clkParameters.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
}

void SystemClock::setHSI ()
{
    oscParameters.OscillatorType &= ~RCC_OSCILLATORTYPE_HSE;
    oscParameters.OscillatorType |= RCC_OSCILLATORTYPE_HSI;
    oscParameters.HSEState = RCC_HSE_OFF;
    oscParameters.HSIState = RCC_HSI_ON;
    clkParameters.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
}

void SystemClock::setLSE (const HardwareLayout::Port & _port, uint32_t /*pin*/)
{
    lsePort = &_port;
    oscParameters.OscillatorType &= ~RCC_OSCILLATORTYPE_LSI;
    oscParameters.OscillatorType |= RCC_OSCILLATORTYPE_LSE;
    oscParameters.LSEState = RCC_LSE_ON;
    oscParameters.LSIState = RCC_LSI_OFF;
}

void SystemClock::setLSI ()
{
    oscParameters.OscillatorType &= ~RCC_OSCILLATORTYPE_LSE;
    oscParameters.OscillatorType |= RCC_OSCILLATORTYPE_LSI;
    oscParameters.LSIState = RCC_LSI_ON;
    oscParameters.LSEState = RCC_LSE_OFF;
}

#ifdef STM32F1
/** 
  * @brief Configure PLLs
  * @brief PLL2 configuration: PLL2CLK = (HSE / HSEPrediv2Value) * PLL2MUL = (25 / 5) * 8 = 40 MHz
  * @brief PREDIV1 configuration: PREDIV1CLK = PLL2CLK / HSEPredivValue = 40 / 5 = 8 MHz
  * @brief PLL configuration: PLLCLK = PREDIV1CLK * PLLMUL = 8 * 9 = 72 MHz
  * @brief Enable HSE Oscillator and activate PLL with HSE as source
  */
void SystemClock::setPLL (HardwareLayout::SystemPllFactors * /*factors*/)
{
    oscParameters.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    oscParameters.HSEState = RCC_HSE_ON;
    oscParameters.HSEPredivValue = RCC_HSE_PREDIV_DIV5;
    oscParameters.Prediv1Source = RCC_PREDIV1_SOURCE_PLL2;
    oscParameters.PLL.PLLState = RCC_PLL_ON;
    oscParameters.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    oscParameters.PLL.PLLMUL = RCC_PLL_MUL9;
    oscParameters.PLL2.PLL2State = RCC_PLL2_ON;
    oscParameters.PLL2.PLL2MUL = RCC_PLL2_MUL8;
    oscParameters.PLL2.HSEPrediv2Value = RCC_HSE_PREDIV2_DIV5;
}
#endif /* STM32F1 */

#ifdef STM32F4
void SystemClock::setPLL (HardwareLayout::SystemPllFactors * factors)
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
    oscParameters.PLL.PLLM = factors->PLLM;
    oscParameters.PLL.PLLN = factors->PLLN;
    oscParameters.PLL.PLLP = factors->PLLP;
    oscParameters.PLL.PLLQ = factors->PLLQ;
#ifdef STM32F410Rx
    oscParameters.PLL.PLLR = factors->PLLR;
#endif
    clkParameters.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
}
#endif /* STM32F4 */

void SystemClock::setAHB (uint32_t AHBCLKDivider, uint32_t APB1CLKDivider, uint32_t APB2CLKDivider)
{
    clkParameters.AHBCLKDivider = AHBCLKDivider;
    clkParameters.APB1CLKDivider = APB1CLKDivider;
    clkParameters.APB2CLKDivider = APB2CLKDivider;
}

void SystemClock::setRTC ()
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

void SystemClock::setI2S (uint32_t PLLI2SN, uint32_t PLLI2SR)
{
#ifdef HAL_I2S_MODULE_ENABLED
    periphClkParameters.PeriphClockSelection |= RCC_PERIPHCLK_I2S;
    periphClkParameters.PLLI2S.PLLI2SN = PLLI2SN;
    periphClkParameters.PLLI2S.PLLI2SR = PLLI2SR;
#else
    UNUSED(PLLI2SN);
    UNUSED(PLLI2SR);
#endif
}

#ifdef STM32F1
void SystemClock::start ()
{
    if (HAL_RCC_OscConfig(&oscParameters) != HAL_OK)
    {
        /* Initialization Error */
        while(1);
    }
    //TODO: second parameter must be configurable
    if (HAL_RCC_ClockConfig(&clkParameters, fLatency)!= HAL_OK)
    {
        /* Initialization Error */
        while(1);
    }
}
#endif /* STM32F1 */

#ifdef STM32F4
void SystemClock::start ()
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
    HAL_SYSTICK_Config(mcuFreq / 1000);
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /* SysTick_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(sysTickIrq.irqn, sysTickIrq.prio, sysTickIrq.subPrio);
}
#endif /* STM32F4 */

void SystemClock::stop ()
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

MCO::MCO (const HardwareLayout::Port & _port, uint32_t pin, uint32_t _source, uint32_t _div) :
    port { _port },
    source { _source },
    divider { _div }
{
    parameters.Pin = pin;
    parameters.Mode = GPIO_MODE_AF_PP;
    parameters.Pull = GPIO_NOPULL;
    parameters.Speed = GPIO_SPEED_FREQ_HIGH;
    #ifdef STM32F4
    parameters.Alternate = GPIO_AF0_MCO;
    #endif
}

void MCO::start ()
{
    port.enableClock();
    HAL_GPIO_Init(port.getInstance(), &parameters);
    HAL_RCC_MCOConfig(RCC_MCO1, source, divider);
}

void MCO::stop ()
{
    HAL_GPIO_DeInit(port.getInstance(), parameters.Pin);
    port.disableClock();
}


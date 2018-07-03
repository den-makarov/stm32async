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

#include <cstdlib>

#include "HardwareLayout.h"

#ifndef STM32ASYNC_SYSTEMCLOCK_H_
#define STM32ASYNC_SYSTEMCLOCK_H_

namespace Stm32async
{

/**
 * @brief Singleton class collecting helper methods for general system clock settings.
 */
class SystemClock final
{
    DECLARE_STATIC_INSTANCE(SystemClock)

public:

    SystemClock (HardwareLayout::Interrupt && sysTickIrq);

    void setHSE (const HardwareLayout::Port & _port, uint32_t pin);
    void setHSI ();
    void setLSE (const HardwareLayout::Port & _port, uint32_t pin);
    void setLSI ();

    #ifdef STM32F1
    void setPLL ();
    #endif /* STM32F1 */

    #ifdef STM32F4
    void setPLL (uint32_t PLLM, uint32_t PLLN, uint32_t PLLP, uint32_t PLLQ, uint32_t PLLR = 0);
    #endif /* STM32F4 */

    void setAHB (uint32_t AHBCLKDivider, uint32_t APB1CLKDivider, uint32_t APB2CLKDivider);
    void setRTC ();
    #ifdef STM32F4
    void setI2S (uint32_t PLLI2SN, uint32_t PLLI2SR);
    #endif /* STM32F4 */

    void start ();
    void stop ();

    inline void setLatency (uint32_t _fLatency)
    {
        fLatency = _fLatency;
    }

    inline RCC_OscInitTypeDef & getOscParameters ()
    {
        return oscParameters;
    }

    inline RCC_ClkInitTypeDef & getClkParameters ()
    {
        return clkParameters;
    }

    inline RCC_PeriphCLKInitTypeDef & getPeriphClkParameters ()
    {
        return periphClkParameters;
    }

    inline uint32_t getHSEFreq () const
    {
        return HSE_VALUE;
    }

    inline uint32_t getHSIFreq () const
    {
        return HSI_VALUE;
    }

    inline uint32_t getMcuFreq () const
    {
        return mcuFreq;
    }

private:

    RCC_OscInitTypeDef oscParameters;
    RCC_ClkInitTypeDef clkParameters;
    RCC_PeriphCLKInitTypeDef periphClkParameters;
    HardwareLayout::Interrupt sysTickIrq;
    const HardwareLayout::Port * hsePort;
    const HardwareLayout::Port * lsePort;
    uint32_t mcuFreq;
    uint32_t fLatency;
};

/**
 * @brief Class that implements microcontroller clock output.
 */
class MCO final
{
public:

    MCO (const HardwareLayout::Port & _port, uint32_t pin);
    void start (uint32_t source, uint32_t div);
    void stop ();

private:

    const HardwareLayout::Port & port;
    GPIO_InitTypeDef parameters;
};

} // end namespace
#endif

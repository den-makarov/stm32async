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

#include "HardwareLayout.h"

#include <cstdlib>

#ifndef STM32ASYNC_SYSTEM_H_
#define STM32ASYNC_SYSTEM_H_

namespace Stm32async
{

#define DECLARE_SINGLETON(name) \
    private: \
        static name * instance; \
    public: \
        inline void initInstance () { instance = this; } \
        static System * getInstance () { return instance; }

/**
 * @brief Singleton class collecting helper methods for general system settings.
 */
class System final
{
    DECLARE_SINGLETON(System);

public:

    System (HardwareLayout::Interrupt && sysTickIrq);

    void initHSE (const HardwareLayout::Port & _port, uint32_t pin);
    void initHSI ();
    void initLSE (const HardwareLayout::Port & _port, uint32_t pin);
    void initLSI ();
    void initPLL (uint32_t PLLM, uint32_t PLLN, uint32_t PLLP, uint32_t PLLQ, uint32_t PLLR = 0);
    void initAHB (uint32_t AHBCLKDivider, uint32_t APB1CLKDivider, uint32_t APB2CLKDivider);
    void initRTC ();
    void initI2S (uint32_t PLLI2SN, uint32_t PLLI2SR);

    void start (uint32_t fLatency, int32_t msAdjustment = 0);
    void stop ();

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

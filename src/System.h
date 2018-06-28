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

#ifdef STM32F4
#include "stm32f4xx.h"
#endif

#include <cstdlib>
#include <utility>

#ifndef STM32ASYNC_SYSTEM_H_
#define STM32ASYNC_SYSTEM_H_

namespace Stm32async {

/**
 * @brief Helper class used to configure interrupt parameters.
 */
class Interrupt
{
public:
    /**
     * @brief Interrupt Number Definition
     */
    IRQn_Type irqn;
    uint32_t prio, subPrio;

    Interrupt (IRQn_Type _irqn, uint32_t _prio, uint32_t _subPrio):
        irqn{_irqn},
        prio{_prio},
        subPrio{_subPrio}
    {
       // empty
    }

    Interrupt (Interrupt && irq):
        irqn{irq.irqn},
        prio{irq.prio},
        subPrio{irq.subPrio}
    {
       // empty
    }

    inline void start () const
    {
        HAL_NVIC_SetPriority(irqn, prio, subPrio);
        HAL_NVIC_EnableIRQ(irqn);
    }

    inline void stop () const
    {
        HAL_NVIC_DisableIRQ(irqn);
    }
};


/**
 * @brief Singleton class collecting helper methods for general system settings.
 */
class System
{
public:

    System (Interrupt && sysTickIrq);

    inline void initInstance ()
    {
        instance = this;
    }

    static System * getInstance ()
    {
        return instance;
    }

    void initHSE (bool external);
    void initLSE (bool external);
    void initPLL (uint32_t PLLM, uint32_t PLLN, uint32_t PLLP, uint32_t PLLQ, uint32_t PLLR = 0);
    void initAHB (uint32_t AHBCLKDivider, uint32_t APB1CLKDivider, uint32_t APB2CLKDivider);
    void initRTC ();
    void initI2S (uint32_t PLLI2SN, uint32_t PLLI2SR);

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

    void start (uint32_t fLatency, int32_t msAdjustment = 0);
    void stop ();

private:

    static System * instance;
    RCC_OscInitTypeDef oscParams;
    RCC_ClkInitTypeDef clkParams;
    RCC_PeriphCLKInitTypeDef periphClkParams;
    Interrupt sysTickIrq;
    uint32_t mcuFreq;
};


} // end namespace
#endif

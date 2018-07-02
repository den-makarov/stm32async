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

#ifndef HARDWARE_LAYOUT_H_
#define HARDWARE_LAYOUT_H_

#include <utility>

//TODO: take following stuff away in order to
//      simplify client`s specific controller

#if defined(STM32F4)
    #include "stm32f4xx.h"
#elif defined(STM32F100xB)
    #include "stm32f100xb.h"
#elif defined(STM32F100xE)
    #include "stm32f100xe.h"
#elif defined(STM32F101x6)
    #include "stm32f101x6.h"
#elif defined(STM32F101xB)
    #include "stm32f101xb.h"
#elif defined(STM32F101xE)
    #include "stm32f101xe.h"
#elif defined(STM32F101xG)
    #include "stm32f101xg.h"
#elif defined(STM32F102x6)
    #include "stm32f102x6.h"
#elif defined(STM32F102xB)
    #include "stm32f102xb.h"
#elif defined(STM32F103x6)
    #include "stm32f103x6.h"
#elif defined(STM32F103xB)
    #include "stm32f103xb.h"
#elif defined(STM32F103xE)
    #include "stm32f103xe.h"
#elif defined(STM32F103xG)
    #include "stm32f103xg.h"
#elif defined(STM32F105xC)
    #include "stm32f105xc.h"
#elif defined(STM32F107xC)
    #include "stm32f107xc.h"
#else
    #error "Please select first the target STM32Fxxx device used in your application (in stm32fxxx.h file)"
#endif

namespace HardwareLayout
{

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

    Interrupt (IRQn_Type _irqn, uint32_t _prio, uint32_t _subPrio) :
        irqn { _irqn },
        prio { _prio },
        subPrio { _subPrio }
    {
        // empty
    }

    Interrupt (Interrupt && irq) :
        irqn { irq.irqn },
        prio { irq.prio },
        subPrio { irq.subPrio }
    {
        // empty
    }

    inline void enable () const
    {
        HAL_NVIC_SetPriority(irqn, prio, subPrio);
        HAL_NVIC_EnableIRQ(irqn);
    }

    inline void disable () const
    {
        HAL_NVIC_DisableIRQ(irqn);
    }
};

/**
 * @brief Class provides an interface for any HAL device that have a clock.
 */
class HalDevice
{
public:

    size_t id;

    HalDevice (size_t _id) :
        id { _id }
    {
        // empty
    }

    virtual ~HalDevice ()
    {
        // empty
    }

    virtual void enableClock () const =0;
    virtual void disableClock () const =0;
};

/**
 * @brief Class provides an interface for any HAL device that have a shared clock
 *        (i.e the clock is used by more than one device)
 */
class HalSharedDevice : public HalDevice
{
public:

    HalSharedDevice (size_t _id) :
        HalDevice { _id },
        occupations { 0 }
    {
        // empty
    }

    virtual void enableClock () const
    {
        onClockEnable();
        ++objectsCount;
    }

    virtual void disableClock () const
    {
        if (!isUsed())
        {
            return;
        }
        else
        {
            --objectsCount;
            if(!isUsed())
            {
                onClockDisable();
            }
        }
    }

protected:

    /**
     * @brief Counter: how many clients currently use
     * @brief this instance with common clocking domain
     */
    mutable size_t objectsCount;

    inline bool isUsed () const
    {
        return objectsCount > 0;
    }

    virtual void onClockEnable () const =0;
    virtual void onClockDisable () const =0;
};

/**
 * @brief Parameters of a port.
 */
class Port : public HalSharedDevice
{
public:

    /**
     * @brief General Purpose I/O
     */
    GPIO_TypeDef * instance;

    explicit Port (size_t _id, GPIO_TypeDef * _instance) :
        HalSharedDevice { _id },
        instance { _instance }
    {
        // empty
    }
};

} // end namespace
#endif

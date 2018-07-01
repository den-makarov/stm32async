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

#ifdef STM32F4
#include "stm32f4xx.h"
#endif

#include <utility>

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

    inline bool isUsed () const
    {
        return occupations > 0;
    }

    virtual void enableClock () const
    {
        onClockEnable();
        ++occupations;
    }

    virtual void disableClock () const
    {
        if (occupations > 0)
        {
            --occupations;
        }
        if (!isUsed())
        {
            onClockDisable();
        }
    }

protected:

    /**
     * @brief Counter: how many devices currently use this port
     */
    mutable size_t occupations;

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

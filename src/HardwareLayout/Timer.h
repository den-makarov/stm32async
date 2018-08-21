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

#ifndef HARDWARE_LAYOUT_TIMER_H_
#define HARDWARE_LAYOUT_TIMER_H_

#include "HardwareLayout.h"

#ifdef HAL_TIM_MODULE_ENABLED

namespace Stm32async
{
namespace HardwareLayout
{

/**
 * @brief Parameters of a timer.
 */
class Timer : public HalDevice
{
    DECLARE_INSTANCE(TIM_TypeDef)

public:

    /**
     * @brief Interrupt Number Definition
     */
    Interrupt timerIrq;

    explicit Timer (size_t _id, TIM_TypeDef * _instance, Interrupt && _timerIrq) :
        HalDevice { _id },
        instance { _instance },
        timerIrq { std::move(_timerIrq) }
    {
        // empty
    }

    /**
     * @brief Helper method used to enable all interrupts for the module
     */
    void enableIrq () const
    {
        timerIrq.enable();
    }

    /**
     * @brief Helper method used to disable all interrupts for the module
     */
    void disableIrq () const
    {
        timerIrq.disable();
    }
};

} // end of namespace HardwareLayout
} // end of namespace Stm32async

#endif
#endif

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

#ifndef HARDWARE_LAYOUT_GPIOA_H_
#define HARDWARE_LAYOUT_GPIOA_H_

#include "HardwareLayout.h"

#ifdef GPIOA

namespace Stm32async
{
namespace HardwareLayout
{

/**
 * @brief Wrapper class for GPIOA module.
 *
 * Implementation shall provide wrappers for GPIOA clock enable/disable macros.
 */
class PortA : public HardwareLayout::Port
{
public:
    PortA () :
        Port { 0, GPIOA }
    {
        // empty
    }
    virtual void onClockEnable () const
    {
        __HAL_RCC_GPIOA_CLK_ENABLE();
    }
    virtual void onClockDisable () const
    {
        __HAL_RCC_GPIOA_CLK_DISABLE();
    }
};

} // end of namespace HardwareLayout
} // end of namespace Stm32async

#endif
#endif

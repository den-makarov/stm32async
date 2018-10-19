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

#ifndef HARDWARE_LAYOUT_DAC1_H_
#define HARDWARE_LAYOUT_DAC1_H_

#include "Dac.h"

#ifdef HAL_DAC_MODULE_ENABLED
#ifdef DAC1

namespace Stm32async
{
namespace HardwareLayout
{

/**
 * @brief Wrapper class for DAC1 module.
 */
class Dac1 : public HardwareLayout::Dac
{
public:
    Dac1 (HardwareLayout::Port & _Port, uint32_t _Pin) :
        Dac { 1, DAC1, _Port, _Pin }
    {
        // empty
    }

    virtual void enableClock () const
    {
        __HAL_RCC_DAC_CLK_ENABLE();
    }

    virtual void disableClock () const
    {
        __HAL_RCC_DAC_CLK_DISABLE();
    }
};

} // end of namespace HardwareLayout
} // end of namespace Stm32async

#endif
#endif
#endif

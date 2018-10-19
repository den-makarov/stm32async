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

#ifndef HARDWARE_LAYOUT_AFIO_H_
#define HARDWARE_LAYOUT_AFIO_H_

#include "HardwareLayout.h"

#ifdef AFIO

namespace Stm32async
{
namespace HardwareLayout
{

/**
 * @brief Wrapper class for AFIO peripheral module.
 *
 * Implementation shall provide wrappers for AFIO clock enable/disable macros.
 * This class is only suitable for STM32F1 series.
 */
class AfioModule : public HardwareLayout::Afio
{
public:
    AfioModule () :
        Afio { 0, AFIO }
    {
        // empty
    }
    virtual void onClockEnable () const
    {
        __HAL_RCC_AFIO_CLK_ENABLE();
    }
    virtual void onClockDisable () const
    {
        __HAL_RCC_AFIO_CLK_DISABLE();
    }
};

} // end of namespace HardwareLayout
} // end of namespace Stm32async

#endif /* AFIO */
#endif /* HARDWARE_LAYOUT_AFIO_H_ */

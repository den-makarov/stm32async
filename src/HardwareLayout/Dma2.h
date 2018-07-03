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

#ifndef HARDWARE_LAYOUT_DMA2_H_
#define HARDWARE_LAYOUT_DMA2_H_

#include "../HardwareLayout.h"

#ifdef DMA2

namespace HardwareLayout
{

class Dma2 : public HardwareLayout::Dma
{
public:
    explicit Dma2 () :
        Dma { 2, DMA2 }
    {
        // empty
    }
    virtual void onClockEnable () const
    {
        __HAL_RCC_DMA2_CLK_ENABLE();
    }
    virtual void onClockDisable () const
    {
        __HAL_RCC_DMA2_CLK_DISABLE();
    }
};

} // end namespace
#endif
#endif
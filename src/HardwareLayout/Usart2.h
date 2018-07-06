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

#ifndef HARDWARE_LAYOUT_USART2_H_
#define HARDWARE_LAYOUT_USART2_H_

#include "../HardwareLayout.h"

#ifdef USART2

namespace HardwareLayout
{

class Usart2 : public HardwareLayout::Usart
{
public:
    explicit Usart2 (HardwareLayout::Port & txPort, uint32_t txPin,
                     HardwareLayout::Port & rxPort, uint32_t rxPin,
                     bool remapped,
                     HardwareLayout::Interrupt && txRxIrq,
                     HardwareLayout::DmaStream && txDma,
                     HardwareLayout::Interrupt && txDmaIrq,
                     HardwareLayout::DmaStream && rxDma,
                     HardwareLayout::Interrupt && rxDmaIrq) :
        Usart { 2, USART2, txPort, txPin, rxPort, rxPin, remapped,
                std::move(txRxIrq),
                std::move(txDma), std::move(txDmaIrq),
                std::move(rxDma), std::move(rxDmaIrq) }
    {
        // empty
    }
    virtual void enableClock () const
    {
        __HAL_RCC_USART2_CLK_ENABLE();
    }

    virtual void disableClock () const
    {
        __HAL_RCC_USART2_CLK_DISABLE();
    }

    virtual void remapPins (GPIO_InitTypeDef & gpioParameters) const
    {
        #if defined(STM32F4)
        gpioParameters.Alternate = GPIO_AF7_USART2;
        #elif defined(STM32F1)
        UNUSED(gpioParameters);
        __HAL_RCC_AFIO_CLK_ENABLE();
        __HAL_AFIO_REMAP_USART2_ENABLE();
        #endif
    }
};

} // end namespace
#endif
#endif

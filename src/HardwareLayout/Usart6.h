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

#ifndef HARDWARE_LAYOUT_USART6_H_
#define HARDWARE_LAYOUT_USART6_H_

#include "Usart.h"

#ifdef HAL_UART_MODULE_ENABLED
#ifdef USART6

namespace Stm32async
{
namespace HardwareLayout
{

/**
 * @brief Wrapper class for USART6 module.
 *
 * Implementation shall provide wrappers for USART6 clock enable/disable macros,
 * and platform-dependent functions remapPins() and unremapPins();
 */
class Usart6 : public HardwareLayout::Usart
{
public:
    Usart6 (HardwareLayout::Port & txPort, uint32_t txPin, HardwareLayout::Port & rxPort,
                     uint32_t rxPin, bool _remapped, HardwareLayout::Afio * _afio,
                     HardwareLayout::Interrupt && txRxIrq, HardwareLayout::DmaStream && txDma,
                     HardwareLayout::DmaStream && rxDma) :
        Usart { 6, USART6, txPort, txPin, rxPort, rxPin, _remapped, _afio, std::move(txRxIrq),
                std::move(txDma), std::move(rxDma) }
    {
        // empty
    }
    virtual void enableClock () const
    {
        __HAL_RCC_USART6_CLK_ENABLE();
    }

    virtual void disableClock () const
    {
        __HAL_RCC_USART6_CLK_DISABLE();
    }

    virtual void remapPins (GPIO_InitTypeDef & gpioParameters) const
    {
        if (remapped)
        {
            gpioParameters.Alternate = GPIO_AF8_USART6;
        }
    }
};

} // end of namespace HardwareLayout
} // end of namespace Stm32async

#endif
#endif
#endif

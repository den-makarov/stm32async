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

#ifndef HARDWARE_LAYOUT_SPI2_H_
#define HARDWARE_LAYOUT_SPI2_H_

#include "Spi.h"

#ifdef HAL_SPI_MODULE_ENABLED
#ifdef SPI2

namespace Stm32async
{
namespace HardwareLayout
{

/**
 * @brief Wrapper class for SPI2 module.
 *
 * Implementation shall provide wrappers for SPI2 clock enable/disable macros,
 * and platform-dependent functions remapPins() and unremapPins();
 */
class Spi2 : public HardwareLayout::Spi
{
public:
    Spi2 (HardwareLayout::Port & _sclkPort, uint32_t _sclkPin,
                   HardwareLayout::Port & _mosiPort, uint32_t _mosiPin,
                   HardwareLayout::Port & _misoPort, uint32_t _misoPin, bool _remapped,
                   HardwareLayout::Afio * _afio, HardwareLayout::Interrupt && txRxIrq,
                   HardwareLayout::DmaStream && txDma, HardwareLayout::DmaStream && rxDma) :
        Spi { 2, SPI2, _sclkPort, _sclkPin, _mosiPort, _mosiPin, _misoPort, _misoPin, _remapped,
              _afio, std::move(txRxIrq), std::move(txDma), std::move(rxDma) }
    {
        // empty
    }

    virtual void enableClock () const
    {
        __HAL_RCC_SPI2_CLK_ENABLE();
    }

    virtual void disableClock () const
    {
        __HAL_RCC_SPI2_CLK_DISABLE();
    }

    virtual void remapPins (GPIO_InitTypeDef & gpioParameters) const
    {
        if (remapped)
        {
            #if defined(STM32F4)
                gpioParameters.Alternate = GPIO_AF5_SPI2;
            #elif defined(STM32F1)
                UNUSED(gpioParameters);
                __HAL_AFIO_REMAP_SPI2_ENABLE();
            #endif
        }
    }

    virtual void unremapPins (GPIO_InitTypeDef & gpioParameters) const
    {
        if (remapped)
        {
            UNUSED(gpioParameters);
            #if defined(STM32F1)
                __HAL_AFIO_REMAP_SPI2_DISABLE();
            #endif
        }
    }
};

} // end of namespace HardwareLayout
} // end of namespace Stm32async

#endif
#endif
#endif

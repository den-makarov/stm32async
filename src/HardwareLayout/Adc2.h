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

#ifndef HARDWARE_LAYOUT_ADC2_H_
#define HARDWARE_LAYOUT_ADC2_H_

#include "Adc.h"

#ifdef HAL_ADC_MODULE_ENABLED
#ifdef ADC2

namespace Stm32async
{
namespace HardwareLayout
{

/**
 * @brief Wrapper class for ADC2 module.
 *
 * Implementation shall provide wrappers for ADC2 clock enable/disable macros,
 * and platform-dependent functions remapPins() and unremapPins();
 */
class Adc2 : public HardwareLayout::Adc
{
public:
    Adc2 (HardwareLayout::Port & _Port, uint32_t _Pin,
                   bool _remapped, HardwareLayout::Afio * _afio,
                   HardwareLayout::DmaStream && rxDma) :
        Adc { 2, ADC2, _Port, _Pin, _remapped, _afio, std::move(rxDma) }
    {
        // empty
    }

    virtual void enableClock () const
    {
        __HAL_RCC_ADC2_CLK_ENABLE();
    }

    virtual void disableClock () const
    {
        __HAL_RCC_ADC2_CLK_DISABLE();
    }

    virtual void remapPins (GPIO_InitTypeDef & gpioParameters) const
    {
        if (remapped)
        {
            UNUSED(gpioParameters);
            #if defined(STM32F1)
                __HAL_AFIO_REMAP_ADC2_ENABLE();
            #endif
        }
    }

    virtual void unremapPins (GPIO_InitTypeDef & gpioParameters) const
    {
        if (remapped)
        {
            UNUSED(gpioParameters);
            #if defined(STM32F1)
                __HAL_AFIO_REMAP_ADC2_DISABLE();
            #endif
        }
    }
};

} // end of namespace HardwareLayout
} // end of namespace Stm32async

#endif
#endif
#endif

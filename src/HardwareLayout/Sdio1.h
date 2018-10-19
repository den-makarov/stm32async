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

#ifndef HARDWARE_LAYOUT_SDIO1_H_
#define HARDWARE_LAYOUT_SDIO1_H_

#include "Sdio.h"

#ifdef HAL_SD_MODULE_ENABLED

namespace Stm32async
{
namespace HardwareLayout
{

/**
 * @brief Wrapper class for SDIO1 module.
 *
 * Implementation shall provide wrappers for SDIO1 clock enable/disable macros,
 * and platform-dependent functions remapPins() and unremapPins();
 */
class Sdio1 : public HardwareLayout::Sdio
{
public:
    Sdio1 (HardwareLayout::Port & port1, uint32_t port1pins,
                    HardwareLayout::Port & port2, uint32_t port2pins,
                    HardwareLayout::Interrupt && txRxIrq,
                    HardwareLayout::DmaStream && txDma, HardwareLayout::DmaStream && rxDma) :
        Sdio { 1, SDIO, port1, port1pins, port2, port2pins,
               true, NULL,
               std::move(txRxIrq), std::move(txDma), std::move(rxDma) }
    {
        // empty
    }

    virtual void enableClock () const
    {
        __HAL_RCC_SDIO_CLK_ENABLE();
    }

    virtual void disableClock () const
    {
        __HAL_RCC_SDIO_CLK_DISABLE();
    }

    virtual void remapPins (GPIO_InitTypeDef & gpioParameters) const
    {
        if (remapped)
        {
            gpioParameters.Alternate = GPIO_AF12_SDIO;
        }
    }
};

} // end of namespace HardwareLayout
} // end of namespace Stm32async

#endif
#endif

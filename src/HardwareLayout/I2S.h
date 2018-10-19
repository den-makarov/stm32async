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

#ifndef HARDWARE_LAYOUT_I2S_H_
#define HARDWARE_LAYOUT_I2S_H_

#include "HardwareLayout.h"

#ifdef HAL_I2S_MODULE_ENABLED

namespace Stm32async
{
namespace HardwareLayout
{

/**
 * @brief Configuration of Serial Peripheral Interface (I2S instance).
 */
class I2S : public HalAfioDevice
{
    DECLARE_INSTANCE(SPI_TypeDef)

public:

    /**
     * @brief Pins from corresponding ports
     */
    Pins pins;

    /**
     * @brief TX DMA channel
     */
    DmaStream txDma;

    /**
     * @brief RX DMA channel
     */
    DmaStream rxDma;

    /**
     * @brief Standard initialization constructor.
     */
    I2S (size_t _id,  SPI_TypeDef *_instance,
                  Port & _port, uint32_t _pins,
                  bool _remapped, Afio * _afio,
                  DmaStream && _txDma, DmaStream && _rxDma) :
        HalAfioDevice { _id, _remapped, _afio },
        instance { _instance },
        pins { _port, _pins },
        txDma { std::move(_txDma) },
        rxDma { std::move(_rxDma) }
    {
        // empty
    }

    /**
     * @brief Helper method used to enable all interrupts for the module
     */
    void enableIrq () const
    {
        txDma.dmaIrq.enable();
        rxDma.dmaIrq.enable();
    }

    /**
     * @brief Helper method used to disable all interrupts for the module
     */
    void disableIrq () const
    {
        txDma.dmaIrq.disable();
        rxDma.dmaIrq.disable();
    }
};

} // end of namespace HardwareLayout
} // end of namespace Stm32async

#endif
#endif

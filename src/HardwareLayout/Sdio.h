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

#ifndef HARDWARE_LAYOUT_SDIO_H_
#define HARDWARE_LAYOUT_SDIO_H_

#include "HardwareLayout.h"

#ifdef HAL_SD_MODULE_ENABLED

namespace Stm32async
{
namespace HardwareLayout
{

/**
 * @brief Configuration of SDIO device.
 */
class Sdio : public HalAfioDevice
{
DECLARE_INSTANCE(SD_TypeDef)

public:

    /**
     * @brief Pins from two ports
     */
    Pins pins1, pins2;

    /**
     * @brief SDIO global interrupt configuration
     */
    Interrupt txRxIrq;

    /**
     * @brief TX DMA channel
     */
    DmaStream txDma;

    /**
     * @brief RX DMA channel
     */
    DmaStream rxDma;

    Sdio (size_t _id,  SD_TypeDef *_instance,
                   Port & _port1, uint32_t _port1pins,
                   Port & _port2, uint32_t _port2pins,
                   bool _remapped, Afio * _afio,
                   Interrupt && _txRxIrq,
                   DmaStream && _txDma, DmaStream && _rxDma) :
        HalAfioDevice { _id, _remapped, _afio },
        instance { _instance },
        pins1 { _port1, _port1pins },
        pins2 { _port2, _port2pins },
        txRxIrq { std::move(_txRxIrq) },
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
        txRxIrq.enable();
        txDma.dmaIrq.enable();
        rxDma.dmaIrq.enable();
    }

    /**
     * @brief Helper method used to disable all interrupts for the module
     */
    void disableIrq () const
    {
        txRxIrq.disable();
        txDma.dmaIrq.disable();
        rxDma.dmaIrq.disable();
    }
};

} // end of namespace HardwareLayout
} // end of namespace Stm32async

#endif
#endif

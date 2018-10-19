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

#ifndef HARDWARE_LAYOUT_SPI_H_
#define HARDWARE_LAYOUT_SPI_H_

#include "HardwareLayout.h"

#ifdef HAL_SPI_MODULE_ENABLED

namespace Stm32async
{
namespace HardwareLayout
{

/**
 * @brief Configuration of Serial Peripheral Interface.
 */
class Spi : public HalAfioDevice
{
    DECLARE_INSTANCE(SPI_TypeDef)

public:

    /**
     * @brief Serial clock pin: output from master
     */
    Pins sclkPin;

    /**
     * @brief Master Out Slave In (MOSI, data output from master) pin
     */
    Pins mosiPin;

    /**
     * @brief Master In Slave Out (MISO, data output from slave) pin
     */
    Pins misoPin;

    /**
     * @brief USART global interrupt configuration
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

    /**
     * @brief Standard initialization constructor.
     *
     * @param _id a numerical device ID used for logging purpose.
     * @param _instance pointer to the HAL USART definition structure.
     * @param _sclkPort the port of serial clock line.
     * @param _sclkPin the pin of serial clock line.
     * @param _mosiPort the port of MOSI line.
     * @param _mosiPin the pin of MOSI line.
     * @param _misoPort the port of MISO line.
     * @param _misoPin the pin of MISO line.
     * @param _remapped flag indicating whether the TX/RX pins shall be remapped.
     * @param _afio pointer to the AFIO module if it necessary for remapping.
     * @param _txRxIrq link to the USART global interrupt.
     * @param _txDma link to the transmitter DMA stream.
     * @param _rxDma to the receiver DMA stream.
     */
    Spi (size_t _id,  SPI_TypeDef *_instance,
                  Port & _sclkPort, uint32_t _sclkPin,
                  Port & _mosiPort, uint32_t _mosiPin,
                  Port & _misoPort, uint32_t _misoPin,
                  bool _remapped, Afio * _afio,
                  Interrupt && _txRxIrq,
                  DmaStream && _txDma, DmaStream && _rxDma) :
        HalAfioDevice { _id, _remapped, _afio },
        instance { _instance },
        sclkPin { _sclkPort, _sclkPin },
        mosiPin { _mosiPort, _mosiPin },
        misoPin { _misoPort, _misoPin },
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

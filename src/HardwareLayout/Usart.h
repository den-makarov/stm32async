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

#ifndef HARDWARE_LAYOUT_USART_H_
#define HARDWARE_LAYOUT_USART_H_

#include "HardwareLayout.h"

#ifdef HAL_UART_MODULE_ENABLED

namespace Stm32async
{
namespace HardwareLayout
{

/**
 * @brief Configuration of Universal Synchronous Asynchronous Receiver Transmitter (USART).
 *
 * This class holds the configuration of USART module: USART definition structure, used TX/RX pins,
 * optional link to AFIO module used to remap TX/RX pins, configuration of USART global interrupt,
 * configuration of TX/RX DMA channels with corresponding interrupts.
 *
 * Since USART is able to use alternative pins, the derived class shall implement virtual
 * remapPins() and unremapPins() methods from the base class.
 */
class Usart : public HalAfioDevice
{
    DECLARE_INSTANCE(USART_TypeDef)

public:

    /**
     * @brief TX pin
     */
    Pins txPin;

    /**
     * @brief RX pin
     */
    Pins rxPin;

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
     * @param _txPort the port of transmitter line.
     * @param _txPin the pin of transmitter line.
     * @param _rxPort the port of receiver line.
     * @param _rxPin the pin of receiver line.
     * @param _remapped flag indicating whether the TX/RX pins shall be remapped.
     * @param _afio pointer to the AFIO module if it necessary for remapping.
     * @param _txRxIrq link to the USART global interrupt.
     * @param _txDma link to the transmitter DMA stream.
     * @param _rxDma to the receiver DMA stream.
     */
    Usart (size_t _id,  USART_TypeDef *_instance,
                    Port & _txPort, uint32_t _txPin, Port & _rxPort, uint32_t _rxPin,
                    bool _remapped, Afio * _afio,
                    Interrupt && _txRxIrq,
                    DmaStream && _txDma, DmaStream && _rxDma) :
        HalAfioDevice { _id, _remapped, _afio },
        instance { _instance },
        txPin { _txPort, _txPin },
        rxPin { _rxPort, _rxPin },
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

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

#ifndef STM32ASYNC_ASYNCSPI_H_
#define STM32ASYNC_ASYNCSPI_H_

#include "BaseSpi.h"
#include "SharedDevice.h"

namespace Stm32async
{

/**
 * @brief Class that implements SPI interface.
 */
class AsyncSpi : public BaseSpi, public SharedDevice
{
public:

    /**
     * @brief Default constructor.
     */
    AsyncSpi (const HardwareLayout::Spi & _device);

    /**
     * @brief Open transmission session with given parameters.
     */
    DeviceStart::Status start (uint32_t direction, uint32_t prescaler,
                               uint32_t dataSize = SPI_DATASIZE_8BIT,
                               uint32_t CLKPhase = SPI_PHASE_1EDGE);

    /**
     * @brief Close the transmission session.
     */
    void stop ();

    /**
     * @brief Send an amount of data in DMA mode.
     */
    inline HAL_StatusTypeDef transmit (DeviceClient * _client, uint8_t * buffer, uint16_t n)
    {
        startCommunication(_client, State::TX, State::TX_CMPL);
        halStatus = HAL_SPI_Transmit_DMA(&parameters, buffer, n);
        return halStatus;
    }

    /**
     * @brief Send an amount of data in interrupt mode.
     */
    inline HAL_StatusTypeDef transmitIt (DeviceClient * _client, uint8_t * buffer, uint16_t n)
    {
        startCommunication(_client, State::TX, State::TX_CMPL);
        halStatus = HAL_SPI_Transmit_IT(&parameters, buffer, n);
        return halStatus;
    }

    /**
     * @brief Receive an amount of data in DMA mode.
     */
    inline HAL_StatusTypeDef receive (DeviceClient * _client, uint8_t * buffer, uint16_t n)
    {
        startCommunication(_client, State::RX, State::RX_CMPL);
        halStatus = HAL_SPI_Receive_DMA(&parameters, buffer, n);
        return halStatus;
    }

    /**
     * @brief Receive an amount of data in interrupt mode.
     */
    inline HAL_StatusTypeDef receiveIt (DeviceClient * _client, uint8_t * buffer, uint16_t n)
    {
        startCommunication(_client, State::RX, State::RX_CMPL);
        halStatus = HAL_SPI_Receive_IT(&parameters, buffer, n);
        return halStatus;
    }

    /**
     * @brief Interrupt handling.
     */
    inline void processInterrupt ()
    {
        HAL_SPI_IRQHandler(&parameters);
    }
};

} // end namespace
#endif


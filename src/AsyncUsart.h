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

#ifndef STM32ASYNC_ASYNCUSART_H_
#define STM32ASYNC_ASYNCUSART_H_

#include "BaseUsart.h"
#include "SharedDevice.h"

namespace Stm32async
{

/**
 * @brief Class that implements UART interface.
 */
class AsyncUsart : public BaseUsart, public SharedDevice
{
public:

    /**
     * @brief Default constructor.
     */
    AsyncUsart (const HardwareLayout::Usart & _device);

    /**
     * @brief Open transmission session with given parameters.
     */
    HAL_StatusTypeDef start (uint32_t mode, uint32_t baudRate,
                             uint32_t wordLength = UART_WORDLENGTH_8B,
                             uint32_t stopBits = UART_STOPBITS_1,
                             uint32_t parity = UART_PARITY_NONE);

    /**
     * @brief Close the transmission session.
     */
    void stop ();

    /**
     * @brief Send an amount of data in DMA mode.
     */
    inline HAL_StatusTypeDef transmit (DeviceClient * _client, const char * buffer, size_t n)
    {
        startCommunication(_client, State::TX, State::TX_CMPL);
        return HAL_UART_Transmit_DMA(&parameters, (unsigned char *) buffer, n);
    }

    /**
     * @brief Send an amount of data in interrupt mode.
     */
    inline HAL_StatusTypeDef transmitIt (DeviceClient * _client, const char * buffer, size_t n)
    {
        startCommunication(_client, State::TX, State::TX_CMPL);
        return HAL_UART_Transmit_IT(&parameters, (unsigned char *) buffer, n);
    }

    /**
     * @brief Receive an amount of data in DMA mode.
     */
    inline HAL_StatusTypeDef receive (DeviceClient * _client, const char * buffer, size_t n)
    {
        startCommunication(_client, State::RX, State::RX_CMPL);
        return HAL_UART_Receive_DMA(&parameters, (unsigned char *) buffer, n);
    }

    /**
     * @brief Receive an amount of data in interrupt mode.
     */
    inline HAL_StatusTypeDef receiveIt (DeviceClient * _client, const char * buffer, size_t n)
    {
        startCommunication(_client, State::RX, State::RX_CMPL);
        return HAL_UART_Receive_IT(&parameters, (unsigned char *) buffer, n);
    }

    /**
     * @brief Interrupt handling.
     */
    inline void processInterrupt ()
    {
        HAL_UART_IRQHandler(&parameters);
    }
};

} // end namespace
#endif


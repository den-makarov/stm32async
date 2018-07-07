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

#include "HardwareLayout.h"
#include "IOPort.h"

#ifndef STM32ASYNC_BASEUSART_H_
#define STM32ASYNC_BASEUSART_H_

namespace Stm32async
{

/**
 * @brief Base USART class that holds the USART parameters and implements the communication in
 *        a blocking mode.
 */
class BaseUsart
{
public:

    /**
     * @brief Default constructor.
     */
    BaseUsart (const HardwareLayout::Usart & _device);

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
     * @brief Send an amount of data in blocking mode.
     */
    inline HAL_StatusTypeDef transmitBlocking (const char * buffer, size_t n,
                                               uint32_t timeout = __UINT32_MAX__)
    {
        return HAL_UART_Transmit(&parameters, (unsigned char *) buffer, n, timeout);
    }

    /**
     * @brief Receive an amount of data in blocking mode.
     */
    inline HAL_StatusTypeDef receiveBlocking (const char * buffer, size_t n,
                                              uint32_t timeout = __UINT32_MAX__)
    {
        return HAL_UART_Receive(&parameters, (unsigned char *) buffer, n, timeout);
    }

    /**
     * @brief Procedure waits in a blocking mode until USART is ready.
     */
    inline void ensureReady ()
    {
        while (HAL_UART_GetState(&parameters) != HAL_UART_STATE_READY);
    }

protected:

    const HardwareLayout::Usart & device;
    IOPort txPin, rxPin;
    UART_HandleTypeDef parameters;
};

} // end namespace
#endif

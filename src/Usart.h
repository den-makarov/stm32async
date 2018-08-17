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

#ifndef STM32ASYNC_USART_H_
#define STM32ASYNC_USART_H_

#include "HardwareLayout/Usart.h"

#ifdef HAL_UART_MODULE_ENABLED

#include "IODevice.h"
#include "SharedDevice.h"

namespace Stm32async
{

/**
 * @brief Base USART class that holds the USART parameters and implements the communication in
 *        a blocking mode.
 */
class BaseUsart : public IODevice<HardwareLayout::Usart, UART_HandleTypeDef, 2>
{
public:

    /**
     * @brief Default constructor.
     */
    BaseUsart (const HardwareLayout::Usart & _device);

    /**
     * @brief Open transmission session with given parameters.
     */
    DeviceStart::Status start (uint32_t mode, uint32_t baudRate,
                               uint32_t wordLength = UART_WORDLENGTH_8B,
                               uint32_t stopBits = UART_STOPBITS_1,
                               uint32_t parity = UART_PARITY_NONE);

    /**
     * @brief Open transmission session with given mode.
     */
    inline HAL_StatusTypeDef changeMode (uint32_t mode)
    {
        parameters.Init.Mode = mode;
        halStatus = HAL_UART_Init(&parameters);
        return halStatus;
    }

    /**
     * @brief Close the transmission session.
     */
    void stop ();

    /**
     * @brief Returns pointer to the TX pin.
     */
    inline IOPort * getTxPin ()
    {
        return &(ports[0]);
    }

   /**
    * @brief Returns pointer to the RX pin.
    */
   inline IOPort * getRxPin ()
   {
       return &(ports[1]);
   }

    /**
     * @brief Send an amount of data in blocking mode.
     */
    inline HAL_StatusTypeDef transmitBlocking (const char * buffer, size_t n,
                                               uint32_t timeout = __UINT32_MAX__)
    {
        halStatus = HAL_UART_Transmit(&parameters, (unsigned char *) buffer, n, timeout);
        return halStatus;
    }

    /**
     * @brief Receive an amount of data in blocking mode.
     */
    inline HAL_StatusTypeDef receiveBlocking (const char * buffer, size_t n,
                                              uint32_t timeout = __UINT32_MAX__)
    {
        halStatus = HAL_UART_Receive(&parameters, (unsigned char *) buffer, n, timeout);
        return halStatus;
    }

    /**
     * @brief Procedure waits in a blocking mode until USART is ready.
     */
    inline void ensureReady ()
    {
        while (HAL_UART_GetState(&parameters) != HAL_UART_STATE_READY);
    }
};


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
    DeviceStart::Status start (uint32_t mode, uint32_t baudRate,
                               uint32_t wordLength = UART_WORDLENGTH_8B,
                               uint32_t stopBits = UART_STOPBITS_1,
                               uint32_t parity = UART_PARITY_NONE,
                               bool isIrqEnabled = true);

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
        halStatus = HAL_UART_Transmit_DMA(&parameters, (unsigned char *) buffer, n);
        return halStatus;
    }

    /**
     * @brief Send an amount of data in interrupt mode.
     */
    inline HAL_StatusTypeDef transmitIt (DeviceClient * _client, const char * buffer, size_t n)
    {
        startCommunication(_client, State::TX, State::TX_CMPL);
        halStatus = HAL_UART_Transmit_IT(&parameters, (unsigned char *) buffer, n);
        return halStatus;
    }

    /**
     * @brief Receive an amount of data in DMA mode.
     */
    inline HAL_StatusTypeDef receive (DeviceClient * _client, const char * buffer, size_t n)
    {
        startCommunication(_client, State::RX, State::RX_CMPL);
        halStatus = HAL_UART_Receive_DMA(&parameters, (unsigned char *) buffer, n);
        return halStatus;
    }

    /**
     * @brief Receive an amount of data in interrupt mode.
     */
    inline HAL_StatusTypeDef receiveIt (DeviceClient * _client, const char * buffer, size_t n)
    {
        startCommunication(_client, State::RX, State::RX_CMPL);
        halStatus = HAL_UART_Receive_IT(&parameters, (unsigned char *) buffer, n);
        return halStatus;
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
#endif

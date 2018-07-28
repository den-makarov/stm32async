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

#include "AsyncUsart.h"

#ifndef STM32ASYNC_USARTPORT_H
#define STM32ASYNC_USARTPORT_H

namespace Stm32async
{

/**
 * @brief Class implementing USART listening port.
 */
class UsartPort
{
    DECLARE_STATIC_INSTANCE(UsartPort)

public:

    /**
     * @brief Default constructor.
     */
    UsartPort (AsyncUsart & _device, uint32_t _baudRate);

    /**
     * @brief Open listening session on USART with given parameters.
     */
    HAL_StatusTypeDef start (SharedDevice::DeviceClient * _client, uint8_t * buffer, size_t n);

    /**
     * @brief Close the listening session.
     */
    void stop ()
    {
        HAL_UART_Abort_IT(&usart.getParameters());
        usart.stop();
    }

    void initInstance ();

    inline void clearInstance ()
    {
        instance = NULL;
    }

    static UsartPort & getStream ()
    {
        return *instance;
    }

    AsyncUsart & getUsart ()
    {
        return usart;
    }

private:

    AsyncUsart & usart;
    uint32_t baudRate;
    const uint32_t TIMEOUT = 20000;
};

} // end namespace
#endif // STM32ASYNC_USARTPORT_H

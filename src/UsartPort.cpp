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

#include "UsartPort.h"

using namespace Stm32async;

/************************************************************************
 * Class UsartLogger
 ************************************************************************/

UsartPort * UsartPort::instance = nullptr;

UsartPort::UsartPort (AsyncUsart & _device, uint32_t _baudRate) :
    usart { _device },
    baudRate { _baudRate }
{
    //empty
}

HAL_StatusTypeDef UsartPort::start (SharedDevice::DeviceClient * _client, uint8_t * buffer, size_t n)
{
    initInstance();
    usart.setTimeout(TIMEOUT);
    usart.start(UART_MODE_RX, baudRate, UART_WORDLENGTH_8B, UART_STOPBITS_1, UART_PARITY_NONE);
    usart.ensureReady();

    HAL_StatusTypeDef status = HAL_ERROR;
    if (buffer != nullptr && n > 0)
    {
        usart.waitForRelease();
        //status = usart.receiveIt(_client, buffer, n);
        usart.receiveBlocking(buffer, n, TIMEOUT);
    }
    return status;
}

void UsartPort::initInstance ()
{
    instance = this;
}

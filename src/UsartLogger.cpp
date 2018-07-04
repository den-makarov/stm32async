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

#include "UsartLogger.h"

using namespace Stm32async;

/************************************************************************
 * Class UsartLogger
 ************************************************************************/

UsartLogger * UsartLogger::instance = NULL;

UsartLogger::UsartLogger (const HardwareLayout::Usart & _device, uint32_t _baudRate) :
    usart { _device },
    baudRate { _baudRate }
{
    // empty
}

void UsartLogger::initInstance ()
{
    instance = this;
    usart.setTimeout(TIMEOUT);
    usart.start(UART_MODE_TX, baudRate, UART_WORDLENGTH_8B, UART_STOPBITS_1, UART_PARITY_NONE);
    usart.ensureReady();
}

UsartLogger & UsartLogger::operator << (const char * buffer)
{
    size_t bSize = ::strlen(buffer);
    if (bSize > 0)
    {
        usart.waitForRelease();
        usart.transmit(NULL, buffer, bSize);
    }
    return *this;
}

UsartLogger & UsartLogger::operator << (int n)
{
    usart.waitForRelease();
    char buffer[256];
    ::__itoa(n, buffer, 10);
    size_t bSize = ::strlen(buffer);
    if (bSize > 0)
    {
        usart.transmit(NULL, buffer, bSize);
    }
    return *this;
}

UsartLogger & UsartLogger::operator << (Manupulator m)
{
    usart.waitForRelease();
    switch (m)
    {
    case Manupulator::ENDL:
        usart.transmit(NULL, "\n\r", 2);
        break;
    case Manupulator::TAB:
        usart.transmit(NULL, "    ", 4);
        break;
    }
    return *this;
}

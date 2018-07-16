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

#define BLOCKING_TRANSMITION
#undef BLOCKING_TRANSMITION

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
        #ifndef BLOCKING_TRANSMITION
        usart.waitForRelease();
        usart.transmit(NULL, buffer, bSize);
        #else
        usart.transmitBlocking(buffer, bSize);
        #endif
    }
    return *this;
}

UsartLogger & UsartLogger::operator << (int n)
{
    char buffer[256];
    ::__itoa(n, buffer, 10);
    size_t bSize = ::strlen(buffer);
    if (bSize > 0)
    {
        #ifndef BLOCKING_TRANSMITION
        usart.waitForRelease();
        usart.transmit(NULL, buffer, bSize);
        #else
        usart.transmitBlocking(buffer, bSize);
        #endif
    }
    return *this;
}

UsartLogger & UsartLogger::operator << (Manupulator m)
{
    #ifndef BLOCKING_TRANSMITION
    usart.waitForRelease();
    #endif
    switch (m)
    {
    case Manupulator::ENDL:
        #ifndef BLOCKING_TRANSMITION
        usart.transmit(NULL, "\n\r", 2);
        #else
        usart.transmitBlocking("\n\r", 2);
        #endif
        break;
    case Manupulator::TAB:
        #ifndef BLOCKING_TRANSMITION
        usart.transmit(NULL, "    ", 4);
        #else
        usart.transmitBlocking("    ", 4);
        #endif
        break;
    }
    return *this;
}

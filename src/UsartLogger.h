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

#include "Usart.h"

#ifndef STM32ASYNC_USART_LOGGER_H_
#define STM32ASYNC_USART_LOGGER_H_

namespace Stm32async
{

#define IS_USART_DEBUG_ACTIVE() (UsartLogger::getInstance() != NULL)

#define USART_DEBUG(text) {\
    if (IS_USART_DEBUG_ACTIVE())\
    {\
        UsartLogger::getStream() << USART_DEBUG_MODULE << text;\
    }}

/**
 * @brief Class implementing USART logger.
 */
class UsartLogger
{
    DECLARE_STATIC_INSTANCE(UsartLogger)

public:

    const uint32_t TIMEOUT = 1000;

    enum Manupulator
    {
        ENDL = 0,
        TAB = 1
    };

    /**
     * @brief Default constructor.
     */
    UsartLogger (const HardwareLayout::Usart & _device, uint32_t _baudRate);

    void initInstance ();

    inline void clearInstance ()
    {
        usart.waitForRelease();
        usart.stop();
        instance = NULL;
    }

    static UsartLogger & getStream ()
    {
        return *instance;
    }

    AsyncUsart & getUsart ()
    {
        return usart;
    }

    UsartLogger & operator << (const char * buffer);
    UsartLogger & operator << (int n);
    UsartLogger & operator << (Manupulator m);

private:

    AsyncUsart usart;
    uint32_t baudRate;
};

} // end namespace
#endif

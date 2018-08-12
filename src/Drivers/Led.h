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

#ifndef DRIVERS_LED_H_
#define DRIVERS_LED_H_

#include "../IOPort.h"

namespace Stm32async
{
namespace Drivers
{

/** 
 * @brief Class describing a LED element connecting to a pin
 */
class Led: public IOPort
{
public:

    enum class ConnectionType
    {
        ANODE = 0,
        CATHODE = 1
    };

public:

    Led (const HardwareLayout::Port & _port, uint32_t _pin, ConnectionType _connectionType):
        IOPort { _port, _pin, GPIO_MODE_OUTPUT_PP },
        connectionType { _connectionType }
    {
        // empty
    }

    inline void turnOn ()
    {
        (connectionType == ConnectionType::ANODE) ? setHigh() : setLow();
    }

    inline void turnOff ()
    {
        (connectionType == ConnectionType::ANODE) ? setLow() : setHigh();
    };

protected:

    ConnectionType connectionType;

};

} // end of namespace Drivers
} // end of namespace Stm32async

#endif

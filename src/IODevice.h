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

#ifndef STM32ASYNC_IODEVICE_H_
#define STM32ASYNC_IODEVICE_H_

#include "HardwareLayout.h"
#include "IOPort.h"

#include <array>

namespace Stm32async
{

/**
 * @brief A base class that represents IO device working on some IOPort.
 */
template <typename DEVICE, std::size_t PORTS> class IODevice
{
public:

    /**
     * @brief Standard initialization constructor.
     *
     * @param _device some device from namespace HardwareLayout.
     * @param _ports array containing the associated ports.
     */
    explicit IODevice (const DEVICE & _device, std::array<IOPort, PORTS> && _ports) :
        device { _device },
        ports { std::move(_ports) }
    {
        // empty
    }

    /**
     * @brief Getter for the ports registered for this device.
     */
    const std::array<IOPort, PORTS> & getPorts () const
    {
        return ports;
    }

    /**
     * @brief Helper method that activates all ports used by this device.
     */
    void enablePorts ()
    {
        if (device.afio != NULL)
        {
            device.afio->enableClock();
        }
        for (auto & p : ports)
        {
            device.remapPins(p.getParameters());
            p.start();
        }
    }

    /**
     * @brief Helper method that de-activates all ports used by this device.
     */
    void disablePorts ()
    {
        for (auto & p : ports)
        {
            p.stop();
            device.unremapPins(p.getParameters());
        }
        if (device.afio != NULL)
        {
            device.afio->disableClock();
        }
    }

protected:

    const DEVICE & device;
    std::array<IOPort, PORTS> ports;
};

} // end namespace
#endif

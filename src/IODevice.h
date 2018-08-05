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

#include "IOPort.h"

namespace Stm32async
{

/**
 * @brief A class providing the device start-up success/error code
 */
class DeviceStart
{
public:
    /**
     * @brief Set of valid enumeration values
     */
    enum Status
    {
        OK = 0,
        DEVICE_INIT_ERROR        = 1,
        TX_DMA_INIT_ERROR        = 2,
        RX_DMA_INIT_ERROR        = 3,
        SD_NOT_INSERTED          = 4,
        SD_WIDE_BUS_ERROR        = 5,
        SD_READ_STATUS_ERROR     = 6,
        FAT_DRIVER_NOT_LINKED    = 7,
        FAT_VOLUME_NOT_MOUNTED   = 8,
        FAT_VOLUME_STATUS_ERROR  = 9,
        FAT_DIR_STATUS_ERROR     = 10
    };

    /**
     * @brief Number of enumeration values
     */
    enum
    {
        size = 11
    };

    /**
     * @brief String representations of all enumeration values
     */
    static const char * strings[];

    /**
     * @brief the AsString() method
     */
    static AsStringClass<Status, size, strings> asString;
};

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
        ports { std::move(_ports) },
        halStatus { HAL_ERROR }
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
     * @brief Getter for HAL status of the last operation
     */
    inline HAL_StatusTypeDef getHalStatus () const
    {
        return halStatus;
    }

    const DEVICE & getDevice () const
    {
        return device;
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
    HAL_StatusTypeDef halStatus;
};

} // end namespace
#endif

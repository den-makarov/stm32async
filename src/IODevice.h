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
        FAT_DIR_STATUS_ERROR     = 10,
        ADC_CHANNEL_ERROR        = 11,
        TIMER_START_ERROR        = 12
    };

    /**
     * @brief Number of enumeration values
     */
    enum
    {
        size = 13
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

static const uint16_t UNUSED_PIN = 0;

/**
 * @brief A base class that represents IO device working on some IOPort.
 */
template <typename DEVICE, typename PARAMETERS, std::size_t PORTS> class IODevice
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
        halStatus { HAL_ERROR },
        ports { std::move(_ports) }
    {
        // empty
    }

    /**
     * @brief Returns device ID.
     */
    inline size_t getId () const
    {
        return device.getId();
    }

    /**
     * @brief Getter for the device parameters
     */
    inline PARAMETERS & getParameters ()
    {
        return parameters;
    }

    /**
     * @brief Getter for HAL status of the last operation
     */
    inline HAL_StatusTypeDef getHalStatus () const
    {
        return halStatus;
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
            if (p.getParameters().Pin != UNUSED_PIN)
            {
                device.remapPins(p.getParameters());
                p.start();
            }
        }
    }

    /**
     * @brief Helper method that de-activates all ports used by this device.
     */
    void disablePorts ()
    {
        for (auto & p : ports)
        {
            if (p.getParameters().Pin != UNUSED_PIN)
            {
                p.stop();
                device.unremapPins(p.getParameters());
            }
        }
        if (device.afio != NULL)
        {
            device.afio->disableClock();
        }
    }

    /**
     * @brief Disable all related interrupts.
     */
    inline void disableIrq ()
    {
        device.disableIrq();
    }

    /**
     * @brief Enable all related interrupts.
     */
    inline void enableIrq ()
    {
        device.enableIrq();
    }

protected:

    PARAMETERS parameters;
    const DEVICE & device;
    HAL_StatusTypeDef halStatus;
    std::array<IOPort, PORTS> ports;
};

} // end namespace
#endif

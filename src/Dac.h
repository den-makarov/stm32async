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

#ifndef STM32ASYNC_DAC_H_
#define STM32ASYNC_DAC_H_

#include "HardwareLayout/Dac.h"

#ifdef HAL_DAC_MODULE_ENABLED

#include "IODevice.h"

namespace Stm32async
{

/**
 * @brief Base class that implements digital-toanalog converter
 */
class BaseDac : public IODevice<HardwareLayout::Dac, DAC_HandleTypeDef, 1>
{
public:

    /**
     * @brief Default constructor.
     */
    BaseDac (const HardwareLayout::Dac & _device, uint32_t _channel);

    /**
     * @brief Start this DAC device.
     */
    DeviceStart::Status start ();

    /**
     * @brief Stop this DAC device.
     */
    void stop ();

    HAL_StatusTypeDef setValue (uint32_t v);

    /**
     * @brief Procedure returns the parameters of DAC channel.
     */
    inline DAC_ChannelConfTypeDef & getDacChannel ()
    {
        return dacChannel;
    }

protected:

    DAC_ChannelConfTypeDef dacChannel;
    uint32_t channelIdx;
};



} // end namespace
#endif
#endif

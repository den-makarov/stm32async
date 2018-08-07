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

#ifndef STM32ASYNC_I2S_H_
#define STM32ASYNC_I2S_H_

#include "HardwareLayout/I2S.h"

#ifdef HAL_I2S_MODULE_ENABLED

#include "IODevice.h"
#include "SharedDevice.h"

namespace Stm32async
{

/**
 * @brief Class that implements I2S interface
 */
class AsyncI2S : public IODevice<HardwareLayout::I2S, 1>, public SharedDevice
{
public:

    AsyncI2S (const HardwareLayout::I2S & _device);

    DeviceStart::Status start (uint32_t standard, uint32_t audioFreq, uint32_t dataFormat);
    void stop ();

    inline I2S_HandleTypeDef & getParameters ()
    {
        return parameters;
    }

    inline HAL_StatusTypeDef transmit (DeviceClient * _client, uint16_t * pData, uint16_t size)
    {
        startCommunication(_client, State::TX, State::TX_CMPL);
        return HAL_I2S_Transmit_DMA(&parameters, pData, size);
    }

private:

    I2S_HandleTypeDef parameters;
};

} // end namespace

#endif
#endif

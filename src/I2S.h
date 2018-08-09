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
class AsyncI2S : public IODevice<HardwareLayout::I2S, I2S_HandleTypeDef, 1>, public SharedDevice
{
public:

    /**
     * @brief Default constructor.
     */
    AsyncI2S (const HardwareLayout::I2S & _device);

    /**
     * @brief Open transmission session with given parameters.
     */
    DeviceStart::Status start (uint32_t standard, uint32_t audioFreq, uint32_t dataFormat);

    /**
     * @brief Close the transmission session.
     */
    void stop ();

    /**
     * @brief Send an amount of data in DMA mode.
     */
    inline HAL_StatusTypeDef transmit (DeviceClient * _client, uint16_t * pData, uint16_t size)
    {
        startCommunication(_client, State::TX, State::TX_CMPL);
        halStatus = HAL_I2S_Transmit_DMA(&parameters, pData, size);
        return halStatus;
    }

    /**
     * @brief Receive an amount of data in DMA mode.
     */
    inline HAL_StatusTypeDef receive (DeviceClient * _client, uint16_t * pData, uint16_t size)
    {
        startCommunication(_client, State::RX, State::RX_CMPL);
        halStatus = HAL_I2S_Receive_DMA(&parameters, pData, size);
        return halStatus;
    }
};

} // end namespace

#endif
#endif

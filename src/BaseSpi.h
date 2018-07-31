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

#ifndef STM32ASYNC_BASESPI_H_
#define STM32ASYNC_BASESPI_H_

#include "HardwareLayout/Spi.h"
#include "IODevice.h"

namespace Stm32async
{

/**
 * @brief Base SPI class that holds the SPI parameters and implements the communication in
 *        a blocking mode.
 */
class BaseSpi : public IODevice<HardwareLayout::Spi, 3>
{
public:

    /**
     * @brief Default constructor.
     */
    BaseSpi (const HardwareLayout::Spi & _device);

    /**
     * @brief Getter for the device parameters
     */
    inline SPI_HandleTypeDef & getParameters ()
    {
        return parameters;
    }

    /**
     * @brief Open transmission session with given parameters.
     */
    DeviceStart::Status start (uint32_t direction, uint32_t prescaler,
                               uint32_t dataSize = SPI_DATASIZE_8BIT,
                               uint32_t CLKPhase = SPI_PHASE_1EDGE);

    /**
     * @brief Close the transmission session.
     */
    void stop ();

    /**
     * @brief Send an amount of data in blocking mode.
     */
    inline HAL_StatusTypeDef transmitBlocking (uint8_t * buffer, uint16_t n, uint32_t timeout = __UINT32_MAX__)
    {
        halStatus = HAL_SPI_Transmit(&parameters, buffer, n, timeout);
        return halStatus;
    }

    /**
     * @brief Receive an amount of data in blocking mode.
     */
    inline HAL_StatusTypeDef receiveBlocking (uint8_t * buffer, uint16_t n, uint32_t timeout = __UINT32_MAX__)
    {
        halStatus = HAL_SPI_Receive(&parameters, buffer, n, timeout);
        return halStatus;
    }

    /**
     * @brief Check whether the SPI communication is completed.
     */
    inline bool isBusy () const
    {
        return (((parameters.Instance->SR) & (SPI_FLAG_BSY)) == (SPI_FLAG_BSY));
    }

protected:

    SPI_HandleTypeDef parameters;
};

} // end namespace
#endif

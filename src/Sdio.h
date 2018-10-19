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

#ifndef STM32ASYNC_SDIO_H_
#define STM32ASYNC_SDIO_H_

#include "HardwareLayout/Sdio.h"

#ifdef HAL_SD_MODULE_ENABLED

#include "IODevice.h"
#include "SharedDevice.h"

namespace Stm32async
{

/**
 * @brief Class that implements SDIO interface.
 */
class Sdio : public IODevice<HardwareLayout::Sdio, SD_HandleTypeDef, 2>, public SharedDevice
{
public:

    static constexpr uint32_t TIMEOUT = 10000;

    /**
     * @brief Default constructor.
     */
    Sdio (const HardwareLayout::Sdio & _device, uint32_t _clockDiv);

    DeviceStart::Status start ();
    void stop ();
    void printInfo ();

    HAL_SD_ErrorTypedef readBlocks (uint32_t *pData, uint64_t addr, uint32_t blockSize, uint32_t numOfBlocks);
    HAL_SD_ErrorTypedef writeBlocks (uint32_t *pData, uint64_t addr, uint32_t blockSize, uint32_t numOfBlocks);

    inline const HAL_SD_CardInfoTypedef & getCardInfo () const
    {
        return cardInfo;
    }

    inline const HAL_SD_CardStatusTypedef & getCardStatus () const
    {
        return cardStatus;
    }

    inline void processSdIOInterrupt ()
    {
        HAL_SD_IRQHandler(&parameters);
    }

private:

    HAL_SD_CardInfoTypedef cardInfo;
    HAL_SD_CardStatusTypedef cardStatus;
};

} // end namespace

#endif
#endif

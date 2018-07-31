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

#ifndef STM32ASYNC_SDCARD_H_
#define STM32ASYNC_SDCARD_H_

#include "IODevice.h"
#include "SharedDevice.h"

#ifdef HAL_SD_MODULE_ENABLED

#include "HardwareLayout/Sdio.h"
#include "FatFS/ff_gen_drv.h"

namespace Stm32async
{

/**
 * @brief Class that implements SD card interface.
 */
class SdCard : public IODevice<HardwareLayout::Sdio, 2>, public SharedDevice
{
    DECLARE_STATIC_INSTANCE(SdCard)

public:

    static const uint32_t SDHC_BLOCK_SIZE = 512;
    static const size_t FAT_FS_OBJECT_LENGHT = 64;
    static const uint32_t TIMEOUT = 10000;

    typedef struct
    {
        FATFS key;  /* File system object for SD card logical drive */
        char path[4]; /* SD card logical drive path */
        DWORD volumeSN;
        char volumeLabel[FAT_FS_OBJECT_LENGHT];
        char currentDirectory[FAT_FS_OBJECT_LENGHT];
    } FatFs;

    /**
     * @brief Default constructor.
     */
    SdCard (const HardwareLayout::Sdio & _device, IOPort & _sdDetect, uint32_t _clockDiv);

    DeviceStart::Status start ();
    void stop ();

    HAL_SD_ErrorTypedef readBlocks (uint32_t *pData, uint64_t addr, uint32_t blockSize, uint32_t numOfBlocks);
    HAL_SD_ErrorTypedef writeBlocks (uint32_t *pData, uint64_t addr, uint32_t blockSize, uint32_t numOfBlocks);

    DeviceStart::Status mountFatFs ();
    void listFiles ();

    inline void processSdIOInterrupt ()
    {
        HAL_SD_IRQHandler(&parameters);
    }

    inline SD_HandleTypeDef & getParameters ()
    {
        return parameters;
    }

    inline const HAL_SD_CardInfoTypedef & getCardInfo () const
    {
        return cardInfo;
    }

    inline const HAL_SD_CardStatusTypedef & getCardStatus () const
    {
        return cardStatus;
    }

    inline bool isCardInserted () const
    {
        return !sdDetect.getBit();
    }

    const FatFs & getFatFs () const
    {
        return fatFs;
    }

private:

    IOPort & sdDetect;
    SD_HandleTypeDef parameters;
    HAL_SD_CardInfoTypedef cardInfo;
    HAL_SD_CardStatusTypedef cardStatus;

    // FAT FS
    static Diskio_drvTypeDef fatFsDriver;
    FatFs fatFs;
};

} // end namespace

#endif
#endif

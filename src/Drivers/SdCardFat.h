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

#ifndef DRIVERS_SDCARDFAT_H_
#define DRIVERS_SDCARDFAT_H_

#include "../Sdio.h"

#ifdef HAL_SD_MODULE_ENABLED

#include "FatFS/ff_gen_drv.h"

namespace Stm32async
{
namespace Drivers
{

/**
 * @brief Class that implements SD card handling using FAT FS.
 */
class SdCardFat
{
    DECLARE_STATIC_INSTANCE(SdCardFat)

public:

    class EventHandler
    {
    public:
        virtual void onSdCardAttach () =0;
        virtual void onSdCardDeAttach () =0;
    };

    static constexpr uint32_t SDHC_BLOCK_SIZE = 512;
    static constexpr size_t FAT_FS_OBJECT_LENGHT = 64;

    typedef struct
    {
        FATFS key; /* File system object for SD card logical drive */
        char path[4]; /* SD card logical drive path */
        DWORD volumeSN;
        char volumeLabel[FAT_FS_OBJECT_LENGHT];
        char currentDirectory[FAT_FS_OBJECT_LENGHT];
    } FatFs;

    /**
     * @brief Default constructor.
     */
    SdCardFat (const HardwareLayout::Sdio & _device, IOPort & _sdDetect, uint32_t _clockDiv);

    void periodic ();
    DeviceStart::Status mountFatFs ();
    void listFiles ();

    inline void setHandler (EventHandler * handler)
    {
        this->handler = handler;
    }

    inline EventHandler * getHandler () const
    {
        return handler;
    }

    inline Sdio & getSdio ()
    {
        return sdio;
    }

    inline bool isCardInserted () const
    {
        return !sdDetect.getBit();
    }

    inline const FatFs & getFatFs () const
    {
        return fatFs;
    }

    inline DeviceStart::Status start ()
    {
        if (!isCardInserted())
        {
            return DeviceStart::SD_NOT_INSERTED;
        }
        return sdio.start();
    }

    inline void stop ()
    {
        sdio.stop();
    }

private:

    Sdio sdio;
    IOPort & sdDetect;
    bool sdCardInserted;
    EventHandler * handler;
    static Diskio_drvTypeDef fatFsDriver;
    FatFs fatFs;
};

} // end of namespace Drivers
} // end of namespace Stm32async

#endif
#endif

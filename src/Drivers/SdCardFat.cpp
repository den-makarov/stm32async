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

#include "SdCardFat.h"
#include "../UsartLogger.h"

#ifdef HAL_SD_MODULE_ENABLED

using namespace Stm32async::Drivers;

#define USART_DEBUG_MODULE "SD: "

/************************************************************************
 * FAT FS driver
 ************************************************************************/

SdCardFat * SdCardFat::instance = NULL;

/**
  * @brief  Initializes a Drive
  * @param  lun : not used
  * @retval DSTATUS: Operation status
  */
DSTATUS SD_initialize(BYTE /*lun*/)
{
    return RES_OK;
}

/**
  * @brief  Gets Disk Status
  * @param  lun : not used
  * @retval DSTATUS: Operation status
  */
DSTATUS SD_status(BYTE /*lun*/)
{
    return RES_OK;
}

/**
  * @brief  Reads Sector(s)
  * @param  lun : not used
  * @param  *buff: Data buffer to store read data
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to read (1..128)
  * @retval DRESULT: Operation result
  */
DRESULT SD_read(BYTE /*lun*/, BYTE *buff, DWORD sector, UINT count)
{
    HAL_SD_ErrorTypedef status = SdCardFat::getInstance()->getSdio().readBlocks(
            (uint32_t*)buff, (uint64_t) (sector * SdCardFat::SDHC_BLOCK_SIZE), SdCardFat::SDHC_BLOCK_SIZE, count);
    return (status != SD_OK)? RES_ERROR : RES_OK;
}

/**
  * @brief  Writes Sector(s)
  * @param  lun : not used
  * @param  *buff: Data to be written
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to write (1..128)
  * @retval DRESULT: Operation result
  */
DRESULT SD_write(BYTE /*lun*/, const BYTE *buff, DWORD sector, UINT count)
{
    HAL_SD_ErrorTypedef status = SdCardFat::getInstance()->getSdio().writeBlocks(
            (uint32_t*)buff, (uint64_t)(sector * SdCardFat::SDHC_BLOCK_SIZE), SdCardFat::SDHC_BLOCK_SIZE, count);
    return (status != SD_OK)? RES_ERROR : RES_OK;
}

/**
  * @brief  I/O control operation
  * @param  lun : not used
  * @param  cmd: Control code
  * @param  *buff: Buffer to send/receive control data
  * @retval DRESULT: Operation result
  */
DRESULT SD_ioctl(BYTE /*lun*/, BYTE cmd, void *buff)
{
    DRESULT res = RES_ERROR;
    const HAL_SD_CardInfoTypedef & cardInfo = SdCardFat::getInstance()->getSdio().getCardInfo();
    switch (cmd)
    {
    /* Make sure that no pending write process */
    case CTRL_SYNC :
        res = RES_OK;
        break;

    /* Get number of sectors on the disk (DWORD) */
    case GET_SECTOR_COUNT :
        *(DWORD*)buff = cardInfo.CardCapacity / SdCardFat::SDHC_BLOCK_SIZE;
        res = RES_OK;
        break;

    /* Get R/W sector size (WORD) */
    case GET_SECTOR_SIZE :
        *(WORD*)buff = SdCardFat::SDHC_BLOCK_SIZE;
        res = RES_OK;
        break;

    /* Get erase block size in unit of sector (DWORD) */
    case GET_BLOCK_SIZE :
        *(DWORD*)buff = SdCardFat::SDHC_BLOCK_SIZE;
        break;

    default:
        res = RES_PARERR;
    }
    return res;
}


Diskio_drvTypeDef SdCardFat::fatFsDriver =
{
  SD_initialize,
  SD_status,
  SD_read,
  SD_write,
  SD_ioctl,
};


/************************************************************************
 * Class SdCardFat
 ************************************************************************/

SdCardFat::SdCardFat (const HardwareLayout::Sdio & _device, IOPort & _sdDetect, uint32_t _clockDiv):
    sdio { _device, _clockDiv },
    sdDetect { _sdDetect },
    sdCardInserted { false },
    handler { NULL }
{
    instance = this;
}


void SdCardFat::periodic ()
{
    bool s = isCardInserted();
    if (sdCardInserted != s)
    {
        sdCardInserted = s;
        USART_DEBUG("SD card " << (s? "inserted" : "de-attached") << UsartLogger::ENDL);
        if (handler != NULL)
        {
            if (sdCardInserted)
            {
                handler->onSdCardAttach();
            }
            else
            {
                handler->onSdCardDeAttach();
            }
        }
    }
}


Stm32async::DeviceStart::Status SdCardFat::mountFatFs ()
{
    uint8_t code1 = FATFS_LinkDriver(&fatFsDriver, fatFs.path);
    if (code1 != 0)
    {
        return DeviceStart::FAT_DRIVER_NOT_LINKED;
    }

    FRESULT code2 = f_mount(&fatFs.key, fatFs.path, 1);
    if (code2 != FR_OK)
    {
        return DeviceStart::FAT_VOLUME_NOT_MOUNTED;
    }

    code2 = f_getlabel(fatFs.path, fatFs.volumeLabel, &fatFs.volumeSN);
    if (code2 != FR_OK)
    {
        return DeviceStart::FAT_VOLUME_STATUS_ERROR;
    }

    code2 = f_getcwd(fatFs.currentDirectory, sizeof(fatFs.currentDirectory));
    if (code2 != FR_OK)
    {
        return DeviceStart::FAT_DIR_STATUS_ERROR;
    }

    return DeviceStart::OK;
}


void SdCardFat::listFiles()
{
    FRESULT res;
    DIR dir;
    FILINFO fno;

    res = f_opendir(&dir, fatFs.currentDirectory); /* Open the directory */
    if (res == FR_OK)
    {
        for (;;)
        {
            res = f_readdir(&dir, &fno); /* Read a directory item */
            if (res != FR_OK || fno.fname[0] == 0)
            {
                break; /* Break on error or end of dir */
            }
            if (fno.fattrib & AM_DIR)
            {
                USART_DEBUG("  " << fno.fname << " <DIR>" << UsartLogger::ENDL);
            }
            else
            {
                USART_DEBUG("  " << fno.fname << " (" << fno.fsize << ")" << UsartLogger::ENDL);
            }
        }
        f_closedir(&dir);
    }
}

#endif

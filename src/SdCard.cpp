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

#include "SdCard.h"
#include "UsartLogger.h"

#ifdef HAL_SD_MODULE_ENABLED

using namespace Stm32async;

#define USART_DEBUG_MODULE "SD: "

/************************************************************************
 * FAT FS driver
 ************************************************************************/

SdCard * SdCard::instance = NULL;

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
    HAL_SD_ErrorTypedef status = SdCard::getInstance()->readBlocks(
            (uint32_t*)buff, (uint64_t) (sector * SdCard::SDHC_BLOCK_SIZE), SdCard::SDHC_BLOCK_SIZE, count);
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
    HAL_SD_ErrorTypedef status = SdCard::getInstance()->writeBlocks(
            (uint32_t*)buff, (uint64_t)(sector * SdCard::SDHC_BLOCK_SIZE), SdCard::SDHC_BLOCK_SIZE, count);
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
    const HAL_SD_CardInfoTypedef & cardInfo = SdCard::getInstance()->getCardInfo();
    switch (cmd)
    {
    /* Make sure that no pending write process */
    case CTRL_SYNC :
        res = RES_OK;
        break;

    /* Get number of sectors on the disk (DWORD) */
    case GET_SECTOR_COUNT :
        *(DWORD*)buff = cardInfo.CardCapacity / SdCard::SDHC_BLOCK_SIZE;
        res = RES_OK;
        break;

    /* Get R/W sector size (WORD) */
    case GET_SECTOR_SIZE :
        *(WORD*)buff = SdCard::SDHC_BLOCK_SIZE;
        res = RES_OK;
        break;

    /* Get erase block size in unit of sector (DWORD) */
    case GET_BLOCK_SIZE :
        *(DWORD*)buff = SdCard::SDHC_BLOCK_SIZE;
        break;

    default:
        res = RES_PARERR;
    }
    return res;
}


Diskio_drvTypeDef SdCard::fatFsDriver =
{
  SD_initialize,
  SD_status,
  SD_read,
  SD_write,
  SD_ioctl,
};


/************************************************************************
 * Class SdCard
 ************************************************************************/

SdCard::SdCard (const HardwareLayout::Sdio & _device, IOPort & _sdDetect, uint32_t _clockDiv):
    IODevice { _device, {
        IOPort { _device.pins1.port, _device.pins1.pins, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH },
        IOPort { _device.pins2.port, _device.pins2.pins, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH }
    } },
    SharedDevice { _device.txDma, _device.rxDma, DMA_PDATAALIGN_WORD, DMA_MDATAALIGN_WORD },
    sdDetect { _sdDetect }
{
    parameters.Instance = device.getInstance();
    parameters.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
    parameters.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
    parameters.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
    parameters.Init.BusWide = SDIO_BUS_WIDE_1B;
    parameters.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_ENABLE;
    parameters.Init.ClockDiv = _clockDiv;

    instance = this;
}


DeviceStart::Status SdCard::start ()
{
    if (!isCardInserted())
    {
        return DeviceStart::SD_NOT_INSERTED;
    }

    device.enableClock();
    IODevice::enablePorts();

    HAL_SD_ErrorTypedef sdErrorStatus = HAL_SD_Init(&parameters, &cardInfo);
    if (sdErrorStatus != SD_OK)
    {
        return DeviceStart::DEVICE_INIT_ERROR;
    }

    sdErrorStatus = HAL_SD_WideBusOperation_Config(&parameters, SDIO_BUS_WIDE_4B);
    if (sdErrorStatus != SD_OK)
    {
        return DeviceStart::SD_WIDE_BUS_ERROR;
    }

    sdErrorStatus = HAL_SD_GetCardStatus(&parameters, &cardStatus);
    if (sdErrorStatus != SD_OK)
    {
        return DeviceStart::SD_READ_STATUS_ERROR;
    }

    txDma.Init.Mode = DMA_PFCTRL;
    txDma.Init.Priority = DMA_PRIORITY_LOW;
    txDma.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    txDma.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    txDma.Init.MemBurst = DMA_MBURST_INC4;
    txDma.Init.PeriphBurst = DMA_PBURST_INC4;
    device.txDma.dma->enableClock();
    __HAL_LINKDMA(&parameters, hdmatx, txDma);
    halStatus = HAL_DMA_Init(&txDma);
    if (halStatus != HAL_OK)
    {
        return DeviceStart::TX_DMA_INIT_ERROR;
    }

    rxDma.Init.Mode = DMA_PFCTRL;
    rxDma.Init.Priority = DMA_PRIORITY_LOW;
    rxDma.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    rxDma.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    rxDma.Init.MemBurst = DMA_MBURST_INC4;
    rxDma.Init.PeriphBurst = DMA_PBURST_INC4;
    device.rxDma.dma->enableClock();
    __HAL_LINKDMA(&parameters, hdmarx, rxDma);
    halStatus = HAL_DMA_Init(&rxDma);
    if (halStatus != HAL_OK)
    {
        return DeviceStart::RX_DMA_INIT_ERROR;
    }

    device.enableIrq();
    return DeviceStart::OK;
}


void SdCard::stop ()
{
    device.disableIrq();
    HAL_DMA_DeInit(&rxDma);
    device.rxDma.dma->disableClock();
    HAL_DMA_DeInit(&txDma);
    device.txDma.dma->disableClock();
    HAL_SD_DeInit(&parameters);
    IODevice::disablePorts();
    device.disableClock();
}


HAL_SD_ErrorTypedef SdCard::readBlocks (uint32_t *pData, uint64_t addr, uint32_t blockSize, uint32_t numOfBlocks)
{
    HAL_SD_ErrorTypedef status = HAL_SD_ReadBlocks_DMA(&parameters, pData, addr, blockSize, numOfBlocks);
    if (status == SD_OK)
    {
        uint8_t repeatNr = 0xFF;
        do
        {
            status = HAL_SD_CheckReadOperation(&parameters, TIMEOUT);
        }
        while (status == SD_DATA_TIMEOUT && --repeatNr > 0);

        if (status != SD_OK)
        {
            USART_DEBUG("Error at reading blocks (operation finish): " << status << UsartLogger::ENDL);
        }
    }
    else
    {
        USART_DEBUG("Error at reading blocks (operation start): " << status << UsartLogger::ENDL);
    }
    return status;
}


HAL_SD_ErrorTypedef SdCard::writeBlocks (uint32_t *pData, uint64_t addr, uint32_t blockSize, uint32_t numOfBlocks)
{
    HAL_SD_ErrorTypedef status = HAL_SD_WriteBlocks_DMA(&parameters, pData, addr, blockSize, numOfBlocks);
    if (status == SD_OK)
    {
        uint8_t repeatNr = 0xFF;
        do
        {
            status = HAL_SD_CheckWriteOperation(&parameters, TIMEOUT);
        }
        while (status == SD_DATA_TIMEOUT && --repeatNr > 0);

        if (status != SD_OK)
        {
            USART_DEBUG("Error at writing blocks (operation finish): " << status << UsartLogger::ENDL);
        }
    }
    else
    {
        USART_DEBUG("Error at writing blocks (operation start): " << status << UsartLogger::ENDL);
    }
    return status;
}


DeviceStart::Status SdCard::mountFatFs ()
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


void SdCard::listFiles()
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

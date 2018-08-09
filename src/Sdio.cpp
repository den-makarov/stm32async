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

#include "Sdio.h"
#include "UsartLogger.h"

#ifdef HAL_SD_MODULE_ENABLED

using namespace Stm32async;

#define USART_DEBUG_MODULE "SDIO: "

/************************************************************************
 * Class Sdio
 ************************************************************************/
Sdio::Sdio (const HardwareLayout::Sdio & _device, uint32_t _clockDiv) :
    IODevice { _device, {
              IOPort { _device.pins1.port, _device.pins1.pins, GPIO_MODE_AF_PP,
                       GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH },
              IOPort { _device.pins2.port, _device.pins2.pins, GPIO_MODE_AF_PP,
                       GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH } } },
    SharedDevice { &device.txDma, &device.rxDma, DMA_PDATAALIGN_WORD, DMA_MDATAALIGN_WORD }
{
    parameters.Instance = device.getInstance();
    parameters.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
    parameters.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
    parameters.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
    parameters.Init.BusWide = SDIO_BUS_WIDE_1B;
    parameters.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_ENABLE;
    parameters.Init.ClockDiv = _clockDiv;
}

DeviceStart::Status Sdio::start ()
{
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
    __HAL_LINKDMA(&parameters, hdmatx, txDma);

    rxDma.Init.Mode = DMA_PFCTRL;
    rxDma.Init.Priority = DMA_PRIORITY_LOW;
    rxDma.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    rxDma.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    rxDma.Init.MemBurst = DMA_MBURST_INC4;
    rxDma.Init.PeriphBurst = DMA_PBURST_INC4;
    __HAL_LINKDMA(&parameters, hdmarx, rxDma);

    DeviceStart::Status status = startDma(halStatus);
    if (status == DeviceStart::OK)
    {
        device.enableIrq();
    }
    return status;
}

void Sdio::stop ()
{
    device.disableIrq();
    stopDma();
    HAL_SD_DeInit(&parameters);
    IODevice::disablePorts();
    device.disableClock();
}

void Sdio::printInfo ()
{
    USART_DEBUG("CardType = " << cardInfo.CardType << UsartLogger::ENDL
             << UsartLogger::TAB << "CardCapacity = " << cardInfo.CardCapacity/1024L/1024L << "Mb" << UsartLogger::ENDL
             << UsartLogger::TAB << "CardBlockSize = " << cardInfo.CardBlockSize << UsartLogger::ENDL
             << UsartLogger::TAB << "DAT_BUS_WIDTH = " << cardStatus.DAT_BUS_WIDTH << UsartLogger::ENDL
             << UsartLogger::TAB << "SD_CARD_TYPE = " << cardStatus.SD_CARD_TYPE << UsartLogger::ENDL
             << UsartLogger::TAB << "SPEED_CLASS = " << cardStatus.SPEED_CLASS << UsartLogger::ENDL);
}

HAL_SD_ErrorTypedef Sdio::readBlocks (uint32_t *pData, uint64_t addr, uint32_t blockSize, uint32_t numOfBlocks)
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

HAL_SD_ErrorTypedef Sdio::writeBlocks (uint32_t *pData, uint64_t addr, uint32_t blockSize, uint32_t numOfBlocks)
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

#endif

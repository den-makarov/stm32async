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

#include "SharedDevice.h"

using namespace Stm32async;

/************************************************************************
 * Class SharedDevice
 ************************************************************************/

SharedDevice::SharedDevice (const HardwareLayout::DmaStream * _txStream, const HardwareLayout::DmaStream * _rxStream,
                            uint32_t _periphDataAlignment, uint32_t _memDataAlignment) :
    client { NULL },
    currState { State::NONE },
    targetState { State::NONE },
    startTime { __UINT32_MAX__ },
    timeout { __UINT32_MAX__ },
    txStream { _txStream },
    rxStream { _rxStream }
{
    if (txStream != NULL)
    {
        txDma.Instance = txStream->stream;
        HAL_EXT_DMA_SET_CHANNEL(txDma, txStream->channel);
        txDma.Init.Direction = DMA_MEMORY_TO_PERIPH;
        txDma.Init.PeriphInc = DMA_PINC_DISABLE;
        txDma.Init.MemInc = DMA_MINC_ENABLE;
        txDma.Init.PeriphDataAlignment = _periphDataAlignment;
        txDma.Init.MemDataAlignment = _memDataAlignment;
        txDma.Init.Mode = DMA_NORMAL;
        txDma.Init.Priority = DMA_PRIORITY_LOW;
        HAL_EXT_DMA_SET_FIFOMODE(txDma, DMA_FIFOMODE_DISABLE);
    }

    if (rxStream != NULL)
    {
        rxDma.Instance = rxStream->stream;
        HAL_EXT_DMA_SET_CHANNEL(rxDma, rxStream->channel);
        rxDma.Init.Direction = DMA_PERIPH_TO_MEMORY;
        rxDma.Init.PeriphInc = DMA_PINC_DISABLE;
        rxDma.Init.MemInc = DMA_MINC_ENABLE;
        rxDma.Init.PeriphDataAlignment = _periphDataAlignment;
        rxDma.Init.MemDataAlignment = _memDataAlignment;
        rxDma.Init.Mode = DMA_NORMAL;
        rxDma.Init.Priority = DMA_PRIORITY_LOW;
        HAL_EXT_DMA_SET_FIFOMODE(rxDma, DMA_FIFOMODE_DISABLE);
    }
}

void SharedDevice::startCommunication (DeviceClient * client, State currState, State targetState)
{
    this->client = client;
    this->currState = currState;
    this->targetState = targetState;
    startTime = HAL_GetTick();
}

void SharedDevice::waitForRelease ()
{
    while (!isFinished())
    {
        periodic();
    }
}

DeviceStart::Status SharedDevice::startDma (HAL_StatusTypeDef & halStatus)
{
    if (txStream != NULL)
    {
        txStream->dma->enableClock();
        halStatus = HAL_DMA_Init(&txDma);
        if (halStatus != HAL_OK)
        {
            return DeviceStart::TX_DMA_INIT_ERROR;
        }
    }

    if (rxStream != NULL)
    {
        rxStream->dma->enableClock();
        halStatus = HAL_DMA_Init(&rxDma);
        if (halStatus != HAL_OK)
        {
            return DeviceStart::RX_DMA_INIT_ERROR;
        }
    }

    return DeviceStart::OK;
}

void SharedDevice::stopDma ()
{
    if (txStream != NULL)
    {
        HAL_DMA_DeInit(&txDma);
        txStream->dma->disableClock();
    }
    if (rxStream != NULL)
    {
        HAL_DMA_DeInit(&rxDma);
        rxStream->dma->disableClock();
    }
}

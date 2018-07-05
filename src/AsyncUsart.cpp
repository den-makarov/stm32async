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

#include "AsyncUsart.h"

using namespace Stm32async;

/************************************************************************
 * Class AsyncUsart
 ************************************************************************/

AsyncUsart::AsyncUsart (const HardwareLayout::Usart & _device) :
    device { _device },
    txPin { _device.txPin.port, _device.txPin.pins, GPIO_MODE_AF_PP },
    rxPin { _device.rxPin.port, _device.rxPin.pins, GPIO_MODE_AF_PP }
{
    txPin.setAlternate(device.txPin.alternate);
    rxPin.setAlternate(device.rxPin.alternate);
    parameters.Instance = device.getInstance();
    parameters.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    parameters.Init.OverSampling = UART_OVERSAMPLING_16;
    #ifdef UART_ONE_BIT_SAMPLE_DISABLE
        usartParameters.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    #endif
    #ifdef UART_ADVFEATURE_NO_INIT
        usartParameters.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    #endif

    txDma.Instance = device.txDma.stream;
#ifndef STM32F1
    txDma.Init.Channel = device.txDma.channel;
#endif
    txDma.Init.Direction = DMA_MEMORY_TO_PERIPH;
    txDma.Init.PeriphInc = DMA_PINC_DISABLE;
    txDma.Init.MemInc = DMA_MINC_ENABLE;
    txDma.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    txDma.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    txDma.Init.Mode = DMA_NORMAL;
    txDma.Init.Priority = DMA_PRIORITY_LOW;
#ifndef STM32F1
    txDma.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
#endif

    rxDma.Instance = device.rxDma.stream;
#ifndef STM32F1
    rxDma.Init.Channel = device.rxDma.channel;
#endif
    rxDma.Init.Direction = DMA_MEMORY_TO_PERIPH;
    rxDma.Init.PeriphInc = DMA_PINC_DISABLE;
    rxDma.Init.MemInc = DMA_MINC_ENABLE;
    rxDma.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    rxDma.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    rxDma.Init.Mode = DMA_NORMAL;
    rxDma.Init.Priority = DMA_PRIORITY_LOW;
#ifndef STM32F1
    rxDma.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
#endif
}

HAL_StatusTypeDef AsyncUsart::start (uint32_t mode, uint32_t baudRate,
                                     uint32_t wordLength/* = UART_WORDLENGTH_8B*/,
                                     uint32_t stopBits/* = UART_STOPBITS_1*/,
                                     uint32_t parity/* = UART_PARITY_NONE*/)
{
    device.enableClock();
    txPin.start();
    rxPin.start();
    parameters.Init.Mode = mode;
    parameters.Init.BaudRate = baudRate;
    parameters.Init.WordLength = wordLength;
    parameters.Init.StopBits = stopBits;
    parameters.Init.Parity = parity;
    HAL_StatusTypeDef status = HAL_UART_Init(&parameters);
    if (status != HAL_OK)
    {
        return HAL_ERROR;
    }

    device.txDma.dma->enableClock();
    __HAL_LINKDMA(&parameters, hdmatx, txDma);
    status = HAL_DMA_Init(&txDma);
    if (status != HAL_OK)
    {
        return HAL_ERROR;
    }

    device.rxDma.dma->enableClock();
    __HAL_LINKDMA(&parameters, hdmarx, rxDma);
    status = HAL_DMA_Init(&rxDma);
    if (status != HAL_OK)
    {
        return HAL_ERROR;
    }

    device.enableIrq();
    return HAL_OK;
}

void AsyncUsart::stop ()
{
    device.disableIrq();
    HAL_DMA_DeInit(&txDma);
    HAL_DMA_DeInit(&rxDma);
    HAL_UART_DeInit(&parameters);
    txPin.stop();
    rxPin.stop();
    device.txDma.dma->disableClock();
    device.rxDma.dma->disableClock();
    device.disableClock();
}

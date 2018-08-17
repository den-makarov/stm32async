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

#include "Usart.h"

#ifdef HAL_UART_MODULE_ENABLED

using namespace Stm32async;

/************************************************************************
 * Class BaseUsart
 ************************************************************************/

BaseUsart::BaseUsart (const HardwareLayout::Usart & _device) :
    IODevice { _device, {
        IOPort { _device.txPin.port, _device.txPin.pins, GPIO_MODE_AF_PP, GPIO_PULLUP },
        IOPort { _device.rxPin.port, _device.rxPin.pins, GPIO_MODE_AF_PP, GPIO_PULLUP }
    } }
{
    parameters.Instance = device.getInstance();
    parameters.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    parameters.Init.OverSampling = UART_OVERSAMPLING_16;
}

DeviceStart::Status BaseUsart::start (uint32_t mode, uint32_t baudRate,
                                      uint32_t wordLength/* = UART_WORDLENGTH_8B*/,
                                      uint32_t stopBits/* = UART_STOPBITS_1*/,
                                      uint32_t parity/* = UART_PARITY_NONE*/)
{
    __HAL_UART_ENABLE(&parameters);

    device.enableClock();
    IODevice::enablePorts();

    parameters.Init.Mode = mode;
    parameters.Init.BaudRate = baudRate;
    parameters.Init.WordLength = wordLength;
    parameters.Init.StopBits = stopBits;
    parameters.Init.Parity = parity;
    HAL_StatusTypeDef status = HAL_UART_Init(&parameters);
    if (status != HAL_OK)
    {
        return DeviceStart::DEVICE_INIT_ERROR;
    }

    return DeviceStart::OK;
}

void BaseUsart::stop ()
{
    HAL_UART_DeInit(&parameters);
    IODevice::disablePorts();
    device.disableClock();
    __HAL_UART_DISABLE(&parameters);
}

/************************************************************************
 * Class AsyncUsart
 ************************************************************************/

AsyncUsart::AsyncUsart (const HardwareLayout::Usart & _device) :
    BaseUsart { _device },
    SharedDevice { (isPortUsed(getTxPin())? &device.txDma : NULL), 
                   (isPortUsed(getRxPin())? &device.rxDma : NULL),
                   DMA_PDATAALIGN_BYTE, DMA_MDATAALIGN_BYTE }
{
    // empty
}

DeviceStart::Status AsyncUsart::start (uint32_t mode, uint32_t baudRate,
                                       uint32_t wordLength/* = UART_WORDLENGTH_8B*/,
                                       uint32_t stopBits/* = UART_STOPBITS_1*/,
                                       uint32_t parity/* = UART_PARITY_NONE*/,
                                       bool isIrqEnabled/* = true*/)
{
    DeviceStart::Status status = BaseUsart::start(mode, baudRate, wordLength, stopBits, parity);
    if (status == DeviceStart::OK)
    {
        if (isTxMode())
        {
            __HAL_LINKDMA(&parameters, hdmatx, txDma);
        }
        if (isRxMode())
        {
            __HAL_LINKDMA(&parameters, hdmarx, rxDma);
        }
        status = startDma(halStatus);
        if (isIrqEnabled && status == DeviceStart::OK)
        {
            enableIrq();
        }
    }
    return status;
}

void AsyncUsart::stop ()
{
    disableIrq();
    stopDma();
    BaseUsart::stop();
}

#endif

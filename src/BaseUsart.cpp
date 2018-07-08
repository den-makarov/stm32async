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

#include "BaseUsart.h"

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
    #ifdef UART_ONE_BIT_SAMPLE_DISABLE
        usartParameters.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    #endif
    #ifdef UART_ADVFEATURE_NO_INIT
        usartParameters.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    #endif
}

HAL_StatusTypeDef BaseUsart::start (uint32_t mode, uint32_t baudRate,
                                    uint32_t wordLength/* = UART_WORDLENGTH_8B*/,
                                    uint32_t stopBits/* = UART_STOPBITS_1*/,
                                    uint32_t parity/* = UART_PARITY_NONE*/)
{
    device.enableClock();
    IODevice::enablePort();

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

    return HAL_OK;
}

void BaseUsart::stop ()
{
    HAL_UART_DeInit(&parameters);
    IODevice::disablePort();
    device.disableClock();
}

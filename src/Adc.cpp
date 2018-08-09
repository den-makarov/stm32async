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

#include "Adc.h"

#ifdef HAL_ADC_MODULE_ENABLED

using namespace Stm32async;

/************************************************************************
 * Class BaseAdc
 ************************************************************************/

BaseAdc::BaseAdc (const HardwareLayout::Adc & _device, uint32_t _channel, uint32_t _samplingTime):
    IODevice { _device, {
         IOPort { _device.pin.port, _device.pin.pins, GPIO_MODE_ANALOG, GPIO_NOPULL }
    } },
    vRef { 0.0 }
{
    parameters.Instance = device.getInstance();
    parameters.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
    parameters.Init.Resolution = ADC_RESOLUTION_12B;
    parameters.Init.ScanConvMode = DISABLE;
    parameters.Init.ContinuousConvMode = ENABLE;
    parameters.Init.DiscontinuousConvMode = DISABLE;
    parameters.Init.NbrOfDiscConversion = 0;
    parameters.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    parameters.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
    parameters.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    parameters.Init.NbrOfConversion = 1;
    parameters.Init.DMAContinuousRequests = ENABLE;
    parameters.Init.EOCSelection = ADC_EOC_SINGLE_CONV;

    adcChannel.Channel = _channel; // each channel is connected to the specific pin, see pin descriptions
    adcChannel.Rank = 1;
    adcChannel.SamplingTime = _samplingTime;
    adcChannel.Offset = 0;
}


DeviceStart::Status BaseAdc::start ()
{
    HAL_ADC_DeInit(&parameters);
    __HAL_ADC_ENABLE(&parameters);

    device.enableClock();
    IODevice::enablePorts();

    halStatus = HAL_ADC_Init(&parameters);
    if (halStatus != HAL_OK)
    {
        return DeviceStart::DEVICE_INIT_ERROR;
    }

    halStatus = HAL_ADC_ConfigChannel(&parameters, &adcChannel);
    if (halStatus != HAL_OK)
    {
        return DeviceStart::ADC_CHANNEL_ERROR;
    }

    return DeviceStart::OK;
}


void BaseAdc::stop ()
{
    HAL_ADC_DeInit(&parameters);
    IODevice::disablePorts();
    device.disableClock();
    __HAL_ADC_DISABLE(&parameters);
}


uint32_t BaseAdc::readBlocking ()
{
    uint32_t value = INVALID_VALUE;
    if (HAL_ADC_Start(&parameters) == HAL_OK)
    {
        if (HAL_ADC_PollForConversion(&parameters, TIMEOUT) == HAL_OK)
        {
            value = HAL_ADC_GetValue(&parameters);
        }
    }
    HAL_ADC_Stop(&parameters);
    return value;
}

#endif

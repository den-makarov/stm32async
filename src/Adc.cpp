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

#include <algorithm>

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
    parameters.Init.ContinuousConvMode = DISABLE;
    parameters.Init.DiscontinuousConvMode = DISABLE;
    parameters.Init.NbrOfDiscConversion = 0;
    parameters.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    parameters.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
    parameters.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    parameters.Init.NbrOfConversion = 1;
    parameters.Init.DMAContinuousRequests = DISABLE;
    parameters.Init.EOCSelection = ADC_EOC_SINGLE_CONV;

    adcChannel.Channel = _channel; // each channel is connected to the specific pin, see pin descriptions
    adcChannel.Rank = 1;
    adcChannel.SamplingTime = _samplingTime;
    adcChannel.Offset = 0;
}


DeviceStart::Status BaseAdc::start ()
{
    HAL_ADC_Stop(&parameters);
    HAL_ADC_DeInit(&parameters);

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

    halStatus = HAL_ADC_Start(&parameters);
    if (halStatus != HAL_OK)
    {
        return DeviceStart::ADC_CHANNEL_ERROR;
    }

    return DeviceStart::OK;
}


void BaseAdc::stop ()
{
    HAL_ADC_Stop(&parameters);
    HAL_ADC_DeInit(&parameters);
    IODevice::disablePorts();
    device.disableClock();
}


uint32_t BaseAdc::readBlocking ()
{
    uint32_t value = INVALID_VALUE;
    if (HAL_ADC_PollForConversion(&parameters, TIMEOUT) == HAL_OK)
    {
        value = HAL_ADC_GetValue(&parameters);
    }
    return value;
}

/************************************************************************
 * Class AsyncAdc
 ************************************************************************/

AsyncAdc::AsyncAdc (const HardwareLayout::Adc & _device, uint32_t _channel, uint32_t _samplingTime):
    BaseAdc { _device, _channel, _samplingTime },
    SharedDevice { NULL, &device.rxDma, DMA_PDATAALIGN_WORD, DMA_MDATAALIGN_WORD },
    nrReadings { 0 }
{
    parameters.Init.ContinuousConvMode = ENABLE;
    parameters.Init.DMAContinuousRequests = ENABLE;
    rxDma.Init.Mode = DMA_CIRCULAR;
}


DeviceStart::Status AsyncAdc::start ()
{
    DeviceStart::Status status = BaseAdc::start();
    if (status == DeviceStart::OK)
    {
        HAL_DMA_DeInit(&rxDma);
        __HAL_LINKDMA(&parameters, DMA_Handle, rxDma);
        status = startDma(halStatus);
        if (status == DeviceStart::OK)
        {
            enableIrq();
        }
    }
    HAL_ADC_Stop(&parameters);
    return status;
}


void AsyncAdc::stop ()
{
    disableIrq();
    stopDma();
    BaseAdc::stop();
}


HAL_StatusTypeDef AsyncAdc::read ()
{
    nrReadings = 0;
    adcBuffer.fill(0);
    halStatus = HAL_ADC_Start_DMA(&parameters, adcBuffer.data(), ADC_BUFFER_LENGTH);
    return halStatus;
}


bool AsyncAdc::processConvCpltCallback ()
{
    ++nrReadings;
    if (nrReadings == 1)
    {
        HAL_ADC_Stop_DMA(&parameters);
        return true;
    }
    return false;
}


uint32_t AsyncAdc::getMedian ()
{
    std::sort(adcBuffer.begin(), adcBuffer.end());
    return adcBuffer[ADC_BUFFER_LENGTH/2];
}


#endif

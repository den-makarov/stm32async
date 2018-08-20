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

#include "Dac.h"

#ifdef HAL_DAC_MODULE_ENABLED

using namespace Stm32async;

/************************************************************************
 * Class BaseDac
 ************************************************************************/

BaseDac::BaseDac (const HardwareLayout::Dac & _device, uint32_t _channel):
    IODevice { _device, {
         IOPort { _device.pin.port, _device.pin.pins, GPIO_MODE_ANALOG, GPIO_NOPULL }
    } },
    channelIdx { _channel }
{
    parameters.Instance = device.getInstance();
    dacChannel.DAC_Trigger = DAC_TRIGGER_NONE;
    dacChannel.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
}


DeviceStart::Status BaseDac::start ()
{
    HAL_DAC_Stop(&parameters, channelIdx);
    HAL_DAC_DeInit(&parameters);

    device.enableClock();
    IODevice::enablePorts();

    halStatus = HAL_DAC_Init(&parameters);
    if (halStatus != HAL_OK)
    {
        return DeviceStart::DEVICE_INIT_ERROR;
    }

    halStatus = HAL_DAC_ConfigChannel(&parameters, &dacChannel, channelIdx);
    if (halStatus != HAL_OK)
    {
        return DeviceStart::DAC_CHANNEL_ERROR;
    }

    halStatus = HAL_DAC_Start(&parameters, channelIdx);
    if (halStatus != HAL_OK)
    {
        return DeviceStart::DEVICE_INIT_ERROR;
    }

    return DeviceStart::OK;
}


void BaseDac::stop ()
{
    HAL_DAC_Stop(&parameters, channelIdx);
    HAL_DAC_DeInit(&parameters);
    IODevice::disablePorts();
    device.disableClock();
}


HAL_StatusTypeDef BaseDac::setValue (uint32_t v)
{
    return HAL_DAC_SetValue(&parameters, channelIdx, DAC_ALIGN_12B_R, v);
}

#endif

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

#include "I2S.h"

#ifdef HAL_I2S_MODULE_ENABLED

using namespace Stm32async;

/************************************************************************
 * Class AsyncI2S
 ************************************************************************/

AsyncI2S::AsyncI2S (const HardwareLayout::I2S & _device):
    IODevice { _device, {
              IOPort { _device.pins.port, _device.pins.pins, GPIO_MODE_AF_PP,
                       GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH } } },
    SharedDevice { &device.txDma, &device.rxDma, DMA_PDATAALIGN_HALFWORD, DMA_MDATAALIGN_HALFWORD }
{
    parameters.Instance = device.getInstance();
    parameters.Init.Mode = I2S_MODE_MASTER_TX;
    parameters.Init.Standard = I2S_STANDARD_PHILIPS; // will be re-defined at communication start
    parameters.Init.DataFormat = I2S_DATAFORMAT_16B; // will be re-defined at communication start
    parameters.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
    parameters.Init.AudioFreq = I2S_AUDIOFREQ_44K; // will be re-defined at communication start
    parameters.Init.CPOL = I2S_CPOL_LOW;
    parameters.Init.ClockSource = I2S_CLOCK_PLL;
    parameters.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
}


DeviceStart::Status AsyncI2S::start (uint32_t standard, uint32_t audioFreq, uint32_t dataFormat)
{
    __HAL_I2S_ENABLE(&parameters);

    parameters.Init.Standard = standard;
    parameters.Init.AudioFreq = audioFreq;
    parameters.Init.DataFormat = dataFormat;

    device.enableClock();
    IODevice::enablePorts();

    halStatus = HAL_I2S_Init(&parameters);
    if (halStatus != HAL_OK)
    {
        return DeviceStart::DEVICE_INIT_ERROR;
    }

    if (isTxMode())
    {
        HAL_DMA_DeInit(&txDma);
        __HAL_LINKDMA(&parameters, hdmatx, txDma);
    }
    if (isRxMode())
    {
        HAL_DMA_DeInit(&rxDma);
        __HAL_LINKDMA(&parameters, hdmarx, rxDma);
    }

    DeviceStart::Status status = startDma(halStatus);
    if (status == DeviceStart::OK)
    {
        device.enableIrq();
    }
    return status;
}


void AsyncI2S::stop ()
{
    device.disableIrq();
    stopDma();
    HAL_I2S_DeInit(&parameters);
    device.disableClock();
    IODevice::disablePorts();
    __HAL_I2S_DISABLE(&parameters);
}

#endif

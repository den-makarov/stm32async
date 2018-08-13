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

#include "Spi.h"

#ifdef HAL_SPI_MODULE_ENABLED

using namespace Stm32async;

/************************************************************************
 * Class BaseSpi
 ************************************************************************/

BaseSpi::BaseSpi (const HardwareLayout::Spi & _device) :
    IODevice { _device, {
        IOPort { _device.sclkPin.port, _device.sclkPin.pins, GPIO_MODE_AF_PP, GPIO_PULLUP },
        IOPort { _device.mosiPin.port, _device.mosiPin.pins, GPIO_MODE_AF_PP, GPIO_PULLUP },
        IOPort { _device.misoPin.port, _device.misoPin.pins, GPIO_MODE_AF_PP, GPIO_PULLUP }
    } }
{
    parameters.Instance = device.getInstance();
    parameters.Init.Mode = SPI_MODE_MASTER;
    parameters.Init.DataSize = SPI_DATASIZE_8BIT;
    parameters.Init.CLKPolarity = SPI_POLARITY_HIGH;
    parameters.Init.CLKPhase = SPI_PHASE_1EDGE;
    parameters.Init.FirstBit = SPI_FIRSTBIT_MSB;
    parameters.Init.TIMode = SPI_TIMODE_DISABLE;
    parameters.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    parameters.Init.CRCPolynomial = 7;
    parameters.Init.NSS = SPI_NSS_SOFT;
}

DeviceStart::Status BaseSpi::start (uint32_t direction, uint32_t prescaler,
                                    uint32_t dataSize/* = SPI_DATASIZE_8BIT*/,
                                    uint32_t CLKPhase/* = SPI_PHASE_1EDGE*/)
{
    __HAL_SPI_ENABLE(&parameters);

    device.enableClock();
    IODevice::enablePorts();

    parameters.Init.Direction = direction;
    parameters.Init.BaudRatePrescaler = prescaler;
    parameters.Init.DataSize = dataSize;
    parameters.Init.CLKPhase = CLKPhase;
    halStatus = HAL_SPI_Init(&parameters);
    if (halStatus != HAL_OK)
    {
        return DeviceStart::DEVICE_INIT_ERROR;
    }

    /* Configure communication direction : 1Line */
    if (parameters.Init.Direction == SPI_DIRECTION_1LINE)
    {
        SPI_1LINE_TX(&parameters);
    }

    return DeviceStart::OK;
}

void BaseSpi::stop ()
{
    HAL_SPI_DeInit(&parameters);
    IODevice::disablePorts();
    device.disableClock();
    __HAL_SPI_DISABLE(&parameters);
}

/************************************************************************
 * Class AsyncSpi
 ************************************************************************/

AsyncSpi::AsyncSpi (const HardwareLayout::Spi & _device) :
    BaseSpi { _device },
    SharedDevice { (isPortUsed(getMosiPin())? &device.txDma : NULL), 
                   (isPortUsed(getMisoPin())? &device.rxDma : NULL),
                   DMA_PDATAALIGN_BYTE, DMA_MDATAALIGN_BYTE }
{
    // empty
}

DeviceStart::Status AsyncSpi::start (uint32_t direction, uint32_t prescaler,
                                     uint32_t dataSize/* = SPI_DATASIZE_8BIT*/,
                                     uint32_t CLKPhase/* = SPI_PHASE_1EDGE*/)
{
    DeviceStart::Status status = BaseSpi::start(direction, prescaler, dataSize, CLKPhase);
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
        if (status == DeviceStart::OK)
        {
            enableIrq();
        }
    }
    return status;
}

void AsyncSpi::stop ()
{
    disableIrq();
    stopDma();
    BaseSpi::stop();
}

#endif

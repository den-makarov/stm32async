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

#include "IOPort.h"

using namespace Stm32async;

/************************************************************************
 * Class IOPort
 ************************************************************************/

IOPort::IOPort (const HardwareLayout::Port & _port, uint32_t _pins, uint32_t _mode,
                uint32_t _pull, uint32_t _speed) :
    port { _port }
{
    parameters.Pin = _pins;
    parameters.Mode = _mode;
    parameters.Pull = _pull;
    parameters.Speed = _speed;
}

void IOPort::start ()
{
    port.enableClock();
    HAL_GPIO_Init(port.getInstance(), &parameters);
}

void IOPort::stop ()
{
    HAL_GPIO_DeInit(port.getInstance(), parameters.Pin);
    port.disableClock();
}

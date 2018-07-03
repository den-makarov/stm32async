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

SharedDevice::SharedDevice () :
    client { NULL },
    currState { State::NONE },
    targetState { State::NONE },
    startTime { __UINT32_MAX__ },
    timeout { __UINT32_MAX__ }
{
    // empty
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

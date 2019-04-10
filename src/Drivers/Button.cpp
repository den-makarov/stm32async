/*******************************************************************************
 * StmPlusPlus: object-oriented library implementing device drivers for 
 * STM32F3 and STM32F4 MCU
 * *****************************************************************************
 * Copyright (C) 2016-2017 Mikhail Kulesh
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

#include "Button.h"

using namespace Stm32async::Drivers;

/************************************************************************
 * Class Button
 ************************************************************************/

Button::Button (const HardwareLayout::Port & _port, uint32_t _pin, uint32_t _pull,
                duration_ms _pressDelay, duration_ms _pressDuration) :
    IOPort { _port, _pin, GPIO_MODE_INPUT, _pull, GPIO_SPEED_LOW },
    pressDelay { _pressDelay },
    pressDuration { _pressDuration },
    pressTime { INFINITY_TIME },
    currentState { false },
    numOccured { 0 }
{
    // empty
}


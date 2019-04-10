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

#ifndef DRIVERS_BUTTON_H_
#define DRIVERS_BUTTON_H_

#include "../IOPort.h"
#include "../Rtc.h"

namespace Stm32async
{
namespace Drivers
{

/** 
 * @brief Class describing a button connected to a pin
 */
class Button : public IOPort
{
public:

    Button (const HardwareLayout::Port & _port, uint32_t _pin, uint32_t _pull, duration_ms _pressDelay = 50, duration_ms _pressDuration = 300);

    template <typename HANDLER>
    void periodic (HANDLER h);

private:

    duration_ms pressDelay, pressDuration;
    time_ms pressTime;
    bool currentState;
    uint32_t numOccured;
};

template <typename HANDLER>
void Button::periodic (HANDLER h)
{
    bool newState = (parameters.Pull == GPIO_PULLUP) ? !getBit() : getBit();
    time_ms currentTime = Rtc::getInstance()->getUpTimeMillisec();
    if (currentState == newState)
    {
        // state is not changed: check for periodical press event
        if (currentState && pressTime != INFINITY_TIME)
        {
            duration_ms d = currentTime - pressTime;
            if (d >= pressDuration)
            {
                h(numOccured);
                pressTime = currentTime;
                ++numOccured;
            }
        }
    }
    else if (!currentState && newState)
    {
        pressTime = currentTime;
        numOccured = 0;
    }
    else
    {
        duration_ms d = currentTime - pressTime;
        if (d < pressDelay)
        {
            // nothing to do
        }
        else if (numOccured == 0)
        {
            h(numOccured);
        }
        pressTime = INFINITY_TIME;
    }
    currentState = newState;
}

} // end of namespace Drivers
} // end of namespace Stm32async

#endif

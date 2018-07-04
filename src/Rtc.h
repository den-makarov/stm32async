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

#include "HardwareLayout.h"

#ifndef STM32ASYNC_RTC_H_
#define STM32ASYNC_RTC_H_

namespace Stm32async
{

/**
 * @brief Class that implements real-time clock.
 */
class Rtc
{
   DECLARE_STATIC_INSTANCE(Rtc);

public:

    class EventHandler
    {
    public:

        virtual void onRtcWakeUp () =0;
    };

    /**
     * @brief A class providing the device start-up success/error code
     */
    class Start
    {
    public:
        /**
         * @brief Set of valid enumeration values
         */
        enum Status
        {
            OK = 0,
            RTC_CLOCK_ERROR = 1,
            RTC_INIT_ERROR = 2,
            IT_INIT_ERROR = 3
        };

        /**
         * @brief Number of enumeration values
         */
        enum
        {
            size = 4
        };

        /**
         * @brief String representations of all enumeration values
         */
        static const char * strings[];

        /**
         * @brief the Convert() method
         */
        static ConvertClass<Status, size, strings> convert;

        /**
         * @brief the AsString() method
         */
        static AsStringClass<Status, size, strings> asString;
    };

    /**
     * @brief Default constructor.
     */
    Rtc (HardwareLayout::Interrupt && _wkUpIrq);

    Start::Status start (uint32_t counterMode, uint32_t prescaler);
    void stop ();

    void onSecondInterrupt ();

    inline void setHandler (EventHandler * handler)
    {
        this->handler = handler;
    }

    inline HAL_StatusTypeDef getHalStatus () const
    {
        return halStatus;
    }

    inline time_t getTimeSec () const
    {
        return timeSec;
    }

    inline void setTimeSec (time_t sec)
    {
        timeSec = sec;
    }

private:

    RTC_HandleTypeDef rtcParameters;
    RTC_TimeTypeDef timeParameters;
    RTC_DateTypeDef dateParameters;

    HardwareLayout::Interrupt wkUpIrq;
    EventHandler * handler;
    HAL_StatusTypeDef halStatus;

    // These variables are modified from interrupt service routine, therefore declare them as volatile
    volatile time_t timeSec; // up-time and current time (in seconds)
};

} // end namespace
#endif


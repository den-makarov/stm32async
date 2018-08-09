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

#ifndef STM32ASYNC_RTC_H_
#define STM32ASYNC_RTC_H_

#include "Stm32async.h"

#ifdef HAL_RTC_MODULE_ENABLED

namespace Stm32async
{

/**
 * @brief Class that implements real-time clock.
 */
class Rtc
{
   DECLARE_STATIC_INSTANCE(Rtc)

public:

    class EventHandler
    {
    public:
        virtual void onRtcSecondInterrupt () =0;
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
            IT_INIT_ERROR = 3,
            TIME_SET_ERROR = 4,
            DATE_SET_ERROR = 5
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

    const char * getLocalDate (char sep = '.');
    const char * getLocalTime (char sep = ':');

    inline void onMilliSecondInterrupt ()
    {
        ++upTimeMillisec;
    }

    inline void processInterrupt ()
    {
        HAL_EXT_RtcTimerIRQHandler(&rtcParameters);
    }

    inline void processEventCallback ()
    {
        ++timeSec;
        if (handler != NULL)
        {
            handler->onRtcSecondInterrupt();
        }
    }

    inline void setHandler (EventHandler * handler)
    {
        this->handler = handler;
    }

    inline EventHandler * getHandler () const
    {
        return handler;
    }

    inline HAL_StatusTypeDef getHalStatus () const
    {
        return halStatus;
    }

    inline time_t getTimeSec () const
    {
        return timeSec;
    }

    void setTimeSec (time_t timeSec)
    {
        this->timeSec = timeSec;
    }

    inline time_ms getUpTimeMillisec () const
    {
        return upTimeMillisec;
    }

private:

    RTC_HandleTypeDef rtcParameters;
    RTC_TimeTypeDef timeParameters;
    RTC_DateTypeDef dateParameters;

    HardwareLayout::Interrupt wkUpIrq;
    EventHandler * handler;
    HAL_StatusTypeDef halStatus;

    // These variables are modified from interrupt service routine, therefore declare them as volatile
    volatile time_ms upTimeMillisec; // up time (in milliseconds)
    volatile time_t timeSec; // up-time and current time (in seconds)
    char localDate[16];
    char localTime[16];
};

} // end namespace
#endif
#endif

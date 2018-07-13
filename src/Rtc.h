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
   DECLARE_STATIC_INSTANCE(Rtc)

public:

    class EventHandler
    {
    public:

        virtual void onRtcWakeUp () =0;
        virtual void onRtcSecond () =0;
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

    inline EventHandler * getHandler () const
    {
        return handler;
    }

    inline HAL_StatusTypeDef getHalStatus () const
    {
        return halStatus;
    }

    inline time_t getTime () const
    {
        if(HAL_RTC_GetTime(&rtcParameters, const_cast<RTC_TimeTypeDef *>(&timeParameters), RTC_FORMAT_BCD) != HAL_OK)
        {
            halStatus = HAL_ERROR;
        }
        return static_cast<time_t>(timeParameters.Seconds);
    }

    inline void setTime (const RTC_TimeTypeDef * time)
    {
        if(HAL_RTC_SetTime(&rtcParameters, const_cast<RTC_TimeTypeDef *>(time), RTC_FORMAT_BCD) != HAL_OK)
        {
            halStatus = HAL_ERROR;
        }
    }

    inline const RTC_DateTypeDef & getDate () const
    {
        if(HAL_RTC_GetDate(&rtcParameters, const_cast<RTC_DateTypeDef *>(&dateParameters), RTC_FORMAT_BCD) != HAL_OK)
        {
            halStatus = HAL_ERROR;
        }
        return dateParameters;
    }

    inline void setDate (const RTC_DateTypeDef * date)
    {
        if(HAL_RTC_SetDate(&rtcParameters, const_cast<RTC_DateTypeDef *>(date), RTC_FORMAT_BCD) != HAL_OK)
        {
            halStatus = HAL_ERROR;
        }
    }

private:

    mutable RTC_HandleTypeDef rtcParameters;
    mutable RTC_TimeTypeDef timeParameters;
    mutable RTC_DateTypeDef dateParameters;

    HardwareLayout::Interrupt wkUpIrq;
    EventHandler * handler;
    mutable HAL_StatusTypeDef halStatus;

    // These variables are modified from interrupt service routine, therefore declare them as volatile
    volatile time_t timeSec; // up-time and current time (in seconds)
};

} // end namespace
#endif


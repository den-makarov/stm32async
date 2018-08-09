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

#include "Rtc.h"

#ifdef HAL_RTC_MODULE_ENABLED

#include "SystemClock.h"
#include <ctime>

using namespace Stm32async;

/************************************************************************
 * Class Rtc
 ************************************************************************/

Rtc * Rtc::instance = NULL;

const char * Rtc::Start::strings[] = { "RTC started", "RTC system clock is not set. Call SystemClock::setRTC before",
                                       "Can not initialize RTC", "Can not initialize RTC WakeUpTimer interrupt",
                                       "Can not set initial time", "Can not set initial date"};

AsStringClass<Rtc::Start::Status, Rtc::Start::size, Rtc::Start::strings> Rtc::Start::asString;

Rtc::Rtc (HardwareLayout::Interrupt && _wkUpIrq) :
    wkUpIrq { std::move(_wkUpIrq) },
    handler { NULL },
    halStatus { HAL_ERROR },
    upTimeMillisec {0L},
    timeSec { 0 }
{
    rtcParameters.Instance = RTC;
    rtcParameters.Init.OutPut = 0x0;

    timeParameters.Hours = 0x0;
    timeParameters.Minutes = 0x0;
    timeParameters.Seconds = 0x0;

    dateParameters.WeekDay = RTC_WEEKDAY_MONDAY;
    dateParameters.Month = RTC_MONTH_JANUARY;
    dateParameters.Date = 01;
    dateParameters.Year = 00;

    instance = this;
}

Rtc::Start::Status Rtc::start (uint32_t counter, uint32_t prescaler)
{
    if (!SystemClock::getInstance()->isRtcActivated())
    {
        return Start::RTC_CLOCK_ERROR;
    }

    __HAL_RCC_RTC_ENABLE();

    halStatus = HAL_RTC_Init(&rtcParameters);
    if (halStatus != HAL_OK)
    {
        return Start::RTC_INIT_ERROR;
    }

    halStatus = HAL_RTC_SetTime(&rtcParameters, &timeParameters, RTC_FORMAT_BCD);
    if (halStatus != HAL_OK)
    {
        return Start::TIME_SET_ERROR;
    }

    halStatus = HAL_RTC_SetDate(&rtcParameters, &dateParameters, RTC_FORMAT_BCD);
    if (halStatus != HAL_OK)
    {
        return Start::DATE_SET_ERROR;
    }

    halStatus = HAL_EXT_SetRtcTimer_IT(&rtcParameters, counter, prescaler);
    if (halStatus != HAL_OK)
    {
        return Start::IT_INIT_ERROR;
    }

    UNUSED(counter);
    UNUSED(prescaler);

    wkUpIrq.enable();
    return Start::OK;
}

void Rtc::stop ()
{
    wkUpIrq.disable();
    HAL_EXT_DeactivateRtcTimer(&rtcParameters);
    HAL_RTC_DeInit(&rtcParameters);
    __HAL_RCC_RTC_DISABLE();
}

const char * Rtc::getLocalDate (char sep)
{
    time_t total_secs = timeSec;
    struct ::tm * now = ::gmtime(&total_secs);
    if (sep != 0)
    {
        ::sprintf(localDate, "%02d%c%02d%c%04d", now->tm_mday, sep, now->tm_mon+1, sep, now->tm_year+1900);
    }
    else
    {
        ::sprintf(localDate, "%02d%02d%04d", now->tm_mday, now->tm_mon+1, now->tm_year + FIRST_CALENDAR_YEAR);
    }
    return &localDate[0];
}

const char * Rtc::getLocalTime (char sep)
{
    time_t total_secs = timeSec;
    struct ::tm * now = ::gmtime(&total_secs);
    if (sep != 0)
    {
        ::sprintf(localTime, "%02d%c%02d%c%02d", now->tm_hour, sep, now->tm_min, sep, now->tm_sec);
    }
    else
    {
        ::sprintf(localTime, "%02d%02d%02d", now->tm_hour, now->tm_min, now->tm_sec);
    }
    return &localTime[0];
}

#endif

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
#include "SystemClock.h"

using namespace Stm32async;

/************************************************************************
 * Class Rtc
 ************************************************************************/

Rtc * Rtc::instance = NULL;

const char * Rtc::Start::strings[] = { "RTC started", "RTC system clock is not set. Call SystemClock::setRTC before",
                                       "Can not initialize RTC", "Can not initialize RTC WakeUpTimer interrupt" };

AsStringClass<Rtc::Start::Status, Rtc::Start::size, Rtc::Start::strings> Rtc::Start::asString;

Rtc::Rtc (HardwareLayout::Interrupt && _wkUpIrq) :
    wkUpIrq { std::move(_wkUpIrq) },
    handler { NULL },
    halStatus { HAL_ERROR },
    timeSec { 0 }
{
    rtcParameters.Instance = RTC;
#ifndef STM32F1
    rtcParameters.Init.HourFormat = RTC_HOURFORMAT_24;
    rtcParameters.Init.AsynchPrediv = 127;
    rtcParameters.Init.SynchPrediv = 255;
    rtcParameters.Init.OutPut = RTC_OUTPUT_DISABLE;
    rtcParameters.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
    rtcParameters.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
#else
    rtcParameters.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
    rtcParameters.Init.OutPut = RTC_OUTPUTSOURCE_NONE;
#endif

    timeParameters.Hours = 0x0;
    timeParameters.Minutes = 0x0;
    timeParameters.Seconds = 0x0;
#ifndef STM32F1
    timeParameters.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    timeParameters.StoreOperation = RTC_STOREOPERATION_RESET;
#endif

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

    HAL_RTC_SetTime(&rtcParameters, &timeParameters, RTC_FORMAT_BCD);
    HAL_RTC_SetDate(&rtcParameters, &dateParameters, RTC_FORMAT_BCD);

#ifndef STM32F1
    halStatus = HAL_RTCEx_SetWakeUpTimer_IT(&rtcParameters, counter, prescaler);
    if (halStatus != HAL_OK)
    {
        return Start::IT_INIT_ERROR;
    }
#else
    UNUSED(counter);
    UNUSED(prescaler);

    HAL_RTCEx_SetSecond_IT(&rtcParameters);
#endif

    wkUpIrq.enable();
    return Start::OK;
}

void Rtc::onSecondInterrupt ()
{
#ifdef STM32F4
    /* Get the pending status of the WAKEUPTIMER Interrupt */
    if (__HAL_RTC_WAKEUPTIMER_GET_FLAG(&rtcParameters, RTC_FLAG_WUTF) != RESET)
    {
        ++timeSec;
        if (handler != NULL)
        {
            handler->onRtcWakeUp();
        }
        /* Clear the WAKEUPTIMER interrupt pending bit */
        __HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(&rtcParameters, RTC_FLAG_WUTF);
    }

    /* Clear the EXTI's line Flag for RTC WakeUpTimer */
    __HAL_RTC_WAKEUPTIMER_EXTI_CLEAR_FLAG();
#elif defined(STM32F1)
    HAL_RTCEx_RTCIRQHandler(&rtcParameters);
#else

#endif
}

void Rtc::stop ()
{
    wkUpIrq.disable();
    #ifndef STM32F1
    HAL_RTCEx_DeactivateWakeUpTimer(&rtcParameters);
    #else
    HAL_RTCEx_DeactivateSecond(&rtcParameters);
    #endif
    HAL_RTC_DeInit(&rtcParameters);
    __HAL_RCC_RTC_DISABLE();
}


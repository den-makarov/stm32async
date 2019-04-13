/*******************************************************************************
 * stm32async: Asynchronous I/O C++ library for STM32
 * Test unit for the development board: STM32F405RGT6
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

#ifndef MYAPPLICATION_H_
#define MYAPPLICATION_H_

#include "Hardware.h"
#include "Config.h"

class MyApplication : 
	public Hardware, 
	public Drivers::WavStreamer::EventHandler, 
	public Drivers::SdCardFat::EventHandler
{
public:

    enum class EventType
    {
        SECOND_INTERRUPT = 0,
        HEARTBEAT_INTERRUPT = 2,
        ADC_TEMP_READY = 3,
        SD_CARD_ATTACHED = 5,
        SD_CARD_DEATTACHED = 6,
        SPI_CS = 7
    };

public:

    MyApplication ();

    virtual ~MyApplication () = default;

    void run (uint32_t frequency, bool exitIfFinished);

    void updateConfiguration ();

    void handleSeconds ();

    void handleHeartbeat ();

    virtual bool onStartSteaming (Drivers::AudioDac_UDA1334::SourceType s);

    virtual void onFinishSteaming ();

    void handleNtpRequest ();

    float lmt86Temperature (int mv);

    virtual void onSdCardAttach ()
    {
        scheduleEvent(MyApplication::EventType::SD_CARD_ATTACHED);
    }

    virtual void onSdCardDeAttach ()
    {
        scheduleEvent(MyApplication::EventType::SD_CARD_DEATTACHED);
    }

    inline void scheduleEvent (EventType t)
    {
        eventQueue.put(t);
    }

    void receiveSpiData();

private:

    EventQueue<EventType, 100> eventQueue;
    Config config;
    Drivers::NtpMessage ntpMessage;
    bool pendingNtpRequest;
    char messageBuffer[2048];
    float temperature;
    uint8_t inBuffer[32];
};

#endif

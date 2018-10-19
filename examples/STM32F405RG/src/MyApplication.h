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

class MyApplication : public Hardware, public Drivers::WavStreamer::EventHandler, public Drivers::Button::EventHandler
{
public:

    enum class EventType
    {
        SECOND_INTERRUPT = 0,
        ADC1_READY = 1,
        UPDATE_DISPLAY = 2,
        HEARTBEAT_INTERRUPT = 3
    };

public:

    MyApplication ();

    virtual ~MyApplication () = default;

    void run (uint32_t frequency);

    void handleSeconds ();

    void updateDisplay ();

    void initSdCard ();

    virtual bool onStartSteaming (Drivers::AudioDac_UDA1334::SourceType s);

    virtual void onFinishSteaming ();

    virtual void onButtonPressed (const Drivers::Button * b, uint32_t numOccured);

    void handleNtpRequest ();

    float lmt86Temperature (int mv);

    inline void scheduleEvent (EventType t)
    {
        eventQueue.put(t);
    }

private:

    EventQueue<EventType, 100> eventQueue;
    Config config;
    Drivers::NtpMessage ntpMessage;
    bool ntpRequestActive;
    char messageBuffer[2048];
    float temperature;
    char lcdString1[16], lcdString2[16];
};

#endif

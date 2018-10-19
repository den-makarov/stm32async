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

#include "MyApplication.h"

#include <functional>
#include <cmath>
#include <ctime>

#define USART_DEBUG_MODULE "APP: "

/************************************************************************
 * Class MyApplication
 ************************************************************************/
MyApplication::MyApplication () :
    Hardware {},
    config { "conf.txt" },
    ntpMessage {},
    ntpRequestActive { false },
    temperature { 0.0 }
{
    streamer.setHandler(this);
    stopButton.setHandler(this);
}


void MyApplication::run (uint32_t frequency)
{
    initClock(frequency);
    if (!start())
    {
        abort();
    }

    // read configuration
    config.readConfiguration();

    // Prepare ESP11
    esp.setMode(1);
    esp.setIp(config.getThisIp());
    esp.setGatway(config.getGateIp());
    esp.setMask(config.getIpMask());
    esp.setSsid(config.getWlanName());
    esp.setPasswd(config.getWlanPass());
    espSender.setRepeatDelay(config.getRepeatDelay() * MILLIS_IN_SEC);
    espSender.setTurnOffDelay(config.getTurnOffDelay() * MILLIS_IN_SEC);

    // Start WAV player
    audioDac.powerOn();
    streamer.setVolume(0.5);
    {
        char fName[16];
        ::sprintf(fName, "f%ld.wav", frequency);
        streamer.start(Drivers::AudioDac_UDA1334::SourceType::STREAM, fName);
    }

    // Finish initialization
    printResourceOccupation();

    // Start main loop
    uint32_t start = HAL_GetTick();
    uint32_t end = start + 30 * MILLIS_IN_SEC;
    ntpRequestActive = true;
    while (HAL_GetTick() < end || espSender.getEspState() != Drivers::Esp8266::AsyncCmd::OFF || streamer.isActive())
    {
        stopButton.periodic();
        streamer.periodic();
        espSender.periodic();

        if (!eventQueue.empty())
        {
            switch (eventQueue.get())
            {
            case EventType::SECOND_INTERRUPT:
                handleSeconds();
                handleNtpRequest();
                adc.read();
                break;

            case EventType::UPDATE_DISPLAY:
                updateDisplay();
                break;

            case EventType::ADC1_READY:
                temperature = lmt86Temperature(adc.getMedianMV());
                break;

            case EventType::HEARTBEAT_INTERRUPT:
                ledBlue.toggle();
                break;
            }
        }

        if (esp.getInputMessageSize() > 0)
        {
            esp.getInputMessage(messageBuffer, esp.getInputMessageSize());
            ntpMessage.decodeResponce(messageBuffer);
            ntpRequestActive = false;
        }
    }

    stop ();
}


void MyApplication::handleSeconds ()
{
    ledBlue.setLow();
    heartbeatTimer.reset();
    const char * shortTime = Rtc::getInstance()->getLocalTime(0);
    spi.waitForRelease();
    ssd.putString(shortTime, NULL, 4);
    scheduleEvent(MyApplication::EventType::UPDATE_DISPLAY);

    {
        time_t total_secs = Rtc::getInstance()->getTimeSec();
        struct ::tm * now = ::gmtime(&total_secs);
        float v0 = 2730;
        float v1 = 3080;
        dac.setValue(v0 + ((v1 - v0) * (float)now->tm_sec/60.0));
    }
}


void MyApplication::updateDisplay ()
{
    ::sprintf(lcdString1, "%10s  %02ldMH", Rtc::getInstance()->getLocalDate('.'), SystemClock::getInstance()->getMcuFreq() / 1000000);
    spi.waitForRelease();
    lcd.putString(0, 0, lcdString1, ::strlen(lcdString1));
    ::sprintf(lcdString2, "%8s    %02d%cC", Rtc::getInstance()->getLocalTime(':'), (int)temperature, 0b11110010);
    spi.waitForRelease();
    lcd.putString(0, 1, lcdString2, ::strlen(lcdString2));
}


bool MyApplication::onStartSteaming (Drivers::AudioDac_UDA1334::SourceType s)
{
    if (s == Drivers::AudioDac_UDA1334::SourceType::STREAM)
    {
        if (!sdCard.isCardInserted())
        {
            USART_DEBUG("SD Card is not inserted" << UsartLogger::ENDL);
            return false;
        }
    }
    return true;
}


void MyApplication::onFinishSteaming ()
{
    // empty
}


void MyApplication::onButtonPressed (const Drivers::Button * b, uint32_t /*numOccured*/)
{
    if (b == &stopButton)
    {
        if (streamer.isActive())
        {
            streamer.stop();
        }
    }
}


void MyApplication::handleNtpRequest ()
{
    if (ntpRequestActive && espSender.isOutputMessageSent())
    {
        espSender.sendMessage("UDP", config.getNtpServer(), "123", ntpMessage.getRequest(), Drivers::NtpMessage::NTP_PACKET_SIZE);
    }
}


float MyApplication::lmt86Temperature (int mv)
{
    return 30.0 + (10.888 - ::sqrt(10.888*10.888 + 4.0*0.00347*(1777.3 - (float)mv)))/(-2.0*0.00347) - 1;
}

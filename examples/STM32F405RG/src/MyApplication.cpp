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
    pendingNtpRequest { false },
    temperature { 0.0 }
{
    streamer.setHandler(this);
    sdCard.setHandler(this);
}


void MyApplication::run (uint32_t frequency, bool exitIfFinished)
{
    initClock(frequency);
    if (!start())
    {
        abort();
    }

    // inspect SD card
    if (!sdCard.isCardInserted())
    {
        USART_DEBUG("SD card not detected" << UsartLogger::ENDL);
        pinSdPower.setHigh();
    }

    // Start WAV player
    audioDac.powerOn();
    streamer.setVolume(0.5);

    // Finish initialization
    printResourceOccupation();

    // Start main loop
    uint32_t start = HAL_GetTick();
    uint32_t end = start + 30 * MILLIS_IN_SEC;
    while (true)
    {
        if (exitIfFinished)
        {
            if (HAL_GetTick() < end || espSender.getEspState() != Drivers::Esp8266::AsyncCmd::OFF || streamer.isActive())
            {
                break;
            }
        }

        stopButton.periodic([&](uint32_t /*numOccured*/)
        {
            if (streamer.isActive())
            {
                streamer.stop();
            }
        });

        streamer.periodic();
        sdCard.periodic();
        espSender.periodic();

        if (!eventQueue.empty())
        {
            switch (eventQueue.get())
            {
            case EventType::SECOND_INTERRUPT:
                handleSeconds();
                handleNtpRequest();
                adcTemp.read();
                break;

            case EventType::HEARTBEAT_INTERRUPT:
                handleHeartbeat();
                break;

            case EventType::ADC_TEMP_READY:
                temperature = lmt86Temperature(adcTemp.getMedianMV());
                break;
                
            case EventType::SD_CARD_ATTACHED:
                updateConfiguration();
                break;

            case EventType::SD_CARD_DEATTACHED:
                break;
            }
        }

        if (esp.getInputMessageSize() > 0)
        {
            esp.getInputMessage(messageBuffer, esp.getInputMessageSize());
            ntpMessage.decodeResponce(messageBuffer);
            pendingNtpRequest = false;
        }
    }

    stop ();
}


void MyApplication::updateConfiguration ()
{
    if (startSdCard())
    {
        config.readConfiguration();
        esp.setMode(1);
        esp.setIp(config.getThisIp());
        esp.setGatway(config.getGateIp());
        esp.setMask(config.getIpMask());
        esp.setSsid(config.getWlanName());
        esp.setPasswd(config.getWlanPass());
        espSender.setRepeatDelay(config.getRepeatDelay() * MILLIS_IN_SEC);
        espSender.setTurnOffDelay(config.getTurnOffDelay() * MILLIS_IN_SEC);
        pendingNtpRequest = true;
        if(!streamer.start(Drivers::AudioDac_UDA1334::SourceType::STREAM, config.getWavFile()))
        {
            stopSdCard();
        }
    }
}


void MyApplication::handleSeconds ()
{
    heartbeatTimer.reset();
    ledBlue.setLow();
}


void MyApplication::handleHeartbeat ()
{
    ledBlue.toggle();
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
    stopSdCard();
}


void MyApplication::handleNtpRequest ()
{
    ntpMessage.setLocalOffsetSec(3600);

    if (espSender.getEspState() == Drivers::Esp8266::AsyncCmd::OFF &&
        ntpMessage.getLastUpdateTime() > 0 &&
        Rtc::getInstance()->getTimeSec() - ntpMessage.getLastUpdateTime() > 300)
    {
        USART_DEBUG("Periodic NTP request" << UsartLogger::ENDL);
        pendingNtpRequest = true;
    }

    if (pendingNtpRequest && espSender.isOutputMessageSent())
    {
        espSender.sendMessage("UDP", config.getNtpServer(), "123", ntpMessage.getRequest(), Drivers::NtpMessage::NTP_PACKET_SIZE);
    }
}


float MyApplication::lmt86Temperature (int mv)
{
    return 25.0 + (10.888 - ::sqrt(10.888*10.888 + 4.0*0.00347*(1777.3 - (float)mv)))/(-2.0*0.00347);
}



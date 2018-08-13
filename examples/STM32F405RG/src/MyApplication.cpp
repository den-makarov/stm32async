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

#define USART_DEBUG_MODULE "APP: "

/************************************************************************
 * Class MyApplication
 ************************************************************************/
MyApplication::MyApplication () :
    Hardware {},
    config { "conf.txt" },
    fileNames { { "NOLIMIT.WAV", "S44.WAV", "S48.WAV" } },
    ntpMessage {},
    ntpRequestActive { false }
{
    streamer.setHandler(this);
    stopButton.setHandler(this);
}


MyApplication::~MyApplication ()
{
    // empty
}


void MyApplication::run (uint32_t runId, uint32_t pllp)
{
    initClock(pllp);
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
    streamer.start(Drivers::AudioDac_UDA1334::SourceType::STREAM,
                   (runId < fileNames.size()? fileNames[runId] : config.getWavFile()));

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
                if (rtc.getTimeSec() > 1000)
                {
                    adc.read();
                }
                break;

            case EventType::ADC1_READY:
                {
                    int mv = adc.getMedianMV();
                    if (rtc.getTimeSec() % 60 == 0)
                    {
                        USART_DEBUG("ADC1: " <<  mv << " mV =" << (int)(lmt86Temperature(mv)*10.0) << UsartLogger::ENDL);
                    }
                }
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
    /*
    USART_DEBUG(Rtc::getInstance()->getLocalDate() << " "
                << Rtc::getInstance()->getLocalTime()
                << ": ESP state=" << (int)espSender.getEspState()
                << ", message send=" << espSender.isOutputMessageSent()
                << UsartLogger::ENDL);
    */
    ledBlue.toggle();

    spi.waitForRelease();
    const char * shortTime = Rtc::getInstance()->getLocalTime(0);
    ssd.putString(shortTime + 2, NULL, 4);
}


bool MyApplication::onStartSteaming (Drivers::AudioDac_UDA1334::SourceType s)
{
    if (s == Drivers::AudioDac_UDA1334::SourceType::STREAM)
    {
        if (!sdCard.isCardInserted())
        {
            USART_DEBUG("SD Card is not inserted");
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
    return 30.0 + (10.888 - ::sqrt(10.888*10.888 + 4.0*0.00347*(1777.3 - (float)mv)))/(-2.0*0.00347);
}

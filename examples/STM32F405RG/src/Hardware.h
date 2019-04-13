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

#ifndef HARDWARE_H_
#define HARDWARE_H_

// Peripherie used in this projects
#include "stm32async/HardwareLayout/Dma1.h"
#include "stm32async/HardwareLayout/Dma2.h"
#include "stm32async/HardwareLayout/PortA.h"
#include "stm32async/HardwareLayout/PortB.h"
#include "stm32async/HardwareLayout/PortC.h"
#include "stm32async/HardwareLayout/PortD.h"
#include "stm32async/HardwareLayout/PortH.h"
#include "stm32async/HardwareLayout/Usart1.h"
#include "stm32async/HardwareLayout/Usart2.h"
#include "stm32async/HardwareLayout/Spi1.h"
#include "stm32async/HardwareLayout/Sdio1.h"
#include "stm32async/HardwareLayout/I2S2.h"
#include "stm32async/HardwareLayout/Adc1.h"
#include "stm32async/HardwareLayout/Dac1.h"
#include "stm32async/HardwareLayout/Timer3.h"

#include "stm32async/SystemClock.h"
#include "stm32async/Rtc.h"
#include "stm32async/IOPort.h"
#include "stm32async/Adc.h"
#include "stm32async/Dac.h"
#include "stm32async/UsartLogger.h"
#include "stm32async/EventQueue.h"
#include "stm32async/Timer.h"

#include "stm32async/Drivers/Button.h"
#include "stm32async/Drivers/Led.h"
#include "stm32async/Drivers/Ssd.h"
#include "stm32async/Drivers/SdCardFat.h"
#include "stm32async/Drivers/Esp8266.h"
#include "stm32async/Drivers/WavStreamer.h"
#include "stm32async/Drivers/Lcd_DOGM162.h"

using namespace Stm32async;

/**
 * @brief A class collecting clock parameters for STM32F405
 */
class ClockParameters
{
public:
    int M, N, Q, P, APB1, APB2;
    float PLLMout, PLLNout, clock48out, SYSCLC;
    ClockParameters ():
        M {0}, N {0}, Q {0}, P {0}, APB1{1}, APB2 {1},
        PLLMout {0.0},
        PLLNout {0.0},
        clock48out {0.0},
        SYSCLC {0.0}
    {
        // empty
    }

    void assign (const ClockParameters & f);
    void print ();
    void searchBestParameters (uint32_t targetFreq);
};


/**
 * @brief A class providing the map of used hardware
 */
class Hardware
{
public:

    const float V_REF = 3.256f;
    static const uint16_t SPI_CS_PIN = GPIO_PIN_4;

    // Used ports
    HardwareLayout::PortA portA;
    HardwareLayout::PortB portB;
    HardwareLayout::PortC portC;
    HardwareLayout::PortD portD;
    HardwareLayout::PortH portH;

    // DMA
    HardwareLayout::Dma1 dma1;
    HardwareLayout::Dma2 dma2;

    // System and MCO
    ClockParameters clockParameters;
    SystemClock sysClock;
    Rtc rtc;
    MCO mco;

    // LEDs
    Drivers::Led ledGreen;
    Drivers::Led ledBlue;
    Drivers::Led ledRed;

    // SPI
    IOPort spiCs;
    HardwareLayout::Interrupt extCsInterrupt;
    HardwareLayout::Spi1 spi1;
    AsyncSpi spi;

    // SD Card
    HardwareLayout::Sdio1 sdio1;
    IOPort pinSdPower, pinSdDetect;
    Drivers::SdCardFat sdCard;

    // ESP
    HardwareLayout::Usart2 usart2;
    Drivers::Esp8266 esp;
    Drivers::EspSender espSender;

    // I2S2 Audio
    HardwareLayout::I2S2 i2s2;
    AsyncI2S i2s;
    Drivers::AudioDac_UDA1334 audioDac;
    Drivers::WavStreamer streamer;
    Drivers::Button stopButton;

    // ADC/DAC
    HardwareLayout::Adc1 adc1;
    AsyncAdc adcTemp;
    HardwareLayout::Dac1 dac1;
    BaseDac dac;

    // Timer
    HardwareLayout::Timer3 timer3;
    InterruptTimer heartbeatTimer;

    // USART logger
    HardwareLayout::Usart1 usart1;
    UsartLogger usartLogger;

    // Test pin
    IOPort testPin;

    Hardware ();

    void abort ();
    void initClock (uint32_t frequency);
    bool start ();
    void stop ();
    bool startSdCard ();
    void stopSdCard ();
    void printResourceOccupation ();

    inline Drivers::Led & getLedRed ()
    {
        return ledRed;
    }

    inline AsyncUsart & getLoggerUsart ()
    {
        return usartLogger.getUsart();
    }

    inline AsyncSpi & getSpi ()
    {
        return spi;
    }

    inline Sdio & getSdio ()
    {
        return sdCard.getSdio();
    }

    inline Drivers::Esp8266 & getEsp ()
    {
        return esp;
    }

    inline AsyncI2S & getI2S ()
    {
        return i2s;
    }

    inline AsyncAdc & getAdcTemp ()
    {
        return adcTemp;
    }

    inline InterruptTimer & getHeartbeatTimer ()
    {
        return heartbeatTimer;
    }
};

#endif

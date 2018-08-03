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

// Used devices
#include "stm32async/SystemClock.h"
#include "stm32async/Rtc.h"
#include "stm32async/IOPort.h"
#include "stm32async/SdCard.h"
#include "stm32async/UsartLogger.h"
#include "stm32async/Drivers/Ssd.h"
#include "stm32async/Drivers/Esp8266.h"

// Common includes
#include <functional>
#include "Config.h"

using namespace Stm32async;

#define USART_DEBUG_MODULE "Main: "

class MyApplication : public Rtc::EventHandler
{
private:

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
    SystemClock sysClock;
    Rtc rtc;
    MCO mco;

    // LEDs
    IOPort ledBlue, ledRed;

    // SPI
    HardwareLayout::Spi1 spi1;
    AsyncSpi spi;
    Drivers::Ssd_74XX595 ssd;

    // SD Card
    HardwareLayout::Sdio1 sdio1;
    IOPort pinSdPower, pinSdDetect;
    SdCard sdCard;
    Config config;

    // ESP
    HardwareLayout::Usart2 usart2;
    Drivers::Esp8266 esp;
    Drivers::EspSender espSender;
    bool ntpRequestActive;

    // NTP Message
    static const size_t NTP_PACKET_SIZE = 48;  // NTP time is in the first 48 bytes of message
    struct NtpPacket {
            uint8_t flags;
            uint8_t stratum;
            uint8_t poll;
            uint8_t precision;
            uint32_t root_delay;
            uint32_t root_dispersion;
            uint8_t referenceID[4];
            uint32_t ref_ts_sec;
            uint32_t ref_ts_frac;
            uint32_t origin_ts_sec;
            uint32_t origin_ts_frac;
            uint32_t recv_ts_sec;
            uint32_t recv_ts_frac;
            uint32_t trans_ts_sec;
            uint32_t trans_ts_frac;
    } __attribute__((__packed__));

    struct NtpPacket ntpPacket;
    char messageBuffer[2048];

    // USART logger
    HardwareLayout::Usart1 usart1;
    UsartLogger usartLogger;

public:

    MyApplication () :
        // System, RTC and MCO
        sysClock { HardwareLayout::Interrupt { SysTick_IRQn, 0, 0 } },
        rtc { HardwareLayout::Interrupt { RTC_WKUP_IRQn, 15, 0 } },
        mco { portA, GPIO_PIN_8, RCC_MCO1SOURCE_PLLCLK, RCC_MCODIV_5 },

        // LEDs
        ledBlue { portC, GPIO_PIN_2, GPIO_MODE_OUTPUT_PP },
        ledRed { portC, GPIO_PIN_3, GPIO_MODE_OUTPUT_PP },

        // SPI
        spi1 { portA, GPIO_PIN_5, portA, GPIO_PIN_7, portA, GPIO_PIN_6, /*remapped=*/ true, NULL,
                 HardwareLayout::Interrupt { SPI1_IRQn, 1, 0 },
                 HardwareLayout::DmaStream { &dma2, DMA2_Stream5, DMA_CHANNEL_3,
                                             HardwareLayout::Interrupt { DMA2_Stream5_IRQn, 2, 0 } },
                 HardwareLayout::DmaStream { &dma2, DMA2_Stream2, DMA_CHANNEL_3,
                                             HardwareLayout::Interrupt { DMA2_Stream2_IRQn, 2, 0 } }
        },
        spi { spi1 },
        ssd { spi, portC, GPIO_PIN_4, true },

        // SD card
        sdio1 {
                portC, /*SDIO_D0*/GPIO_PIN_8 | /*SDIO_D1*/GPIO_PIN_9 | /*SDIO_D2*/GPIO_PIN_10 | /*SDIO_D3*/GPIO_PIN_11 | /*SDIO_CK*/GPIO_PIN_12,
                portD, /*SDIO_CMD*/GPIO_PIN_2,
                HardwareLayout::Interrupt { SDIO_IRQn, 3, 0 },
                HardwareLayout::DmaStream { &dma2, DMA2_Stream6, DMA_CHANNEL_4,
                                            HardwareLayout::Interrupt { DMA2_Stream6_IRQn, 4, 0 } },
                HardwareLayout::DmaStream { &dma2, DMA2_Stream3, DMA_CHANNEL_4,
                                            HardwareLayout::Interrupt { DMA2_Stream3_IRQn, 5, 0 } }
        },
        pinSdPower { portA, GPIO_PIN_15, GPIO_MODE_OUTPUT_PP, GPIO_PULLUP },
        pinSdDetect { portB, GPIO_PIN_3, GPIO_MODE_INPUT, GPIO_PULLUP },
        sdCard { sdio1, pinSdDetect, /*clockDiv=*/ 2 },
        config("conf.txt"),

        //ESP
        usart2 { portA, GPIO_PIN_2, portA, GPIO_PIN_3, /*remapped=*/ true, NULL,
                 HardwareLayout::Interrupt { USART2_IRQn, 6, 0 },
                 HardwareLayout::DmaStream { &dma1, DMA1_Stream6, DMA_CHANNEL_4,
                                             HardwareLayout::Interrupt { DMA1_Stream6_IRQn, 7, 0 } },
                 HardwareLayout::DmaStream { &dma1, DMA1_Stream5, DMA_CHANNEL_4,
                                             HardwareLayout::Interrupt { DMA1_Stream5_IRQn, 8, 0 } }
        },
        esp { usart2, portA, GPIO_PIN_1 },
        espSender { esp, ledRed },
        ntpRequestActive { false },

        // USART logger
        usart1 { portB, GPIO_PIN_6, portB, GPIO_PIN_7, /*remapped=*/ true, NULL,
                 HardwareLayout::Interrupt { USART1_IRQn, 13, 0 },
                 HardwareLayout::DmaStream { &dma2, DMA2_Stream7, DMA_CHANNEL_4,
                                             HardwareLayout::Interrupt { DMA2_Stream7_IRQn, 14, 0 } },
                 HardwareLayout::DmaStream { &dma2, DMA2_Stream2, DMA_CHANNEL_4,
                                             HardwareLayout::Interrupt { DMA2_Stream2_IRQn, 14, 0 } }
        },
        usartLogger { usart1, 115200 }
    {
        // External oscillators use system pins
        sysClock.setHSE(&portH, GPIO_PIN_0 | GPIO_PIN_1);
        sysClock.setLSE(&portC, GPIO_PIN_14 | GPIO_PIN_15);
        // Prepare SSD mask
        Drivers::Ssd_74XX595::SegmentsMask sm;
        sm.top = 3;
        sm.rightTop = 5;
        sm.rightBottom = 7;
        sm.bottom = 4;
        sm.leftBottom = 1;
        sm.leftTop = 2;
        sm.center = 6;
        sm.dot = 0;
        ssd.setSegmentsMask(sm);
    }

    virtual ~MyApplication ()
    {
        // empty
    }

    void initClock (uint32_t pllp)
    {
        SystemClock::getInstance()->setSysClockSource(RCC_SYSCLKSOURCE_PLLCLK);        
        HardwareLayout::SystemPllFactors pllConfig;
        pllConfig.PLLM = 16;
        pllConfig.PLLN = 336;
        pllConfig.PLLP = pllp;
        pllConfig.PLLQ = 7;
        SystemClock::getInstance()->setPLL(&pllConfig);
        SystemClock::getInstance()->setAHB(RCC_SYSCLK_DIV1, RCC_HCLK_DIV8, RCC_HCLK_DIV8);
        SystemClock::getInstance()->setLatency(FLASH_LATENCY_3);
        SystemClock::getInstance()->setRTC();
        SystemClock::getInstance()->start();
    }

    void run (uint32_t runId, uint32_t pllp)
    {
        initClock(pllp);
        usartLogger.initInstance();

        USART_DEBUG("--------------------------------------------------------" << UsartLogger::ENDL
                    << "Oscillator frequency: " << SystemClock::getInstance()->getHSEFreq()
                    << ", MCU frequency: " << SystemClock::getInstance()->getMcuFreq() << UsartLogger::ENDL
                    << UsartLogger::TAB << "runId=" << runId << UsartLogger::ENDL
                    << UsartLogger::TAB << "pllp=" << pllp << UsartLogger::ENDL);

        // For RTC, it is necessary to reset the state since it will not be
        // automatically reset after MCU programming.
        rtc.stop();
        rtc.setHandler(this);
        do
        {
            Rtc::Start::Status status = rtc.start(8 * 2047 + 7, RTC_WAKEUPCLOCK_RTCCLK_DIV2);
            USART_DEBUG("RTC status: " << Rtc::Start::asString(status) << " (" << rtc.getHalStatus() << ")" << UsartLogger::ENDL);
        }
        while (rtc.getHalStatus() != HAL_OK);

        mco.start();
        ledBlue.start();
        ledRed.start();

        DeviceStart::Status devStatus = spi.start(SPI_DIRECTION_1LINE, SPI_BAUDRATEPRESCALER_64, SPI_DATASIZE_8BIT, SPI_PHASE_2EDGE);
        USART_DEBUG("SPI1 status: " << DeviceStart::asString(devStatus) << " (" << spi.getHalStatus() << ")" << UsartLogger::ENDL);
        ssd.start();

        ledRed.setHigh();
        HAL_Delay(500);
        ledRed.setLow();

        pinSdDetect.start();
        if (sdCard.isCardInserted())
        {
            initSdCard();
        }

        // Prepare ESP11
        esp.setMode(1);
        esp.setIp(config.getThisIp());
        esp.setGatway(config.getGateIp());
        esp.setMask(config.getIpMask());
        esp.setSsid(config.getWlanName());
        esp.setPasswd(config.getWlanPass());
        espSender.setRepeatDelay(config.getRepeatDelay() * MILLIS_IN_SEC);
        espSender.setTurnOffDelay(config.getTurnOffDelay() * MILLIS_IN_SEC);

        uint32_t start = HAL_GetTick();
        uint32_t end = start + 30 * MILLIS_IN_SEC;

        ntpRequestActive = true;
        while (HAL_GetTick() < end)
        {
            __NOP();

            espSender.periodic();

            if (esp.getInputMessageSize() > 0)
            {
                esp.getInputMessage(messageBuffer, esp.getInputMessageSize());
                ::memcpy(&ntpPacket, messageBuffer, NTP_PACKET_SIZE);
                decodeNtpMessage(ntpPacket);
                ntpRequestActive = false;
            }
        }

        sdCard.stop();
        pinSdPower.setHigh();
        pinSdPower.stop();
        pinSdDetect.stop();

        ssd.stop();
        spi.stop();
        ledBlue.stop();
        ledRed.stop();
        mco.stop();
        rtc.stop();

        // Log resource occupations after all devices (expect USART1 for logging, HSE, LSE) are stopped
        // Desired: two at portB and DMA2 (USART1), one for portC (LSE), one for portH (HSE)
        USART_DEBUG("Resource occupations: " << UsartLogger::ENDL
                    << UsartLogger::TAB << "portA=" << portA.getObjectsCount() << UsartLogger::ENDL
                    << UsartLogger::TAB << "portB=" << portB.getObjectsCount() << UsartLogger::ENDL
                    << UsartLogger::TAB << "portC=" << portC.getObjectsCount() << UsartLogger::ENDL
                    << UsartLogger::TAB << "portD=" << portD.getObjectsCount() << UsartLogger::ENDL
                    << UsartLogger::TAB << "portH=" << portH.getObjectsCount() << UsartLogger::ENDL
                    << UsartLogger::TAB << "dma1=" << dma1.getObjectsCount() << UsartLogger::ENDL
                    << UsartLogger::TAB << "dma2=" << dma2.getObjectsCount() << UsartLogger::ENDL);
        usartLogger.clearInstance();

        sysClock.stop();
    }

    inline AsyncUsart & getLoggerUsart ()
    {
        return usartLogger.getUsart();
    }

    inline AsyncSpi & getSpi ()
    {
        return spi;
    }

    inline Drivers::Esp8266 & getEsp ()
    {
        return esp;
    }

    void onRtcSecondInterrupt ()
    {
        USART_DEBUG(Rtc::getInstance()->getLocalDate() << " "
                    << Rtc::getInstance()->getLocalTime()
                    << ": ESP state=" << (int)espSender.getEspState()
                    << ", message send=" << espSender.isOutputMessageSent()
                    << UsartLogger::ENDL);
        ledBlue.toggle();

        spi.waitForRelease();
        const char * shortTime = Rtc::getInstance()->getLocalTime(0);
        ssd.putString(shortTime + 2, NULL, 4);

        handleNtpRequest();
    }

    void initSdCard ()
    {
        pinSdPower.start();
        pinSdPower.setLow();
        HAL_Delay(250);
        DeviceStart::Status devStatus = sdCard.start();
        USART_DEBUG("SD card status: " << DeviceStart::asString(devStatus) << " (" << sdCard.getHalStatus() << ")" << UsartLogger::ENDL);
        if (devStatus == DeviceStart::OK)
        {
            USART_DEBUG("CardType = " << sdCard.getCardInfo().CardType << UsartLogger::ENDL
                     << UsartLogger::TAB << "CardCapacity = " << sdCard.getCardInfo().CardCapacity/1024L/1024L << "Mb" << UsartLogger::ENDL
                     << UsartLogger::TAB << "CardBlockSize = " << sdCard.getCardInfo().CardBlockSize << UsartLogger::ENDL
                     << UsartLogger::TAB << "DAT_BUS_WIDTH = " << sdCard.getCardStatus().DAT_BUS_WIDTH << UsartLogger::ENDL
                     << UsartLogger::TAB << "SD_CARD_TYPE = " << sdCard.getCardStatus().SD_CARD_TYPE << UsartLogger::ENDL
                     << UsartLogger::TAB << "SPEED_CLASS = " << sdCard.getCardStatus().SPEED_CLASS << UsartLogger::ENDL);
            devStatus = sdCard.mountFatFs();
            USART_DEBUG("FAT FS card status: " << DeviceStart::asString(devStatus) << " (" << sdCard.getHalStatus() << ")" << UsartLogger::ENDL);
            if (devStatus == DeviceStart::OK)
            {
                USART_DEBUG("label = " << sdCard.getFatFs().volumeLabel << UsartLogger::ENDL
                            << UsartLogger::TAB << "serial number = " << sdCard.getFatFs().volumeSN << UsartLogger::ENDL
                            << UsartLogger::TAB << "current directory = " << sdCard.getFatFs().currentDirectory << UsartLogger::ENDL);
                sdCard.listFiles();
                config.readConfiguration();
            }
        }
    }

    void handleNtpRequest ()
    {
        if (ntpRequestActive && espSender.isOutputMessageSent())
        {
            fillNtpRrequst(ntpPacket);
            espSender.sendMessage("UDP", config.getNtpServer(), "123", (const char *)(&ntpPacket), NTP_PACKET_SIZE);
        }
    }

    void fillNtpRrequst (NtpPacket & ntpPacket)
    {
        ::memset(&ntpPacket, 0, NTP_PACKET_SIZE);
        ntpPacket.flags = 0xe3;
    }

    #define UNIX_OFFSET             2208988800L
    #define ENDIAN_SWAP32(data)     ((data >> 24) | /* right shift 3 bytes */ \
                                    ((data & 0x00ff0000) >> 8) | /* right shift 1 byte */ \
                                    ((data & 0x0000ff00) << 8) | /* left shift 1 byte */ \
                                    ((data & 0x000000ff) << 24)) /* left shift 3 bytes */

    void decodeNtpMessage (NtpPacket & ntpPacket)
    {
        ntpPacket.root_delay = ENDIAN_SWAP32(ntpPacket.root_delay);
        ntpPacket.root_dispersion = ENDIAN_SWAP32(ntpPacket.root_dispersion);
        ntpPacket.ref_ts_sec = ENDIAN_SWAP32(ntpPacket.ref_ts_sec);
        ntpPacket.ref_ts_frac = ENDIAN_SWAP32(ntpPacket.ref_ts_frac);
        ntpPacket.origin_ts_sec = ENDIAN_SWAP32(ntpPacket.origin_ts_sec);
        ntpPacket.origin_ts_frac = ENDIAN_SWAP32(ntpPacket.origin_ts_frac);
        ntpPacket.recv_ts_sec = ENDIAN_SWAP32(ntpPacket.recv_ts_sec);
        ntpPacket.recv_ts_frac = ENDIAN_SWAP32(ntpPacket.recv_ts_frac);
        ntpPacket.trans_ts_sec = ENDIAN_SWAP32(ntpPacket.trans_ts_sec);
        ntpPacket.trans_ts_frac = ENDIAN_SWAP32(ntpPacket.trans_ts_frac);
        time_t total_secs = ntpPacket.recv_ts_sec - UNIX_OFFSET; /* convert to unix time */;
        Rtc::getInstance()->setTimeSec(total_secs);
        USART_DEBUG("NTP time: " << Rtc::getInstance()->getLocalTime() << UsartLogger::ENDL);
    }

};

MyApplication * appPtr = NULL;

int main (void)
{
    // Note: check the Value of the External oscillator mounted in PCB
    // and set this value in the file stm32f4xx_hal_conf.h
    HAL_Init();

    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    /* STM32F405x/407x/415x/417x Revision Z devices: prefetch is supported  */
    if (HAL_GetREVID() == 0x1001)
    {
        /* Enable the Flash prefetch */
        __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
    }

    MyApplication app;
    appPtr = &app;

    uint32_t pllp = 2, runId = 0;
    while (true)
    {
        app.run(++runId, pllp);
        pllp += 2;
        if (pllp > 8)
        {
            pllp = 2;
        }
    }
}

extern "C"
{

// System
void SysTick_Handler (void)
{
    HAL_IncTick();
    if (Rtc::getInstance() != NULL)
    {
        Rtc::getInstance()->onMilliSecondInterrupt();
    }
}

void RTC_WKUP_IRQHandler ()
{
    if (Rtc::getInstance() != NULL)
    {
        Rtc::getInstance()->processInterrupt();
    }
}

void HAL_RTCEx_WakeUpTimerEventCallback (RTC_HandleTypeDef * /*hrtc*/)
{
    if (Rtc::getInstance() != NULL)
    {
        Rtc::getInstance()->processEventCallback();
    }
}

// UARTs: uses both USART and DMA interrupts
void DMA2_Stream7_IRQHandler (void)
{
    appPtr->getLoggerUsart().processDmaTxInterrupt();
}

void USART1_IRQHandler (void)
{
    appPtr->getLoggerUsart().processInterrupt();
}

void USART2_IRQHandler (void)
{
    appPtr->getEsp().processInterrupt();
}

void HAL_UART_TxCpltCallback (UART_HandleTypeDef * channel)
{
    if (channel->Instance == USART1)
    {
        appPtr->getLoggerUsart().processCallback(SharedDevice::State::TX_CMPL);
    }
    else if (channel->Instance == USART2)
    {
        appPtr->getEsp().processTxCpltCallback();
    }
}

void HAL_UART_RxCpltCallback (UART_HandleTypeDef * channel)
{
    if (channel->Instance == USART1)
    {
        appPtr->getLoggerUsart().processCallback(SharedDevice::State::RX_CMPL);
    }
    else if (channel->Instance == USART2)
    {
        appPtr->getEsp().processRxCpltCallback();
    }
}

void HAL_UART_ErrorCallback (UART_HandleTypeDef * channel)
{
    if (channel->Instance == USART1)
    {
        appPtr->getLoggerUsart().processCallback(SharedDevice::State::ERROR);
    }
    else if (channel->Instance == USART2)
    {
        appPtr->getEsp().processErrorCallback();
    }
}

// SPI
void DMA2_Stream5_IRQHandler (void)
{
    appPtr->getSpi().processDmaTxInterrupt();
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef * /*channel*/)
{
    appPtr->getSpi().processCallback(SharedDevice::State::TX_CMPL);
}

// SD card
void DMA2_Stream3_IRQHandler (void)
{
    SdCard::getInstance()->processDmaRxInterrupt();
}

void DMA2_Stream6_IRQHandler (void)
{
    SdCard::getInstance()->processDmaTxInterrupt();
}

void SDIO_IRQHandler (void)
{
    SdCard::getInstance()->processSdIOInterrupt();
}

}


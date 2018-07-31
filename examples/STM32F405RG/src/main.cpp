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
#include "stm32async/HardwareLayout/Spi1.h"
#include "stm32async/HardwareLayout/Sdio1.h"

// Used devices
#include "stm32async/SystemClock.h"
#include "stm32async/Rtc.h"
#include "stm32async/IOPort.h"
#include "stm32async/SdCard.h"
#include "stm32async/UsartLogger.h"
#include "stm32async/Drivers/Ssd.h"

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

    // USART logger
    HardwareLayout::Usart1 usart1;
    UsartLogger usartLogger;

public:

    MyApplication () :
        // System, RTC and MCO
        sysClock { HardwareLayout::Interrupt { SysTick_IRQn, 0, 0 } },
        rtc { HardwareLayout::Interrupt { RTC_WKUP_IRQn, 10, 0 } },
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
        // USART logger
        usart1 { portB, GPIO_PIN_6, portB, GPIO_PIN_7, /*remapped=*/ true, NULL,
                 HardwareLayout::Interrupt { USART1_IRQn, 6, 0 },
                 HardwareLayout::DmaStream { &dma2, DMA2_Stream7, DMA_CHANNEL_4,
                                             HardwareLayout::Interrupt { DMA2_Stream7_IRQn, 7, 0 } },
                 HardwareLayout::DmaStream { &dma2, DMA2_Stream2, DMA_CHANNEL_4,
                                             HardwareLayout::Interrupt { DMA2_Stream2_IRQn, 7, 0 } }
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

        for (int i = 0; i < 15; ++i)
        {
            // main loop empty for 15 sec
            HAL_Delay(1000);
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

    void onRtcSecondInterrupt ()
    {
        USART_DEBUG(Rtc::getInstance()->getLocalDate() << " "
                    << Rtc::getInstance()->getLocalTime() << UsartLogger::ENDL);
        ledBlue.toggle();

        spi.waitForRelease();
        const char * shortTime = Rtc::getInstance()->getLocalTime(0);
        ssd.putString(shortTime + 2, NULL, 4);
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

// UART: uses both USART and DMA interrupts
void DMA2_Stream7_IRQHandler (void)
{
    appPtr->getLoggerUsart().processDmaTxInterrupt();
}

void USART1_IRQHandler (void)
{
    appPtr->getLoggerUsart().processInterrupt();
}

void HAL_UART_TxCpltCallback (UART_HandleTypeDef * /*channel*/)
{
    appPtr->getLoggerUsart().processCallback(SharedDevice::State::TX_CMPL);
}

void HAL_UART_ErrorCallback (UART_HandleTypeDef * /*channel*/)
{
    appPtr->getLoggerUsart().processCallback(SharedDevice::State::ERROR);
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


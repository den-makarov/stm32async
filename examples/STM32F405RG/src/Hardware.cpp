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

#include "Hardware.h"
#include "MyApplication.h"
#include <cmath>

#define USART_DEBUG_MODULE "HRDW: "


/************************************************************************
 * Class ClockParameters
 ************************************************************************/
void ClockParameters::assign (const ClockParameters & f)
{
    M          = f.M;
    N          = f.N;
    Q          = f.Q;
    P          = f.P;
    PLLMout    = f.PLLMout;
    PLLNout    = f.PLLNout;
    clock48out = f.clock48out;
    SYSCLC     = f.SYSCLC;
    APB1       = f.APB1;
    APB2       = f.APB2;
}

void ClockParameters::print ()
{
    USART_DEBUG("HSE=" << HSE_VALUE << " M=" << M << " N=" << N << " Q=" << Q << " P=" << P <<
                " SYSCLC=" << SYSCLC << " APB1=" << APB1 << " APB2=" << APB2 << UsartLogger::ENDL);
}

void ClockParameters::searchBestParameters (uint32_t targetFreq)
{
    int maxM = 0;
    float bestDiff = 9999.0;
    float hse = HSE_VALUE / 1000000;
    ClockParameters bestFactors;
    for (int M = 2; M <= 63; ++M)
    {
        ClockParameters f;
        f.M = M;
        f.PLLMout = hse / (float)f.M;
        if (f.PLLMout >= 0.95 && f.PLLMout <= 2.1)
        {
            for (int N = 50; N <= 432; ++N)
            {
                f.N = N;
                f.PLLNout = hse * (float)f.N / (float)f.M;
                if (f.PLLNout >= 100.0 && f.PLLNout <= 432.0)
                {
                    for (int Q = 2; Q <= 15; ++Q)
                    {
                        f.Q = Q;
                        f.clock48out = f.PLLNout / (float)f.Q;
                        int clock48out = (int)f.clock48out;
                        if (f.clock48out - (float)clock48out == 0.0 && clock48out == 48)
                        {
                            for (int P = 2; P <= 8; P+=2)
                            {
                                f.P = P;
                                f.SYSCLC = f.PLLNout / (float)f.P;
                                if (f.SYSCLC >= 24.0 && f.SYSCLC <= 168.0)
                                {
                                    float diff = ::fabs(f.SYSCLC - targetFreq);
                                    if (diff < bestDiff || (diff == bestDiff && f.M > maxM))
                                    {
                                        maxM = f.M;
                                        bestDiff = diff;
                                        bestFactors.assign(f);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    // Adjust APB prescalers
    while (true)
    {
        float pclk1 = bestFactors.SYSCLC / (float) bestFactors.APB1;
        if (pclk1 <= 42 || bestFactors.APB1 == 16)
        {
            break;
        }
        bestFactors.APB1 *= 2;
    }
    while (true)
    {
        float pclk1 = bestFactors.SYSCLC / (float) bestFactors.APB2;
        if (pclk1 <= 84 || bestFactors.APB2 == 16)
        {
            break;
        }
        bestFactors.APB2 *= 2;
    }
    assign(bestFactors);
}


/************************************************************************
 * Class Hardware
 ************************************************************************/
Hardware::Hardware ():
    // System, RTC and MCO
    sysClock { HardwareLayout::Interrupt { SysTick_IRQn, 0 } },
    rtc { HardwareLayout::Interrupt { RTC_WKUP_IRQn, 15 } },
    mco { portA, GPIO_PIN_8, RCC_MCO1SOURCE_PLLCLK, RCC_MCODIV_5 },

    // LEDs
    ledGreen { portB, GPIO_PIN_4, Drivers::Led::ConnectionType::CATHODE },
    ledBlue { portB, GPIO_PIN_5, Drivers::Led::ConnectionType::CATHODE },
    ledRed { portB, GPIO_PIN_8, Drivers::Led::ConnectionType::CATHODE },

    // SPI
    spi1 { portA, GPIO_PIN_5, portA, GPIO_PIN_7, portA, UNUSED_PIN, /*remapped=*/ true, NULL,
             HardwareLayout::Interrupt { SPI1_IRQn, 1, 0 },
             HardwareLayout::DmaStream { &dma2, DMA2_Stream5, DMA_CHANNEL_3,
                                         HardwareLayout::Interrupt { DMA2_Stream5_IRQn, 1, 1 } },
             HardwareLayout::DmaStream { &dma2, DMA2_Stream2, DMA_CHANNEL_3,
                                         HardwareLayout::Interrupt { DMA2_Stream2_IRQn, 1, 2 } }
    },
    spi { spi1, GPIO_NOPULL },
    ssd { spi, portA, GPIO_PIN_6, true },
    lcd { spi, portC, GPIO_PIN_4, portC, GPIO_PIN_5, false, 63 },

    // SD card
    sdio1 { portC, /*SDIO_D0*/GPIO_PIN_8 | /*SDIO_D1*/GPIO_PIN_9 | /*SDIO_D2*/GPIO_PIN_10 | /*SDIO_D3*/GPIO_PIN_11 | /*SDIO_CK*/GPIO_PIN_12,
            portD, /*SDIO_CMD*/GPIO_PIN_2,
            HardwareLayout::Interrupt { SDIO_IRQn, 2 },
            HardwareLayout::DmaStream { &dma2, DMA2_Stream6, DMA_CHANNEL_4,
                                        HardwareLayout::Interrupt { DMA2_Stream6_IRQn, 3, 0 } },
            HardwareLayout::DmaStream { &dma2, DMA2_Stream3, DMA_CHANNEL_4,
                                        HardwareLayout::Interrupt { DMA2_Stream3_IRQn, 3, 1 } }
    },
    pinSdPower { portA, GPIO_PIN_15, GPIO_MODE_OUTPUT_PP, GPIO_PULLUP },
    pinSdDetect { portB, GPIO_PIN_3, GPIO_MODE_INPUT, GPIO_PULLUP },
    sdCard { sdio1, pinSdDetect, /*clockDiv=*/ 2 },

    //ESP
    usart2 { portA, GPIO_PIN_2, portA, GPIO_PIN_3, /*remapped=*/ true, NULL,
             HardwareLayout::Interrupt { USART2_IRQn, 4, 0 },
             HardwareLayout::DmaStream { &dma1, DMA1_Stream6, DMA_CHANNEL_4,
                                         HardwareLayout::Interrupt { DMA1_Stream6_IRQn, 5, 0 } },
             HardwareLayout::DmaStream { &dma1, DMA1_Stream5, DMA_CHANNEL_4,
                                         HardwareLayout::Interrupt { DMA1_Stream5_IRQn, 5, 1 } }
    },
    esp { usart2, portA, GPIO_PIN_1, /*sendLed=*/ &ledGreen },
    espSender { esp, /*errorLed=*/ &ledRed },

    // I2S2 Audio
    i2s2 { portB, /*I2S2_CK*/GPIO_PIN_10 | /*I2S2_WS*/GPIO_PIN_12 | /*I2S2_SD*/GPIO_PIN_15,
          /*remapped=*/ true, NULL,
          HardwareLayout::DmaStream { &dma1, DMA1_Stream4, DMA_CHANNEL_0,
                                      HardwareLayout::Interrupt { DMA1_Stream4_IRQn, 6 } },
          HardwareLayout::DmaStream { &dma1, DMA1_Stream3, DMA_CHANNEL_0,
                                      HardwareLayout::Interrupt { DMA1_Stream3_IRQn, 6 } }
    },
    i2s { i2s2 },
    audioDac { i2s,
              /* power    = */ portB, GPIO_PIN_11,
              /* mute     = */ portB, GPIO_PIN_13,
              /* smplFreq = */ portB, GPIO_PIN_14 },
    streamer { sdCard, audioDac },
    stopButton { portB, GPIO_PIN_9, GPIO_PULLUP },

    // ADC/DAC
    adc1 { portA, GPIO_PIN_0, /*remapped=*/ false, NULL,
        HardwareLayout::DmaStream { &dma2, DMA2_Stream0, DMA_CHANNEL_0,
                                    HardwareLayout::Interrupt { DMA2_Stream0_IRQn, 7 } }
    },
    adcTemperature { adc1, /*channel=*/ 0, ADC_SAMPLETIME_144CYCLES },
    dac1 { portA, GPIO_PIN_4 },
    dac { dac1, DAC_CHANNEL_1 },

    // Timers
    timer3 { HardwareLayout::Interrupt { TIM3_IRQn, 8, 0 } },
    heartbeatTimer { timer3 },

    // USART logger
    usart1 { portB, GPIO_PIN_6, portB, UNUSED_PIN, /*remapped=*/ true, NULL,
             HardwareLayout::Interrupt { USART1_IRQn, 13, 0 },
             HardwareLayout::DmaStream { &dma2, DMA2_Stream7, DMA_CHANNEL_4,
                                         HardwareLayout::Interrupt { DMA2_Stream7_IRQn, 14 } },
             HardwareLayout::DmaStream { &dma2, DMA2_Stream2, DMA_CHANNEL_4,
                                         HardwareLayout::Interrupt { DMA2_Stream2_IRQn, 14 } }
    },
    usartLogger { usart1, 115200 },

    // Test pin
    testPin { portC, GPIO_PIN_7, GPIO_MODE_OUTPUT_PP }
{
    // External oscillators use system pins
    sysClock.setHSE(&portH, GPIO_PIN_0 | GPIO_PIN_1);
    sysClock.setLSE(&portC, GPIO_PIN_14 | GPIO_PIN_15);
}


void Hardware::abort ()
{
    ledRed.turnOn();
    while(1)
    {
        __NOP();
    }
}


void Hardware::initClock (uint32_t frequency)
{
    clockParameters.searchBestParameters(frequency);
    sysClock.setSysClockSource(RCC_SYSCLKSOURCE_PLLCLK);
    sysClock.getOscParameters().PLL.PLLState = RCC_PLL_ON;
    sysClock.getOscParameters().PLL.PLLSource = RCC_PLLSOURCE_HSE;
    sysClock.getOscParameters().PLL.PLLM = clockParameters.M;
    sysClock.getOscParameters().PLL.PLLN = clockParameters.N;
    sysClock.getOscParameters().PLL.PLLP = clockParameters.P;
    sysClock.getOscParameters().PLL.PLLQ = clockParameters.Q;
    sysClock.setAHB(RCC_SYSCLK_DIV1, clockParameters.APB1, clockParameters.APB2);
    sysClock.setLatency(FLASH_LATENCY_7);
    sysClock.setRTC();
    sysClock.setI2S(192, 2);
    sysClock.start();
}


bool Hardware::start()
{
    // test pin and MCO
    testPin.start();
    testPin.setHigh();
    mco.start();

    // LEDs
    ledGreen.start();
    ledGreen.turnOff();
    ledBlue.start();
    ledBlue.turnOff();
    ledRed.start();
    ledRed.turnOn();

    // Logger
    usartLogger.initInstance();
    USART_DEBUG("--------------------------------------------------------" << UsartLogger::ENDL);
    USART_DEBUG("MCU frequency: " << SystemClock::getInstance()->getMcuFreq() << UsartLogger::ENDL);
    clockParameters.print();

    // For RTC, it is necessary to reset the state since it will not be
    // automatically reset after MCU programming.
    rtc.stop();
    do
    {
        Rtc::Start::Status status = rtc.start(8 * 2047 + 7, RTC_WAKEUPCLOCK_RTCCLK_DIV2);
        USART_DEBUG("RTC status: " << Rtc::Start::asString(status) << " (" << rtc.getHalStatus() << ")" << UsartLogger::ENDL);
    }
    while (rtc.getHalStatus() != HAL_OK);

    // SPI
    DeviceStart::Status devStatus = spi.start(SPI_DIRECTION_1LINE, SPI_BAUDRATEPRESCALER_256, SPI_DATASIZE_8BIT, SPI_PHASE_2EDGE);
    USART_DEBUG("SPI" << spi.getId() << " status: " << DeviceStart::asString(devStatus) << " (" << spi.getHalStatus() << ")" << UsartLogger::ENDL);
    if (devStatus != DeviceStart::Status::OK)
    {
        return false;
    }

    devStatus = lcd.start(2);
    USART_DEBUG("LCD status: " << DeviceStart::asString(devStatus) << " (" << spi.getHalStatus() << ")" << UsartLogger::ENDL);

    // SSD
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
    ssd.start();

    // ADC/DAC
    devStatus = adcTemperature.start();
    USART_DEBUG("ADC" << adcTemperature.getId() << " status: " << DeviceStart::asString(devStatus) << " (" << adcTemperature.getHalStatus() << ")" << UsartLogger::ENDL);
    if (devStatus != DeviceStart::Status::OK)
    {
        return false;
    }
    adcTemperature.setVRef(3.25);

    devStatus = dac.start();
    USART_DEBUG("DAC" << dac.getId() << " status: " << DeviceStart::asString(devStatus) << " (" << dac.getHalStatus() << ")" << UsartLogger::ENDL);
    if (devStatus != DeviceStart::Status::OK)
    {
        return false;
    }

    // start timers
    uint32_t timerPrescaler = SystemCoreClock/10000 - 1;
    devStatus = heartbeatTimer.start(TIM_COUNTERMODE_UP, timerPrescaler, 5000 - 1);
    USART_DEBUG("TIM" << heartbeatTimer.getId() << " status: " << DeviceStart::asString(devStatus) << " (" << heartbeatTimer.getHalStatus() << ")" << UsartLogger::ENDL);
    if (devStatus != DeviceStart::Status::OK)
    {
        return false;
    }

    // SD card
    pinSdDetect.start();
    if (sdCard.isCardInserted() && !initSdCard())
    {
        return false;
    }

    stopButton.start();

    testPin.setLow();
    ledRed.turnOff();
    USART_DEBUG("--------------------------------------------------------" << UsartLogger::ENDL);
    return true;
}


void Hardware::stop ()
{
    // Stop all devices
    stopButton.stop();
    sdCard.stop();
    pinSdPower.setHigh();
    pinSdPower.stop();
    pinSdDetect.stop();
    heartbeatTimer.stopCounter();
    dac.stop();
    adcTemperature.stop();
    ssd.stop();
    lcd.stop();
    spi.stop();
    rtc.stop();
    ledBlue.stop();
    ledRed.stop();
    ledGreen.stop();
    mco.stop();
    testPin.stop();

    // Log resource occupations after all devices (except USART1 for logging, HSE, LSE) are stopped.
    // Desired: one at portB and DMA2 (USART1), one for portC (LSE), one for portH (HSE)
    printResourceOccupation();
    usartLogger.clearInstance();

    sysClock.stop();
}


bool Hardware::initSdCard ()
{
    pinSdPower.start();
    pinSdPower.setLow();
    HAL_Delay(250);
    DeviceStart::Status devStatus = sdCard.start();
    USART_DEBUG("SD card status: " << DeviceStart::asString(devStatus) << " (" << getSdio().getHalStatus() << ")" << UsartLogger::ENDL);
    if (devStatus == DeviceStart::OK)
    {
        getSdio().printInfo();
        devStatus = sdCard.mountFatFs();
        USART_DEBUG("FAT FS card status: " << DeviceStart::asString(devStatus) << " (" << getSdio().getHalStatus() << ")" << UsartLogger::ENDL);
        if (devStatus == DeviceStart::OK)
        {
            sdCard.listFiles();
            return true;
        }
    }
    return false;
}


void Hardware::printResourceOccupation ()
{
    USART_DEBUG("Resource occupations: " << UsartLogger::ENDL
                << UsartLogger::TAB << "portA=" << portA.getObjectsCount() << UsartLogger::ENDL
                << UsartLogger::TAB << "portB=" << portB.getObjectsCount() << UsartLogger::ENDL
                << UsartLogger::TAB << "portC=" << portC.getObjectsCount() << UsartLogger::ENDL
                << UsartLogger::TAB << "portD=" << portD.getObjectsCount() << UsartLogger::ENDL
                << UsartLogger::TAB << "portH=" << portH.getObjectsCount() << UsartLogger::ENDL
                << UsartLogger::TAB << "dma1=" << dma1.getObjectsCount() << UsartLogger::ENDL
                << UsartLogger::TAB << "dma2=" << dma2.getObjectsCount() << UsartLogger::ENDL);
}


/************************************************************************
 * External interrupts
 ************************************************************************/
extern MyApplication * appPtr;

extern "C"
{
    // Errors
    void HardFault_Handler (void)
    {
        appPtr->abort();
    }

    void MemManage_Handler (void)
    {
        appPtr->abort();
    }

    void BusFault_Handler (void)
    {
        appPtr->abort();
    }

    void UsageFault_Handler (void)
    {
        appPtr->abort();
    }

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
            appPtr->scheduleEvent(MyApplication::EventType::SECOND_INTERRUPT);
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

    void DMA1_Stream6_IRQHandler (void)
    {
        appPtr->getEsp().processDmaTxInterrupt();
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
        appPtr->getSdio().processDmaRxInterrupt();
    }

    void DMA2_Stream6_IRQHandler (void)
    {
        appPtr->getSdio().processDmaTxInterrupt();
    }

    void SDIO_IRQHandler (void)
    {
        appPtr->getSdio().processSdIOInterrupt();
    }

    // I2S
    void DMA1_Stream4_IRQHandler (void)
    {
        appPtr->getI2S().processDmaTxInterrupt();
    }

    void HAL_I2S_TxCpltCallback (I2S_HandleTypeDef * /*channel*/)
    {
        appPtr->getI2S().processCallback(SharedDevice::State::TX_CMPL);
    }

    // ADC
    void DMA2_Stream0_IRQHandler()
    {
        appPtr->getAdcTemp().processDmaRxInterrupt();
    }

    void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef * /*channel*/)
    {
        if (appPtr->getAdcTemp().processConvCpltCallback())
        {
            appPtr->scheduleEvent(MyApplication::EventType::ADCTEMP_READY);
        }
    }

    // Timers
    void TIM3_IRQHandler ()
    {
        appPtr->getHeartbeatTimer().processInterrupt();
        appPtr->scheduleEvent(MyApplication::EventType::HEARTBEAT_INTERRUPT);
    }

}

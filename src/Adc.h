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

#ifndef STM32ASYNC_ADC_H_
#define STM32ASYNC_ADC_H_

#include "HardwareLayout/Adc.h"

#ifdef HAL_ADC_MODULE_ENABLED

#include "IODevice.h"
#include "SharedDevice.h"

namespace Stm32async
{

/**
 * @brief Base class that implements analog-to-digit converter
 */
class BaseAdc : public IODevice<HardwareLayout::Adc, ADC_HandleTypeDef, 1>
{
public:

    static constexpr uint32_t INVALID_VALUE = __UINT32_MAX__;
    static constexpr uint32_t TIMEOUT = 10000;

    /**
     * @brief Default constructor.
     */
    BaseAdc (const HardwareLayout::Adc & _device, uint32_t _channel, uint32_t _samplingTime);

    /**
     * @brief Start this ADC device.
     */
    DeviceStart::Status start ();

    /**
     * @brief Stop this ADC device.
     */
    void stop ();

    /**
     * @brief Read analog value in a blocking mode.
     */
    uint32_t readBlocking ();

    /**
     * @brief Read analog value in a blocking mode (in Volts).
     */
    inline float readBlockingV ()
    {
        return (vRef * (float)readBlocking())/4095.0;
    }

    /**
     * @brief Read analog value in a blocking mode (in Volts).
     */
    inline int readBlockingMV ()
    {
        return (int)(readBlockingV() * 1000);
    }

    /**
     * @brief Set the reference voltage if reading in volts will be performed.
     */
    inline void setVRef (float ref)
    {
        vRef = ref;
    }

    /**
     * @brief Procedure returns the reference voltage.
     */
    inline float getVRef () const
    {
        return vRef;
    }

    /**
     * @brief Procedure returns the parameters of ADC channel.
     */
    inline ADC_ChannelConfTypeDef & getAdcChannel ()
    {
        return adcChannel;
    }

protected:

    ADC_ChannelConfTypeDef adcChannel;
    float vRef;
};


/**
 * @brief Class that implements analog-to-digit converter
 */
class AsyncAdc : public BaseAdc, public SharedDevice
{
public:

    static constexpr size_t ADC_BUFFER_LENGTH = 1000;

    AsyncAdc (const HardwareLayout::Adc & _device, uint32_t _channel, uint32_t _samplingTime);

    DeviceStart::Status start ();

    void stop ();

    HAL_StatusTypeDef read ();

    bool processConvCpltCallback ();

    uint32_t getMedian ();

    /**
     * @brief Read analog value in a blocking mode (in Volts).
     */
    inline float getMedianV ()
    {
        return (vRef * (float)getMedian())/4095.0;
    }

    /**
     * @brief Read analog value in a blocking mode (in Volts).
     */
    inline int getMedianMV ()
    {
        return (int)(getMedianV() * 1000);
    }

    inline void processInterrupt ()
    {
        HAL_ADC_IRQHandler(&parameters);
    }

private:

    volatile size_t nrReadings;
    std::array<uint32_t, ADC_BUFFER_LENGTH> adcBuffer;
};


} // end namespace
#endif
#endif

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

#ifndef STM32ASYNC_TIMER_H_
#define STM32ASYNC_TIMER_H_

#include "HardwareLayout/Timer.h"

#ifdef HAL_TIM_MODULE_ENABLED

#include "IODevice.h"

namespace Stm32async
{

/**
 * @brief Class that implements basic timer interface.
 */
class BaseTimer
{
public:

    /**
     * @brief Default constructor.
     */
    BaseTimer (const HardwareLayout::Timer & _device);

    DeviceStart::Status startCounter (uint32_t counterMode, uint32_t prescaler, uint32_t period,
                                      uint32_t clockDivision = TIM_CLOCKDIVISION_DIV1);

    void stopCounter ();

    /**
     * @brief Returns device ID.
     */
    inline size_t getId () const
    {
        return device.getId();
    }

    inline TIM_HandleTypeDef & getParameters ()
    {
        return parameters;
    }

    /**
     * @brief Getter for HAL status of the last operation
     */
    inline HAL_StatusTypeDef getHalStatus () const
    {
        return halStatus;
    }

    inline uint32_t getValue () const
    {
        return __HAL_TIM_GET_COUNTER(&parameters);
    }

    inline void reset ()
    {
        __HAL_TIM_SET_COUNTER(&parameters, 0);
    }

protected:

    const HardwareLayout::Timer & device;
    TIM_HandleTypeDef parameters;
    HAL_StatusTypeDef halStatus;
};

/**
 * @brief Class that implements timer interface.
 */
class InterruptTimer : public BaseTimer
{
public:

    /**
     * @brief Default constructor.
     */
    InterruptTimer (const HardwareLayout::Timer & _device);

    DeviceStart::Status start (uint32_t counterMode, uint32_t prescaler, uint32_t period,
                               uint32_t clockDivision = TIM_CLOCKDIVISION_DIV1,
                               uint32_t repetitionCounter = 1);

    void stop ();

    inline void setPrescaler (uint32_t prescaler)
    {
        parameters.Init.Prescaler = prescaler;
        __HAL_TIM_SET_PRESCALER(&parameters, prescaler);
    }

    inline void setPeriod (uint32_t period)
    {
        parameters.Init.Period = period;
        __HAL_TIM_SET_AUTORELOAD(&parameters, period);
    }

    inline void processInterrupt ()
    {
        HAL_TIM_IRQHandler(&parameters);
    }
};

} // end namespace
#endif
#endif

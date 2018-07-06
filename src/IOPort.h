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

#include "HardwareLayout.h"

#ifndef STM32ASYNC_IOPORT_H_
#define STM32ASYNC_IOPORT_H_

namespace Stm32async
{

/**
 * @brief Base IO port class.
 */
class IOPort
{
public:

    /**
     * @brief Default constructor.
     */
    IOPort (const HardwareLayout::Port & _port, uint32_t pins = GPIO_PIN_All, uint32_t mode = GPIO_MODE_INPUT,
            uint32_t pull = GPIO_NOPULL, uint32_t speed = GPIO_SPEED_FREQ_LOW, bool callStart = false);

    void start ();
    void stop ();

    inline GPIO_InitTypeDef & getParameters ()
    {
        return parameters;
    }

    /**
     * @brief  Set the selected data port bit.
     *
     * @note   This function uses GPIOx_BSRR and GPIOx_BRR registers to allow atomic read/modify
     *         accesses. In this way, there is no risk of an IRQ occurring between
     *         the read and the modify access.
     */
    inline void setHigh ()
    {
        HAL_GPIO_WritePin(port.getInstance(), parameters.Pin, GPIO_PIN_SET);
    }

    /**
     * @brief  Clear the selected data port bit.
     *
     * @note   This function uses GPIOx_BSRR and GPIOx_BRR registers to allow atomic read/modify
     *         accesses. In this way, there is no risk of an IRQ occurring between
     *         the read and the modify access.
     */
    inline void setLow ()
    {
        HAL_GPIO_WritePin(port.getInstance(), parameters.Pin, GPIO_PIN_RESET);
    }

    /**
     * @brief Toggle the specified GPIO pin.
     */
    inline void toggle ()
    {
        HAL_GPIO_TogglePin(port.getInstance(), parameters.Pin);
    }

    /**
     * @brief Write the given integer into the GPIO port output data register.
     */
    inline void putInt (uint32_t val)
    {
        port.getInstance()->ODR = val;
    }

    /**
     * @brief Returns the value from GPIO port input data register.
     */
    inline uint32_t getInt () const
    {
        return port.getInstance()->IDR;
    }

protected:

    /**
     * @brief Link to the port registers.
     */
    const HardwareLayout::Port & port;

    /**
     * @brief Current port parameters.
     */
    GPIO_InitTypeDef parameters;
};

} // end namespace
#endif

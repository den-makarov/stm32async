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

#ifndef STM32ASYNC_IOPORT_H_
#define STM32ASYNC_IOPORT_H_

#include "Stm32async.h"

namespace Stm32async
{

/**
 * @brief Base IO port class.
 *
 * This class provides access to the base functionality of the GPIO port.
 * Usage example:
 *
 *     IOPort ledBlue { portC, GPIO_PIN_2, GPIO_MODE_OUTPUT_PP };
 *     ledBlue.start();
 *     while (true)
 *     {
 *         ledBlue.toggle();;
 *     }
 *     ledBlue.stop();
 */
class IOPort
{
public:

    /**
     * @brief Standard initialization constructor.
     *
     * @param _port the port this device mapped to.
     * @param _pins affected pin or pins.
     * @param _mode GPIO configuration mode.
     * @param _pull GPIO Pull-Up or Pull-Down Activation.
     * @param _speed GPIO Output Maximum frequency.
     */
    IOPort (const HardwareLayout::Port & _port, uint32_t _pins = GPIO_PIN_All, uint32_t _mode = GPIO_MODE_INPUT,
            uint32_t _pull = GPIO_NOPULL, uint32_t _speed = GPIO_SPEED_FREQ_LOW);

    /**
     * @brief This method activates the port: it starts the clock and calls HAL_GPIO_Init
     */
    void start ();

    /**
     * @brief This method de-activates the port: it stops the clock and calls HAL_GPIO_DeInit
     */
    void stop ();

    /**
     * @brief Getter for the port parameters
     */
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
    inline void setHigh (uint32_t pins = GPIO_PIN_All)
    {
        pins &= parameters.Pin;
        HAL_GPIO_WritePin(port.getInstance(), pins, GPIO_PIN_SET);
    }

    /**
     * @brief  Clear the selected data port bit.
     *
     * @note   This function uses GPIOx_BSRR and GPIOx_BRR registers to allow atomic read/modify
     *         accesses. In this way, there is no risk of an IRQ occurring between
     *         the read and the modify access.
     */
    inline void setLow (uint32_t pins = GPIO_PIN_All)
    {
        pins &= parameters.Pin;
        HAL_GPIO_WritePin(port.getInstance(), pins, GPIO_PIN_RESET);
    }

    /**
     * @brief Toggle the specified GPIO pin.
     */
    inline void toggle (uint32_t pins = GPIO_PIN_All)
    {
        pins &= parameters.Pin;
        HAL_GPIO_TogglePin(port.getInstance(), pins);
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

    /**
     * @brief Return the current value of the pin.
     */
    inline bool getBit () const
    {
        return (bool)HAL_GPIO_ReadPin(port.getInstance(), parameters.Pin);
    }

    /**
     * @brief Set or clear the selected data port bit.
     *
     * @note   This function uses GPIOx_BSRR and GPIOx_BRR registers to allow atomic read/modify
     *         accesses. In this way, there is no risk of an IRQ occurring between
     *         the read and the modify access.
     */
    inline void putBit (bool value)
    {
        HAL_GPIO_WritePin(port.getInstance(), parameters.Pin, (GPIO_PinState)value);
    }

protected:

    /**
     * @brief Link to the port configuration.
     */
    const HardwareLayout::Port & port;

    /**
     * @brief Current port parameters.
     */
    GPIO_InitTypeDef parameters;
};

} // end namespace
#endif

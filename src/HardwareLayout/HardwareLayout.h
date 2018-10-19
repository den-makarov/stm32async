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

#ifndef HARDWARE_LAYOUT_H_
#define HARDWARE_LAYOUT_H_

#include "Platforms.h"
#include <utility>

namespace Stm32async
{

/**
 * @brief Namespace containing classes that describe hardware configuration.
 *
 * These classes are used in order to generalize platform- and project-dependent
 * hardware configuration. In an applications, these classes are initialized at
 * application start, will hold the static hardware configuration and will be
 * used as input parameters for IODevices that implement high-level communication
 * logic.
 *
 * Since these configuration classes do not have any logic and are used for
 * configuration purpose only, all class members are usually declared as "public"
 * and can be changed directly from the user-space code.
 */
namespace HardwareLayout
{

/**
 * @brief Helper define that allows us to declare a non-static "instance" attribute within a device
 */
#define DECLARE_INSTANCE(name) \
    private: \
        name * instance; \
    public: \
        name * getInstance () const { return instance; }

/**
 * @brief Helper class used to configure interrupt parameters.
 */
class Interrupt final
{
public:

    /**
     * @brief Standard initialization constructor.
     *
     * @param _irqn External interrupt number. This parameter is an enumerator of IRQn_Type enumeration.
     * @param _prio The preemption priority for the IRQn channel. This parameter can be a value between 0 and 15.
     * @param _subPrio The sub-priority level for the IRQ channel. This parameter can be a value between 0 and 15.
     */
    Interrupt (IRQn_Type _irqn, uint32_t _prio, uint32_t _subPrio = 0) :
        irqn { _irqn },
        prio { _prio },
        subPrio { _subPrio }
    {
        // empty
    }

    /**
     * @brief Helper method that can be used in order to enable this interrupt.
     */
    inline void enable () const
    {
        HAL_NVIC_SetPriority(irqn, prio, subPrio);
        HAL_NVIC_EnableIRQ(irqn);
    }

    /**
     * @brief Helper method that can be used in order to disable this interrupt.
     */
    inline void disable () const
    {
        HAL_NVIC_DisableIRQ(irqn);
    }

    /**
     * @brief External interrupt number.
     *
     * This parameter is an enumerator of IRQn_Type enumeration.
     */
    IRQn_Type irqn;

    /**
     * @brief The preemption priority for the IRQn channel.
     *
     * This parameter can be a value between 0 and 15. A lower priority
     * value indicates a higher priority.
     */
    uint32_t prio;

    /**
     * @brief The sub-priority level for the IRQ channel.
     *
     * This parameter can be a value between 0 and 15. A lower priority
     * value indicates a higher priority.
     */
    uint32_t subPrio;
};

/**
 * @brief Class provides an interface for any HAL device that have a clock.
 */
class HalDevice
{
public:

    /**
     * @brief Standard initialization constructor.
     *
     * @param _id a numerical device ID used for logging purpose.
     */
    HalDevice (size_t _id) : id { _id }
    {
        // empty
    }

    /**
     * @brief Standard destructor.
     */
    virtual ~HalDevice () = default;

    /**
     * @brief Returns device ID.
     */
    inline size_t getId () const
    {
        return id;
    }

    /**
     * @brief Method that enables device-specific clock.
     *
     * This method shall be implemented in a derived class. For example, a device that implements
     * USART shall have the following implementation of this method:
     *
     *     virtual void enableClock () const
     *     {
     *        __HAL_RCC_USART1_CLK_ENABLE();
     *     }
     */
    virtual void enableClock () const =0;

    /**
     * @brief Method that disables device-specific clock.
     *
     * This method shall be implemented in a derived class. For example, a device that implements
     * USART shall have the following implementation of this method:
     *
     *     virtual void disableClock () const
     *     {
     *        __HAL_RCC_USART1_CLK_DISABLE();
     *     }
     */
    virtual void disableClock () const =0;

protected:

    /**
     * @brief Numerical device ID used for logging purpose.
     */
    size_t id;
};

/**
 * @brief Class provides an interface for any HAL device that have a shared clock
 *        (i.e the clock is used by more than one device)
 *
 * Since some clocks are shared between different devices (like DMAN, ports, AFIO)
 * we shall maintain the clock sharing in order to prevent disable of the clock
 * if any device that uses this clock remains active.
 *
 * Such a device that uses a shared clock shall be derived from this class. It also
 * shall be defined globally in the user-space code and shall be passed to any other
 * device that uses this clock.
 */
class HalSharedDevice : public HalDevice
{
public:

    /**
     * @brief Standard initialization constructor.
     *
     * @param _id a numerical device ID used for logging purpose.
     */
    HalSharedDevice (size_t _id) :
        HalDevice { _id },
        objectsCount { 0 }
    {
        // empty
    }

    /**
     * @brief Method that checks whether the device still be used by any other device.
     */
    inline bool isUsed () const
    {
        return objectsCount > 0;
    }

    /**
     * @brief Method that returns the number of devices that still use this device.
     */
    inline size_t getObjectsCount () const
    {
        return objectsCount;
    }

    /**
     * @brief Default implementation of clock activation method.
     */
    virtual void enableClock () const
    {
        onClockEnable();
        ++objectsCount;
    }

    /**
     * @brief Default implementation of clock de-activation method.
     */
    virtual void disableClock () const
    {
        if (!isUsed())
        {
            return;
        }
        else
        {
            --objectsCount;
            if (!isUsed())
            {
                onClockDisable();
            }
        }
    }

protected:

    /**
     * @brief Counter: how many clients currently use this instance with common clocking domain
     */
    mutable size_t objectsCount;

    /**
     * @brief An abstract callback for the clock activation event.
     *
     * This callback shall be implemented in a derived class. For example, an instance of
     * an IO port can implement this callback as follows:
     *
     *     virtual void onClockEnable () const
     *     {
     *         __HAL_RCC_GPIOC_CLK_ENABLE();
     *     }
     */
    virtual void onClockEnable () const =0;

    /**
     * @brief An abstract callback for the clock de-activation event.
     *
     * This callback shall be implemented in a derived class. For example, an instance of
     * an IO port can implement this callback as follows:
     *
     *     virtual void onClockEnable () const
     *     {
     *         __HAL_RCC_GPIOC_CLK_DISABLE();
     *     }
     */
    virtual void onClockDisable () const =0;
};

/**
 * @brief Helper class that represents a shared DMA device.
 *
 * This class represents DMA instance as a DMA_TypeDef structure. It is a base
 * class for DMA1/DMA2 wrapper classes that provide access to the clock
 * enable/disable macros for both DMA modules.
 */
class Dma : public HalSharedDevice
{
    DECLARE_INSTANCE(DMA_TypeDef)

public:

    /**
     * @brief Standard initialization constructor.
     *
     * @param _id a numerical device ID used for logging purpose.
     * @param _dma pointer to the HAL DMA definition structure
     */
    Dma (size_t _id, DMA_TypeDef * _dma) :
        HalSharedDevice { _id },
        instance { _dma }
    {
        // empty
    }
};

/**
 * @brief Configuration class that collects parameters of DMA stream.
 *
 * This class is aimed to establish a link between DMA module, DMA stream,
 * and DMA channel.
 */
class DmaStream final
{
public:

    /**
     * @brief Specifies the DMA device used for this stream
     */
    const Dma * dma;

    /**
     * @brief Specifies the stream
     */
    DMA_Stream_Struct * stream;

    /**
     * @brief Specifies the channel used for the specified stream
     */
    uint32_t channel;

    /**
     * @brief Interrupt vector definition for DMA channel
     */
    Interrupt dmaIrq;

    /**
     * @brief Standard initialization constructor.
     *
     * @param _dma a pointer to a DMA module instance.
     * @param _stream a pointer to the platform-dependent HAL DMA stream definition structure.
     * @param _channel numerical channel number within the considered DMA stream.
     * @param _dmaIrq an interrupt request on DMA transfer complete event.
     */
    DmaStream (const Dma * _dma, DMA_Stream_Struct * _stream, uint32_t _channel, Interrupt && _dmaIrq) :
        dma { _dma },
        stream { _stream },
        channel { _channel },
        dmaIrq { std::move(_dmaIrq) }
    {
        // empty
    }
};

/**
 * @brief Parameters of a port represented by HAL GPIO_TypeDef.
 */
class Port : public HalSharedDevice
{
    DECLARE_INSTANCE(GPIO_TypeDef)

public:

    /**
     * @brief Standard initialization constructor.
     *
     * @param _id a numerical device ID used for logging purpose.
     * @param _instance pointer to the HAL GPIO definition structure
     */
    Port (size_t _id, GPIO_TypeDef * _instance) :
        HalSharedDevice { _id },
        instance { _instance }
    {
        // empty
    }
};

/**
 * @brief Alternate function input/output module.
 *
 * Helper class used to control counting peripheral modules that need remapping pins.
 *
 * Note: this class is only used for STM32F1 series. The F4 MCUs do not have AFIO module,
 * the GPIO parameter "Alternate" is used instead.
 */
class Afio : public HalSharedDevice
{
    DECLARE_INSTANCE(AFIO_TypeDef)

public:

    /**
     * @brief Standard initialization constructor.
     *
     * @param _id a numerical device ID used for logging purpose.
     * @param _instance pointer to the HAL AFIO definition structure
     */
    Afio (size_t _id, AFIO_TypeDef * _instance) :
        HalSharedDevice { _id },
        instance { _instance }
    {
        // empty
    }
};


class HalAfioDevice : public HalDevice
{
public:

    /**
     * @brief Standard initialization constructor.
     *
     * @param _id a numerical device ID used for logging purpose.
     * @param _remapped a boolean flag indicating that IO port used
     *         by this device shall be re-mapped to alternative pins.
     * @param _afio pointer to the AFIO module if it necessary for remapping.
     */
    HalAfioDevice (size_t _id, bool _remapped, Afio * _afio) :
        HalDevice { _id },
        afio { _afio },
        remapped { _remapped }
    {
        // empty
    }

    /**
     * @brief Method that performs the pin re-mapping for this device, if necessary.
     *
     * If the implementing device can use alternative pins, this method shall be
     * implemented in a derived class. The implementation of this method may depend
     * on the STM32 MCU family.
     */
    virtual void remapPins (GPIO_InitTypeDef &) const
    {
        // default implementation is empty
    }

    /**
     * @brief Method that performs the pin un-mapping for this device, if necessary.
     *
     * If the implementing device uses alternative pins, and these pins shall be set
     * to the default (un-remapped) state, this method shall be  implemented in a derived
     * class. The implementation of this method may depend on the STM32 MCU family.
     */
    virtual void unremapPins (GPIO_InitTypeDef &) const
    {
        // default implementation is empty
    }

    /**
     * @brief Getter for the optional AFIO module.
     */
    inline const Afio * getAfio () const
    {
        return afio;
    }

protected:

    /**
     * @brief AFIO module. Set to NULL in case it's not required (for example, on STM32F4 MCU)
     */
    Afio * afio;

    /**
     * @brief Peripheral to be re-mapped to the selected pins.
     */
    bool remapped;
};

/**
 * @brief Helper class that defines a set of pins of the same port.
 */
class Pins
{
public:
    /**
     * @brief Related port
     */
    Port & port;

    /**
     * @brief Pin indices
     */
    uint32_t pins;

    /**
     * @brief Standard initialization constructor.
     *
     * @param _port a link to the GPIO port.
     * @param _pins pin indices
     */
    Pins (Port & _port, uint32_t _pins) :
        port { _port },
        pins { _pins }
    {
        // empty
    }
};

} // end of namespace HardwareLayout
} // end of namespace Stm32async

#endif

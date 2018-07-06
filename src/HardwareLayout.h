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

#include "Stm32async.h"

#include <utility>

/**
 * @brief Namespace containing classes that describe hardware configuration.
 */
namespace HardwareLayout
{

/**
 * @brief Helper class used to configure interrupt parameters.
 */
class Interrupt
{
public:

    /**
     * @brief Standard initialization constructor.
     *
     * @param External interrupt number. This parameter is an enumerator of IRQn_Type enumeration.
     * @param The preemption priority for the IRQn channel. This parameter can be a value between 0 and 15.
     * @param The subpriority level for the IRQ channel. This parameter can be a value between 0 and 15.
     */
    Interrupt (IRQn_Type _irqn, uint32_t _prio, uint32_t _subPrio) :
        irqn { _irqn },
        prio { _prio },
        subPrio { _subPrio }
    {
        // empty
    }

    /**
     * @brief Move constructor.
     *
     * This constructor can be used in order to initialize some device that needs an
     * interrupt using anonymous class:
     *
     *    device { HardwareLayout::Interrupt { SysTick_IRQn, 0, 0 } }
     *
     * @param External interrupt number. This parameter is an enumerator of IRQn_Type enumeration.
     * @param The preemption priority for the IRQn channel. This parameter can be a value between 0 and 15.
     * @param The subpriority level for the IRQ channel. This parameter can be a value between 0 and 15.
     */
    Interrupt (Interrupt && irq) :
        irqn { irq.irqn },
        prio { irq.prio },
        subPrio { irq.subPrio }
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
     * @brief External interrupt number. This parameter is an enumerator of IRQn_Type enumeration.
     */
    IRQn_Type irqn;

    /**
     * @brief The preemption priority for the IRQn channel. This parameter can be a value between 0 and 15.
     *
     * A lower priority value indicates a higher priority.
     */
    uint32_t prio;

    /**
     * @brief The subpriority level for the IRQ channel. This parameter can be a value between 0 and 15.
     *
     * A lower priority value indicates a higher priority.
     */
    uint32_t subPrio;
};

/**
 * @brief Class provides an interface for any HAL device that have a clock.
 */
class HalDevice
{
public:

    HalDevice (size_t _id, bool _remapped = false) :
        id { _id },
        remapped { _remapped }
    {
        // empty
    }

    virtual ~HalDevice ()
    {
        // empty
    }

    virtual void enableClock () const =0;
    virtual void disableClock () const =0;
    virtual void remapPins (GPIO_InitTypeDef &) const
    {
        // default implementation is empty
    }

protected:

    size_t id;

    /**
     * @brief Peripheral to be re-mapped to the selected pins
     */
    bool remapped;
};

/**
 * @brief Class provides an interface for any HAL device that have a shared clock
 *        (i.e the clock is used by more than one device)
 */
class HalSharedDevice : public HalDevice
{
public:

    HalSharedDevice (size_t _id) :
        HalDevice { _id },
        objectsCount { 0 }
    {
        // empty
    }

    inline bool isUsed () const
    {
        return objectsCount > 0;
    }

    inline size_t getObjectsCount () const
    {
        return objectsCount;
    }

    virtual void enableClock () const
    {
        onClockEnable();
        ++objectsCount;
    }

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

    virtual void onClockEnable () const =0;
    virtual void onClockDisable () const =0;
};

/**
 * @brief Helper class used to configure DMA device.
 */
class Dma : public HalSharedDevice
{
    DECLARE_INSTANCE(DMA_TypeDef)

public:

    Dma (size_t _id, DMA_TypeDef * dma) :
        HalSharedDevice { _id },
        instance { dma }
    {
        // empty
    }
};

/**
 * @brief Helper class used to configure DMA stream.
 */
class DmaStream
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

    DmaStream (const Dma * _dma, DMA_Stream_Struct * _stream, uint32_t _channel) :
        dma { _dma },
        stream { _stream },
        channel { _channel }
    {
        // empty
    }

    DmaStream (DmaStream && dmaStream) :
        dma { dmaStream.dma },
        stream { dmaStream.stream },
        channel { dmaStream.channel }
    {
        // empty
    }
};

/**
 * @brief Parameters of a port.
 */
class Port : public HalSharedDevice
{
    DECLARE_INSTANCE(GPIO_TypeDef)

public:

    explicit Port (size_t _id, GPIO_TypeDef * _instance) :
        HalSharedDevice { _id },
        instance { _instance }
    {
        // empty
    }
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

    explicit Pins (Port & _port, uint32_t _pins) :
        port { _port },
        pins { _pins }
    {
        // empty
    }
};

/**
 * @brief Parameters of USART device.
 */
class Usart : public HalDevice
{
    DECLARE_INSTANCE(USART_TypeDef)

public:

    /**
     * @brief TX pin
     */
    Pins txPin;

    /**
     * @brief RX pin
     */
    Pins rxPin;

    /**
     * @brief Interrupt Number Definition
     */
    Interrupt txRxIrq;

    /**
     * @brief TX DMA channel
     */
    DmaStream txDma;

    /**
     * @brief Interrupt Number Definition for TX DMA channel
     */
    Interrupt txDmaIrq;

    /**
     * @brief RX DMA channel
     */
    DmaStream rxDma;

    /**
     * @brief Interrupt Number Definition for RX DMA channel
     */
    Interrupt rxDmaIrq;

    explicit Usart (size_t _id,  USART_TypeDef *_instance, Port & _txPort, uint32_t _txPin, Port & _rxPort,
                    uint32_t _rxPin, bool _remapped,
                    Interrupt && _txRxIrq,
                    DmaStream && _txDma, Interrupt && _txDmaIrq,
                    DmaStream && _rxDma, Interrupt && _rxDmaIrq) :
        HalDevice { _id, _remapped },
        instance { _instance },
        txPin { _txPort, _txPin },
        rxPin { _rxPort, _rxPin },
        txRxIrq { std::move(_txRxIrq) },
        txDma { std::move(_txDma) },
        txDmaIrq { std::move(_txDmaIrq) },
        rxDma { std::move(_rxDma) },
        rxDmaIrq { std::move(_rxDmaIrq) }
    {
        // empty
    }

    void enableIrq () const
    {
        txRxIrq.enable();
        txDmaIrq.enable();
        rxDmaIrq.enable();
    }

    void disableIrq () const
    {
        txRxIrq.disable();
        txDmaIrq.disable();
        rxDmaIrq.disable();
    }
};


} // end namespace
#endif

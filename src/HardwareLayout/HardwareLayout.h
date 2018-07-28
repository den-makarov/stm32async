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
class Interrupt
{
public:

    /**
     * @brief Standard initialization constructor.
     *
     * @param _irqn External interrupt number. This parameter is an enumerator of IRQn_Type enumeration.
     * @param _prio The preemption priority for the IRQn channel. This parameter can be a value between 0 and 15.
     * @param _subPrio The sub-priority level for the IRQ channel. This parameter can be a value between 0 and 15.
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
     * @param _irq source object to be moved into this object.
     */
    Interrupt (Interrupt && _irq) :
        irqn { _irq.irqn },
        prio { _irq.prio },
        subPrio { _irq.subPrio }
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
     * @param _remapped a boolean flag indicating that IO port used
     *         by this device shall be re-mapped to alternative pins.
     */
    HalDevice (size_t _id, bool _remapped = false) :
        id { _id },
        remapped { _remapped }
    {
        // empty
    }

    /**
     * @brief Standard destructor.
     */
    virtual ~HalDevice ()
    {
        // empty
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

protected:

    /**
     * @brief Numerical device ID used for logging purpose.
     */
    size_t id;

    /**
     * @brief Peripheral to be re-mapped to the selected pins.
     */
    bool remapped;
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

    /**
     * @brief Move constructor.
     *
     * This constructor can be used in order to initialize some device that needs a
     * DMA stream using anonymous class:
     *
     *    usart6 { portC, GPIO_PIN_6, portC, GPIO_PIN_7,
     *           HardwareLayout::Interrupt { USART6_IRQn, 1, 0 },
     *           HardwareLayout::DmaStream { &dma2, DMA2_Stream7, DMA_CHANNEL_5,
     *                                       HardwareLayout::Interrupt { DMA2_Stream7_IRQn, 2, 0 } },
     *           HardwareLayout::DmaStream { &dma2, DMA2_Stream2, DMA_CHANNEL_5,
     *                                       HardwareLayout::Interrupt { DMA2_Stream2_IRQn, 2, 0 } } }
     *
     * @param _dmaStream source object to be moved into this object.
     */
    DmaStream (DmaStream && _dmaStream) :
        dma { _dmaStream.dma },
        stream { _dmaStream.stream },
        channel { _dmaStream.channel },
        dmaIrq { std::move(_dmaStream.dmaIrq) }
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
    explicit Port (size_t _id, GPIO_TypeDef * _instance) :
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
    explicit Afio (size_t _id, AFIO_TypeDef * _instance) :
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

    /**
     * @brief Standard initialization constructor.
     *
     * @param _port a link to the GPIO port.
     * @param _pins pin indices
     */
    explicit Pins (Port & _port, uint32_t _pins) :
        port { _port },
        pins { _pins }
    {
        // empty
    }
};

/**
 * @brief Configuration of Universal Synchronous Asynchronous Receiver Transmitter (USART).
 *
 * This class holds the configuration of USART module: USART definition structure, used TX/RX pins,
 * optional link to AFIO module used to remap TX/RX pins, configuration of USART global interrupt,
 * configuration of TX/RX DMA channels with corresponding interrupts.
 *
 * Since USART is able to use alternative pins, the derived class shall implement virtual
 * remapPins() and unremapPins() methods from the base class.
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
     * @brief AFIO module. Set to NULL in case it's not required (for example, on STM32F4 MCU)
     */
    Afio * afio;

    /**
     * @brief USART global interrupt configuration
     */
    Interrupt txRxIrq;

    /**
     * @brief TX DMA channel
     */
    DmaStream txDma;

    /**
     * @brief RX DMA channel
     */
    DmaStream rxDma;

    /**
     * @brief Standard initialization constructor.
     *
     * @param _id a numerical device ID used for logging purpose.
     * @param _instance pointer to the HAL USART definition structure.
     * @param _txPort the port of transmitter line.
     * @param _txPin the pin of transmitter line.
     * @param _rxPort the port of receiver line.
     * @param _rxPin the pin of receiver line.
     * @param _remapped flag indicating whether the TX/RX pins shall be remapped.
     * @param _afio pointer to the AFIO module if it necessary for remapping.
     * @param _txRxIrq link to the USART global interrupt.
     * @param _txDma link to the transmitter DMA stream.
     * @param _rxDma to the receiver DMA stream.
     */
    explicit Usart (size_t _id,  USART_TypeDef *_instance,
                    Port & _txPort, uint32_t _txPin, Port & _rxPort, uint32_t _rxPin,
                    bool _remapped, Afio * _afio,
                    Interrupt && _txRxIrq,
                    DmaStream && _txDma, DmaStream && _rxDma) :
        HalDevice { _id, _remapped },
        instance { _instance },
        txPin { _txPort, _txPin },
        rxPin { _rxPort, _rxPin },
        afio { _afio },
        txRxIrq { std::move(_txRxIrq) },
        txDma { std::move(_txDma) },
        rxDma { std::move(_rxDma) }
    {
        // empty
    }

    /**
     * @brief Helper method used to enable all interrupts for the module
     */
    void enableIrq () const
    {
        txRxIrq.enable();
        txDma.dmaIrq.enable();
        rxDma.dmaIrq.enable();
    }

    /**
     * @brief Helper method used to disable all interrupts for the module
     */
    void disableIrq () const
    {
        txRxIrq.disable();
        txDma.dmaIrq.disable();
        rxDma.dmaIrq.disable();
    }
};

/**
 * @brief Configuration of Serial Peripheral Interface.
 */
class Spi : public HalDevice
{
    DECLARE_INSTANCE(SPI_TypeDef)

public:

    /**
     * @brief Serial clock pin: output from master
     */
    Pins sclkPin;

    /**
     * @brief Master Out Slave In (MOSI, data output from master) pin
     */
    Pins mosiPin;

    /**
     * @brief Master In Slave Out (MISO, data output from slave) pin
     */
    Pins misoPin;

    /**
     * @brief AFIO module. Set to NULL in case it's not required (for example, on STM32F4 MCU)
     */
    Afio * afio;

    /**
     * @brief USART global interrupt configuration
     */
    Interrupt txRxIrq;

    /**
     * @brief TX DMA channel
     */
    DmaStream txDma;

    /**
     * @brief RX DMA channel
     */
    DmaStream rxDma;

    /**
     * @brief Standard initialization constructor.
     *
     * @param _id a numerical device ID used for logging purpose.
     * @param _instance pointer to the HAL USART definition structure.
     * @param _sclkPort the port of serial clock line.
     * @param _sclkPin the pin of serial clock line.
     * @param _mosiPort the port of MOSI line.
     * @param _mosiPin the pin of MOSI line.
     * @param _misoPort the port of MISO line.
     * @param _misoPin the pin of MISO line.
     * @param _remapped flag indicating whether the TX/RX pins shall be remapped.
     * @param _afio pointer to the AFIO module if it necessary for remapping.
     * @param _txRxIrq link to the USART global interrupt.
     * @param _txDma link to the transmitter DMA stream.
     * @param _rxDma to the receiver DMA stream.
     */
    explicit Spi (size_t _id,  SPI_TypeDef *_instance,
                  Port & _sclkPort, uint32_t _sclkPin,
                  Port & _mosiPort, uint32_t _mosiPin,
                  Port & _misoPort, uint32_t _misoPin,
                  bool _remapped, Afio * _afio,
                  Interrupt && _txRxIrq,
                  DmaStream && _txDma, DmaStream && _rxDma) :
        HalDevice { _id, _remapped },
        instance { _instance },
        sclkPin { _sclkPort, _sclkPin },
        mosiPin { _mosiPort, _mosiPin },
        misoPin { _misoPort, _misoPin },
        afio { _afio },
        txRxIrq { std::move(_txRxIrq) },
        txDma { std::move(_txDma) },
        rxDma { std::move(_rxDma) }
    {
        // empty
    }

    /**
     * @brief Helper method used to enable all interrupts for the module
     */
    void enableIrq () const
    {
        txRxIrq.enable();
        txDma.dmaIrq.enable();
        rxDma.dmaIrq.enable();
    }

    /**
     * @brief Helper method used to disable all interrupts for the module
     */
    void disableIrq () const
    {
        txRxIrq.disable();
        txDma.dmaIrq.disable();
        rxDma.dmaIrq.disable();
    }
};

} // end of namespace HardwareLayout
} // end of namespace Stm32async

#endif

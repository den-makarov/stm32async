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

#ifndef STM32ASYNC_SHARED_DEVICE_H_
#define STM32ASYNC_SHARED_DEVICE_H_

#include "IODevice.h"

namespace Stm32async
{

/**
 * @brief Base class that implements an shared asynchronous I/O interface.
 *
 * This class implements a state machine for a transmit/receive sequence,
 * handles the device occupation and monitors transmit/receive times in
 * order to process a communication timeout
 */
class SharedDevice
{
public:

    /**
     * @brief Available transmit/receive states.
     */
    enum class State
    {
        NONE,
        TX,
        TX_CMPL,
        RX,
        RX_CMPL,
        ERROR,
        TIMEOUT
    };

    /**
     * @brief An abstract interface of some device client.
     */
    class DeviceClient
    {
    public:

        virtual ~DeviceClient () = default;

        virtual bool onTransmissionFinished (State state) =0;
    };

    /**
     * @brief Default constructor.
     *
     * Sets the initial state of this device.
     */
    SharedDevice (const HardwareLayout::DmaStream * _txStream, const HardwareLayout::DmaStream * _rxStream,
                  uint32_t _periphDataAlignment, uint32_t _memDataAlignment);

    /**
     * @brief Communication start handler.
     *
     * This method shall be called from a derived class at the beginning of a transmit/receive session.
     * If the device shall be occupied within this communication session, the caller shall set a valid
     * DeviceClient. The caller shall also set the initial and target state for this communication session.
     * Communication will be finished if:
     * - current state will be equal to the target state
     * - an error occurs (current state is ERROR)
     * - a timeout occurs (current state is TIMEOUT)
     */
    void startCommunication (DeviceClient * client, State currState, State targetState);

    /**
     * @brief The method waits until communication is finished in a blocking mode.
     */
    void waitForRelease ();

    /**
     * @brief This setter allows to set a timeout for the communication sessions. Use __UINT32_MAX__ for unlimited timeout.
     */
    inline void setTimeout (uint32_t timeout)
    {
        this->timeout = timeout;
    }
    
    /**
     * @brief Procedure checks whether a given port of this device is used.
     */
    static bool isPortUsed (IOPort * p)
    {
        return p != NULL && p->getParameters().Pin != UNUSED_PIN;
    }
    
    /**
     * @brief Procedure checks whether a device has TX mode active.
     */
    inline bool isTxMode () const
    {
        return txStream != NULL;
    }

    /**
     * @brief Procedure checks whether a device has RX mode active.
     */
    inline bool isRxMode () const
    {
        return txStream != NULL;
    }

    /**
     * @brief The method returns the current state.
     */
    inline State getCurrState () const
    {
        return currState;
    }

    /**
     * @brief The method checks whether this device is occupied by any other device.
     */
    inline bool isOccupied () const
    {
        return client != NULL;
    }

    /**
     * @brief The method checks whether communication session is finished.
     */
    inline bool isFinished () const
    {
        return currState == targetState || currState == State::NONE || currState == State::ERROR
               || currState == State::TIMEOUT;
    }

    /**
     * @brief This procedure implements DMA TX interrupt handler and shall be called from the
     *        interrupt handlers section of the user program.
     *
     *     extern "C" {
     *         void DMA2_Stream6_IRQHandler (void)
     *         {
     *             "thisDevice"->processDmaTxInterrupt();
     *         }
     *     }
     */
    inline void processDmaTxInterrupt ()
    {
        HAL_DMA_IRQHandler(&txDma);
    }

    /**
     * @brief This procedure implements DMA RX interrupt handler and shall be called from the
     *        interrupt handlers section of the user program.
     *
     *     extern "C" {
     *         void DMA2_Stream3_IRQHandler (void)
     *         {
     *             "thisDevice"->processDmaRxInterrupt();
     *         }
     *     }
     */
    inline void processDmaRxInterrupt ()
    {
        HAL_DMA_IRQHandler(&rxDma);
    }

    /**
     * @brief This procedure handles an event that corresponds to the communication finalization.
     *
     * It shall be called from the interrupt handlers section of the user program.
     *
     *     extern "C" {
     *         void HAL_UART_TxCpltCallback (void)
     *         {
     *             "thisDevice"->processCallback(SharedDevice::State::TX_CMPL);
     *         }
     *     }
     *
     * Depending on the return value of onTransmissionFinished call for the current client that occupies
     * this device, the device can still be occupied and will be free again and can be used by an other device.
     */
    inline void processCallback (State state)
    {
        currState = state;
        if (client != NULL && client->onTransmissionFinished(state))
        {
            client = NULL;
        }
    }

    /**
     * @brief Handling of communication timeout.
     *
     * In order to allow timeout handling for this device, the user program shall periodically
     * (either from a main loop or timer-based) call this method.
     */
    inline void periodic ()
    {
        if (HAL_GetTick() - startTime > timeout)
        {
            processCallback(State::TIMEOUT);
        }
    }

protected:

    DeviceClient * client;
    volatile State currState;
    State targetState;
    uint32_t startTime, timeout;
    DMA_HandleTypeDef txDma;
    DMA_HandleTypeDef rxDma;

    DeviceStart::Status startDma (HAL_StatusTypeDef & halStatus);
    void stopDma ();

private:

    const HardwareLayout::DmaStream * txStream;
    const HardwareLayout::DmaStream * rxStream;
};

} // end namespace
#endif

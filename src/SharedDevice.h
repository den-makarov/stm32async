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
#include "IOPort.h"

#ifndef STM32ASYNC_SHARED_DEVICE_H_
#define STM32ASYNC_SHARED_DEVICE_H_

namespace Stm32async
{

/**
 * @brief Base class that implements a shared device.
 */
class SharedDevice
{
public:

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

    class DeviceClient
    {
    public:

        virtual ~DeviceClient ()
        {
            // empty
        }

        virtual bool onTransmissionFinished (State state) =0;
    };

    SharedDevice ();
    void startCommunication (DeviceClient * client, State currState, State targetState);
    void waitForRelease ();

    inline void setTimeout (uint32_t timeout)
    {
        this->timeout = timeout;
    }

    inline State getCurrState () const
    {
        return currState;
    }

    inline bool isOccupied () const
    {
        return client != NULL;
    }

    inline bool isFinished () const
    {
        return currState == targetState || currState == State::NONE || currState == State::ERROR
               || currState == State::TIMEOUT;
    }

    inline void processDmaTxInterrupt ()
    {
        HAL_DMA_IRQHandler(&txDma);
    }

    inline void processDmaRxInterrupt ()
    {
        HAL_DMA_IRQHandler(&rxDma);
    }

    inline void processCallback (State state)
    {
        currState = state;
        if (client != NULL && client->onTransmissionFinished(state))
        {
            client = NULL;
        }
    }

    inline void periodic ()
    {
        if (HAL_GetTick() - startTime > timeout)
        {
            processCallback(State::TIMEOUT);
        }
    }

protected:

    DeviceClient * client;
    State currState, targetState;
    uint32_t startTime, timeout;
    DMA_HandleTypeDef txDma;
    DMA_HandleTypeDef rxDma;
};

} // end namespace
#endif

/*******************************************************************************
 * StmPlusPlus: object-oriented library implementing device drivers for
 * STM32F3 and STM32F4 MCU
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

#ifndef DRIVERS_ESP8266_H_
#define DRIVERS_ESP8266_H_

#include "../Usart.h"
#include "../Rtc.h"
#include "Led.h"

namespace Stm32async
{
namespace Drivers
{

/************************************************************************
 * Class Esp8266
 ************************************************************************/

class Esp8266
{
public:
    
    enum class AsyncCmd
    {
        POWER_ON       = 0,
        ECHO_OFF       = 1,
        ENSURE_READY   = 2,
        SET_MODE       = 3,
        ENSURE_MODE    = 4,
        SET_ADDR       = 5,
        ENSURE_WLAN    = 6,
        CONNECT_WLAN   = 7,
        PING_SERVER    = 8,
        SET_CON_MODE   = 9,
        SET_SINDLE_CON = 10,
        CONNECT_SERVER = 11,
        SEND_MSG_SIZE  = 12,
        SEND_MESSAGE   = 13,
        RECONNECT      = 14,
        DISCONNECT     = 15,
        POWER_OFF      = 16,
        WAITING        = 17,
        OFF            = 18
    };

    enum class CommState
    {
        NONE    = 0,
        TX      = 1,
        TX_CMPL = 2,
        RX      = 3,
        SUCC    = 4,
        ERROR   = 5
    };

    static constexpr uint32_t ESP_BAUDRATE = 115200;
    static constexpr time_ms ESP_TIMEOUT = 30000L;
    static constexpr uint32_t BUFFER_SIZE = 1024;

public:
    
    Esp8266 (const HardwareLayout::Usart & _device, const HardwareLayout::Port & _powerPort, uint32_t _powerPin, Led * _sendLed = NULL);

    inline void processRxCpltCallback ()
    {
        commState = CommState::ERROR;
    }

    inline void processTxCpltCallback ()
    {
        commState = CommState::TX_CMPL;
    }
    
    inline void processErrorCallback ()
    {
        commState = CommState::ERROR;
    }

    inline void processDmaTxInterrupt ()
    {
        usart.processDmaTxInterrupt();
    }

    inline void processDmaRxInterrupt ()
    {
        usart.processDmaRxInterrupt();
    }

    inline void processInterrupt ()
    {
        usart.processInterrupt();
        if (commState == CommState::TX_CMPL)
        {
            commState = CommState::RX;
            rxIndex = 0;
            usart.changeMode(UART_MODE_RX);
            usart.receiveIt(NULL, rxBuffer, BUFFER_SIZE);
        }
        else if (commState == CommState::RX && rxIndex < BUFFER_SIZE)
        {
            if (rxIndex >= 1
                && rxBuffer[rxIndex-1] == '\r'
                && rxBuffer[rxIndex] == '\n')
            {
                if (shortOkResponse && rxIndex >= 3
                    && rxBuffer[rxIndex - 3] == 'O'
                    && rxBuffer[rxIndex - 2] == 'K')
                {
                    commState = CommState::SUCC;
                }
                else if (rxIndex >= 8
                        && rxBuffer[rxIndex - 8] == 'S'
                        && rxBuffer[rxIndex - 7] == 'E'
                        && rxBuffer[rxIndex - 6] == 'N'
                        && rxBuffer[rxIndex - 5] == 'D'
                        && rxBuffer[rxIndex - 4] == ' '
                        && rxBuffer[rxIndex - 3] == 'O'
                        && rxBuffer[rxIndex - 2] == 'K')
                {
                    commState = CommState::SUCC;
                }
                else if (rxIndex >= 6
                        && rxBuffer[rxIndex - 6] == 'E'
                        && rxBuffer[rxIndex - 5] == 'R'
                        && rxBuffer[rxIndex - 4] == 'R'
                        && rxBuffer[rxIndex - 3] == 'O'
                        && rxBuffer[rxIndex - 2] == 'R')
                {
                    commState = CommState::ERROR;
                }
            }
            ++rxIndex;
            if (commState == CommState::SUCC)
            {
                startListening();
            }
        }
        else if (listening && rxIndex < BUFFER_SIZE)
        {
            ++rxIndex;
        }
    }
    
    inline void setMode (int mode)
    {
        this->mode = mode;
    }

    inline void setIp (const char* ip)
    {
        this->ip = ip;
    }

    inline void setMask (const char* mask)
    {
        this->mask = mask;
    }

    inline void setGatway (const char* gatway)
    {
        this->gatway = gatway;
    }

    inline void setSsid (const char* ssid)
    {
        this->ssid = ssid;
    }

    inline void setPasswd (const char* passwd)
    {
        this->passwd = passwd;
    }

    inline void setProtocol (const char* protocol)
    {
        this->protocol = protocol;
    }

    inline void setServer (const char* server)
    {
        this->server = server;
    }

    inline void setPort (const char* port)
    {
        this->port = port;
    }

    inline void setMessage (const char* message)
    {
        this->message = message;
    }

    inline void setMessageSize (size_t messageSize)
    {
        this->messageSize = messageSize;
    }

    inline bool isResponceAvailable ()
    {
        return commState == CommState::SUCC || commState == CommState::ERROR;
    }

    inline bool isTransmissionStarted ()
    {
        return commState != CommState::NONE;
    }

    inline size_t getInputMessageSize () const
    {
        return inputMessageSize;
    }

    inline bool isListening ()
    {
        return listening;
    }
    
    inline const char* getProtocol () const
    {
        return protocol;
    }
    
    inline const char* getServer () const
    {
        return server;
    }

    inline const char* getPort () const
    {
        return port;
    }
    
    bool transmit (AsyncCmd cmd);
    bool getResponce (AsyncCmd cmd);
    void periodic ();
    void getInputMessage (char * buffer, size_t len);

private:

    const char * CMD_AT = "AT";
    const char * CMD_ECHO_OFF = "ATE0";
    const char * CMD_GETMODE = "AT+CWMODE?";
    const char * RESP_GETMODE = "+CWMODE:";
    const char * RESP_GETNET = "+CWLAP:";
    const char * CMD_SET_NORMAL_MODE = "AT+CIPMODE=0";
    const char * CMD_SET_SINGLE_CONNECTION = "AT+CIPMUX=0";
    const char * CMD_CONNECT_SERVER_RESPONCE = "CONNECT";
    const char * CMD_CLOSE_CONNECT = "AT+CIPCLOSE";
    const char * CMD_INPUT_MESSAGE = "+IPD,";
    const char * CMD_END = "\r\n";
    const char * RESP_READY = "ready\r\n";
    const char * UDP_PORT = "5888";

    AsyncUsart usart;
    IOPort pinPower;
    Led * sendLed;

    __IO CommState commState;
    __IO size_t rxIndex;
    __IO bool listening;
    __IO bool shortOkResponse;

    int mode;
    const char * ip;
    const char * gatway;
    const char * mask;
    const char * ssid;
    const char * passwd;
    const char * protocol;
    const char * server;
    const char * port;
    const char * message;
    size_t messageSize;
    time_ms operationEnd;
    const char * inputMessage;
    size_t inputMessageSize;

    char cmdBuffer[256];
    char txBuffer[BUFFER_SIZE];
    char rxBuffer[BUFFER_SIZE];
    char msgBuffer[BUFFER_SIZE];

    bool waitForResponce (const char * responce);
    bool sendCmd (const char * cmd, size_t cmdLen = 0, bool addCmdEnd = true);
    bool init ();
    bool applyMode ();
    bool applyIpAddress ();
    bool searchWlan ();
    bool connectToWlan ();
    bool ping ();
    bool connectToServer ();
    bool sendMessageSize ();
    bool sendMessage ();
    void processInputMessage ();
    void startListening ();
    void stopListening ();

    inline void powerOff ()
    {
        pinPower.setLow();
        pinPower.stop();
        usart.stop();
    }

    inline void setInputMessage (const char * msg, size_t len)
    {
        inputMessage = msg;
        inputMessageSize = len;
    }
};

/************************************************************************
 * Class EspSender
 ************************************************************************/

class EspSender
{
public:

    EspSender (Esp8266 & _esp, Led * _errorLed = NULL);

    inline bool isOutputMessageSent () const
    {
        return outputMessage == NULL;
    }

    inline Esp8266::AsyncCmd getEspState () const
    {
        return espState;
    }

    inline void setRepeatDelay (time_ms repeatDelay)
    {
        this->repeatDelay = repeatDelay;
    }

    inline void setTurnOffDelay (time_ms turnOffDelay)
    {
        this->turnOffDelay = turnOffDelay;
    }

    void sendMessage (const char* protocol, const char* server, const char* port, const char * msg, size_t messageSize = 0);
    void periodic ();

private:

    static constexpr size_t STATE_NUMBER = 17;

    class AsyncState
    {
    public:

        Esp8266::AsyncCmd cmd;
        Esp8266::AsyncCmd okNextCmd, errorNextCmd;
        const char * description;

        AsyncState(Esp8266::AsyncCmd _cmd, Esp8266::AsyncCmd _okState, const char * _description)
        {
            cmd = _cmd;
            okNextCmd = _okState;
            errorNextCmd = _cmd;
            description = _description;
        }

        AsyncState(Esp8266::AsyncCmd _cmd, Esp8266::AsyncCmd _okState, Esp8266::AsyncCmd _errorState, const char * _description)
        {
            cmd = _cmd;
            okNextCmd = _okState;
            errorNextCmd = _errorState;
            description = _description;
        }
    };

    Esp8266 & esp;
    Led * errorLed;
    Esp8266::AsyncCmd espState;
    const char * outputMessage;
    time_ms repeatDelay, turnOffDelay; // configured delays in millis
    time_ms nextOperationTime, turnOffTime;
    std::array<AsyncState, STATE_NUMBER> asyncStates;

    inline void delayNextOperation ()
    {
        nextOperationTime = Rtc::getInstance()->getUpTimeMillisec() + repeatDelay;
    }

    inline void delayTurnOff ()
    {
        turnOffTime = Rtc::getInstance()->getUpTimeMillisec() + turnOffDelay;
    }

    void stateReport (bool result, const char * description);
    const AsyncState * findState (Esp8266::AsyncCmd st);
};


/************************************************************************
 * Class NtpMessage
 ************************************************************************/

class NtpMessage
{
public:

    static constexpr size_t NTP_PACKET_SIZE = 48;  // NTP time is in the first 48 bytes of message
    struct NtpPacket {
            uint8_t flags;
            uint8_t stratum;
            uint8_t poll;
            uint8_t precision;
            uint32_t root_delay;
            uint32_t root_dispersion;
            uint8_t referenceID[4];
            uint32_t ref_ts_sec;
            uint32_t ref_ts_frac;
            uint32_t origin_ts_sec;
            uint32_t origin_ts_frac;
            uint32_t recv_ts_sec;
            uint32_t recv_ts_frac;
            uint32_t trans_ts_sec;
            uint32_t trans_ts_frac;
    } __attribute__((__packed__));

    struct NtpPacket ntpPacket;

    NtpMessage () = default;

    const char * getRequest ();
    void decodeResponce (const char * responce);

    inline void setLocalOffsetSec (time_t localOffsetSec = 0)
    {
        this->localOffsetSec = localOffsetSec;
    }

    inline time_t getLastUpdateTime () const
    {
        return lastUpdateTime;
    }

private:

    time_t localOffsetSec = 0;
    time_t lastUpdateTime = 0;
};

} // end of namespace Drivers
} // end of namespace Stm32async

#endif

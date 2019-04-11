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

#include "Esp8266.h"
#include "../UsartLogger.h"

using namespace Stm32async::Drivers;

#define USART_DEBUG_MODULE "ESP: "

/************************************************************************
 * Class Esp8266
 ************************************************************************/

Esp8266::Esp8266 (const HardwareLayout::Usart & _device, const HardwareLayout::Port & _powerPort, uint32_t _powerPin, Led * _sendLed) :
        usart { _device },
        pinPower { _powerPort, _powerPin, GPIO_MODE_OUTPUT_PP },
        sendLed { _sendLed },
        commState { CommState::NONE },
        rxIndex { 0 },
        listening { false },
        mode { -1 },
        ip { NULL },
        gatway { NULL },
        mask { NULL },
        ssid { NULL },
        passwd { NULL },
        protocol { NULL },
        server { NULL },
        port { NULL },
        message { NULL },
        messageSize { 0 },
        operationEnd { INFINITY_TIME },
        inputMessage { NULL },
        inputMessageSize { 0 }
{
    // empty
}

bool Esp8266::init ()
{
    DeviceStart::Status status = usart.start(UART_MODE_RX, ESP_BAUDRATE, UART_WORDLENGTH_8B,
                                             UART_STOPBITS_1, UART_PARITY_NONE, false);
    USART_DEBUG("USART status: " << DeviceStart::asString(status) << " (" << usart.getHalStatus() << ")" << UsartLogger::ENDL);
    if (status != DeviceStart::OK)
    {
        return false;
    }

    pinPower.start();
    pinPower.setHigh();
    bool isReady = waitForResponce(RESP_READY);
    usart.enableIrq();

    if (!isReady)
    {
        powerOff();
    }
    return isReady;
}

bool Esp8266::waitForResponce (const char * responce)
{
    bool retValue = false;
    size_t responceLen = ::strlen(responce);
    size_t idx = 0;
    while (true)
    {
        if (usart.receiveBlocking(rxBuffer, 1, ESP_TIMEOUT) != HAL_OK)
        {
            break;
        }
        
        if (rxBuffer[0] != responce[idx])
        {
            idx = 0;
            continue;
        }
        
        ++idx;
        if (idx == responceLen)
        {
            retValue = true;
            break;
        }
    }
    return retValue;
}

bool Esp8266::sendCmd (const char * cmd, size_t cmdLen, bool addCmdEnd)
{
    if (cmdLen == 0)
    {
        cmdLen = ::strlen(cmd);
        if (cmdLen == 0)
        {
            return false;
        }
    }
    
    USART_DEBUG(" -> " << cmd << UsartLogger::ENDL);
    stopListening();
    
    ::memset(msgBuffer, 0, BUFFER_SIZE);
    ::memset(rxBuffer, 0, BUFFER_SIZE);
    ::memcpy(txBuffer, cmd, cmdLen);
    if (addCmdEnd)
    {
        ::memcpy(txBuffer + cmdLen, CMD_END, 2);
        cmdLen += 2;
    }

    HAL_StatusTypeDef status = usart.changeMode(UART_MODE_TX);
    if (status != HAL_OK)
    {
        USART_DEBUG("Cannot start ESP USART/TX: " << status << UsartLogger::ENDL);
        return false;
    }

    status = usart.transmit(NULL, txBuffer, cmdLen);
    if (status != HAL_OK)
    {
        USART_DEBUG("Cannot transmit ESP request message: " << status << UsartLogger::ENDL);
        return false;
    }

    return true;
}

bool Esp8266::applyMode ()
{
    if (mode < 0)
    {
        return false;
    }
    ::sprintf(cmdBuffer, "AT+CWMODE=%d", mode);
    return sendCmd(cmdBuffer);
}

bool Esp8266::applyIpAddress ()
{
    if (ip == NULL || gatway == NULL || mask == NULL)
    {
        return false;
    }
    ::sprintf(cmdBuffer, "AT+CIPSTA_CUR=\"%s\",\"%s\",\"%s\"", ip, gatway, mask);
    return sendCmd(cmdBuffer);
}

bool Esp8266::searchWlan ()
{
    if (ssid == NULL)
    {
        return false;
    }
    ::sprintf(cmdBuffer, "AT+CWLAP=\"%s\"", ssid);
    return sendCmd(cmdBuffer);
}

bool Esp8266::connectToWlan ()
{
    if (ssid == NULL || passwd == NULL)
    {
        return false;
    }
    ::sprintf(cmdBuffer, "AT+CWJAP_CUR=\"%s\",\"%s\"", ssid, passwd);
    return sendCmd(cmdBuffer);
}

bool Esp8266::ping ()
{
    if (server == NULL)
    {
        return false;
    }
    ::sprintf(cmdBuffer, "AT+PING=\"%s\"", server);
    return sendCmd(cmdBuffer);
}

bool Esp8266::connectToServer ()
{
    if (protocol == NULL || server == NULL || port == NULL)
    {
        return false;
    }
    if (::strcmp(protocol, "UDP") == 0)
    {
        ::sprintf(cmdBuffer, "AT+CIPSTART=\"%s\",\"%s\",%s,%s", protocol, server, port, UDP_PORT);
    }
    else
    {
        ::sprintf(cmdBuffer, "AT+CIPSTART=\"%s\",\"%s\",%s", protocol, server, port);
    }
    return sendCmd(cmdBuffer);
}

bool Esp8266::sendMessageSize ()
{
    if (message == NULL || messageSize == 0)
    {
        return false;
    }
    ::sprintf(cmdBuffer, "AT+CIPSEND=%d", messageSize);
    return sendCmd(cmdBuffer);
}

bool Esp8266::sendMessage ()
{
    if (message == NULL)
    {
        return false;
    }
    shortOkResponse = false;
    return sendCmd(message, messageSize, false);
}

bool Esp8266::transmit (Esp8266::AsyncCmd cmd)
{
    if (isTransmissionStarted())
    {
        return false;
    }

    if (sendLed != NULL)
    {
        sendLed->turnOn();
    }

    commState = CommState::TX;
    operationEnd = Rtc::getInstance()->getUpTimeMillisec() + ESP_TIMEOUT;
    shortOkResponse = true;

    bool isReady = true;
    switch (cmd)
    {
    case AsyncCmd::POWER_ON:
        isReady = init();
        commState = CommState::SUCC;
        break;
    case AsyncCmd::ECHO_OFF:
        isReady = sendCmd(CMD_ECHO_OFF);
        break;
    case AsyncCmd::ENSURE_READY:
        isReady = sendCmd(CMD_AT);
        break;
    case AsyncCmd::SET_MODE:
        isReady = applyMode();
        break;
    case AsyncCmd::ENSURE_MODE:
        isReady = sendCmd(CMD_GETMODE);
        break;
    case AsyncCmd::SET_ADDR:
        isReady = applyIpAddress();
        break;
    case AsyncCmd::ENSURE_WLAN:
        isReady = searchWlan();
        break;
    case AsyncCmd::CONNECT_WLAN:
        isReady = connectToWlan();
        break;
    case AsyncCmd::PING_SERVER:
        isReady = ping();
        break;
    case AsyncCmd::SET_CON_MODE:
        isReady = sendCmd(CMD_SET_NORMAL_MODE);
        break;
    case AsyncCmd::SET_SINDLE_CON:
        isReady = sendCmd(CMD_SET_SINGLE_CONNECTION);
        break;
    case AsyncCmd::CONNECT_SERVER:
        isReady = connectToServer();
        break;
    case AsyncCmd::SEND_MSG_SIZE:
        isReady = sendMessageSize();
        break;
    case AsyncCmd::SEND_MESSAGE:
        isReady = sendMessage();
        break;
    case AsyncCmd::DISCONNECT:
    case AsyncCmd::RECONNECT:
        isReady = sendCmd(CMD_CLOSE_CONNECT);
        break;
    case AsyncCmd::POWER_OFF:
        powerOff();
        commState = CommState::SUCC;
        break;
    default:
        // nothing to do
        commState = CommState::SUCC;
        break;
    }

    if (!isReady)
    {
        commState = CommState::ERROR;
    }
    return isReady;
}

bool Esp8266::getResponce (AsyncCmd cmd)
{
    if (!isResponceAvailable())
    {
        return false;
    }

    bool retValue = false;
    if (commState == CommState::SUCC)
    {
        const char * responce = NULL;
        switch (cmd)
        {
        case AsyncCmd::ENSURE_MODE:
            responce = ::strstr(rxBuffer, RESP_GETMODE);
            retValue = responce != 0 && ::atoi(responce + ::strlen(RESP_GETMODE)) == mode;
            break;

        case AsyncCmd::ENSURE_WLAN:
            retValue = (::strstr(rxBuffer, RESP_GETNET) != NULL && ::strstr(rxBuffer, ssid) != NULL);
            break;

        case AsyncCmd::CONNECT_SERVER:
            retValue = (::strstr(rxBuffer, CMD_CONNECT_SERVER_RESPONCE) != NULL);
            break;

        default:
            // nothing to do
            retValue = true;
        }
    }

    commState = CommState::NONE;
    operationEnd = INFINITY_TIME;

    if (sendLed != NULL)
    {
        sendLed->turnOff();
    }

    return retValue;
}

void Esp8266::periodic ()
{
    if (listening && rxIndex > 0)
    {
        processInputMessage();
    }

    if (isResponceAvailable())
    {
        return;
    }

    if (Rtc::getInstance()->getUpTimeMillisec() > operationEnd)
    {
        commState = CommState::ERROR;
        USART_DEBUG("Cannot receive ESP response message: ESP_TIMEOUT" << UsartLogger::ENDL);
    }
}

void Esp8266::startListening ()
{
    listening = true;
    setInputMessage(NULL, 0);
    rxIndex = 0;
    usart.changeMode(UART_MODE_RX);
    usart.receiveIt(NULL, msgBuffer, BUFFER_SIZE);
}

void Esp8266::stopListening ()
{
    if (listening)
    {
        listening = false;
    }
}

void Esp8266::processInputMessage ()
{
    size_t idx = rxIndex;
    size_t messagePrefixLen = ::strlen(CMD_INPUT_MESSAGE);
    if (idx > messagePrefixLen)
    {
        const char * startIpd = ::strstr(msgBuffer, CMD_INPUT_MESSAGE);
        if (startIpd != NULL)
        {
            const char * startLen = startIpd + messagePrefixLen;
            const char * endLen = ::strchr(startLen, ':');
            size_t lenSize = endLen - startLen;
            if (endLen != NULL && lenSize > 0 && lenSize < 256)
            {
                ::memcpy(cmdBuffer, startLen, lenSize);
                cmdBuffer[lenSize] = 0;
                size_t msgSize = ::atoi(cmdBuffer);
                if (idx >= messagePrefixLen + lenSize + 1 + msgSize)
                {
                    setInputMessage(endLen + 1, msgSize);
                }
            }
        }
    }
}

void Esp8266::getInputMessage (char * buffer, size_t len)
{
    if (inputMessage != NULL && len > 0)
    {
        ::memcpy(buffer, inputMessage, len);
    }
    startListening();
}

/************************************************************************
 * Class EspSender
 ************************************************************************/

EspSender::EspSender (Esp8266 & _esp, Led * _errorLed) :
        esp { _esp },
        errorLed { _errorLed },
        espState { Esp8266::AsyncCmd::OFF },
        outputMessage { NULL },
        repeatDelay { 0 },
        turnOffDelay { 0 },
        nextOperationTime { 0 },
        turnOffTime { INFINITY_TIME },
        asyncStates { {
            AsyncState(Esp8266::AsyncCmd::POWER_ON,       Esp8266::AsyncCmd::ECHO_OFF,       "power on"),
            AsyncState(Esp8266::AsyncCmd::ECHO_OFF,       Esp8266::AsyncCmd::ENSURE_READY,   "echo off"),
            AsyncState(Esp8266::AsyncCmd::ENSURE_READY,   Esp8266::AsyncCmd::SET_MODE,       "ensure ready"),
            AsyncState(Esp8266::AsyncCmd::SET_MODE,       Esp8266::AsyncCmd::ENSURE_MODE,    "set ESP mode"),
            AsyncState(Esp8266::AsyncCmd::ENSURE_MODE,    Esp8266::AsyncCmd::SET_ADDR,       "ensure set ESP"),
            AsyncState(Esp8266::AsyncCmd::SET_ADDR,       Esp8266::AsyncCmd::CONNECT_WLAN,   "set IP"),
            AsyncState(Esp8266::AsyncCmd::CONNECT_WLAN,   Esp8266::AsyncCmd::SET_CON_MODE,   "connect SSID"),
            AsyncState(Esp8266::AsyncCmd::SET_CON_MODE,   Esp8266::AsyncCmd::SET_SINDLE_CON, "set normal connection mode"),
            AsyncState(Esp8266::AsyncCmd::SET_SINDLE_CON, Esp8266::AsyncCmd::CONNECT_SERVER, "set single connection"),
            //AsyncState(Esp8266::AsyncCmd::PING_SERVER,    Esp8266::AsyncCmd::CONNECT_SERVER, "ping"),
            AsyncState(Esp8266::AsyncCmd::CONNECT_SERVER, Esp8266::AsyncCmd::SEND_MSG_SIZE,  "connect server"),
            AsyncState(Esp8266::AsyncCmd::SEND_MSG_SIZE,  Esp8266::AsyncCmd::SEND_MESSAGE,   Esp8266::AsyncCmd::RECONNECT, "send message size"),
            AsyncState(Esp8266::AsyncCmd::SEND_MESSAGE,   Esp8266::AsyncCmd::WAITING,        Esp8266::AsyncCmd::RECONNECT, "send message"),
            AsyncState(Esp8266::AsyncCmd::RECONNECT,      Esp8266::AsyncCmd::SET_CON_MODE,   Esp8266::AsyncCmd::SET_CON_MODE, "reconnect server"),
            AsyncState(Esp8266::AsyncCmd::WAITING,        Esp8266::AsyncCmd::WAITING,        "waiting message"),
            AsyncState(Esp8266::AsyncCmd::DISCONNECT,     Esp8266::AsyncCmd::POWER_OFF,      Esp8266::AsyncCmd::POWER_OFF, "disconnect"),
            AsyncState(Esp8266::AsyncCmd::POWER_OFF,      Esp8266::AsyncCmd::OFF,            "power off"),
            AsyncState(Esp8266::AsyncCmd::OFF,            Esp8266::AsyncCmd::OFF,            "turned off")
        } }
{
    // empty
}

void EspSender::sendMessage (const char* protocol, const char * server, const char * port, const char * msg, size_t messageSize)
{
    outputMessage = msg;
    esp.setMessage(outputMessage);
    if (messageSize == 0)
    {
        messageSize = ::strlen(outputMessage);
    }
    esp.setMessageSize(messageSize);
    USART_DEBUG("Sending message to " << server << "/" << port << "[" << messageSize << "]: " << msg << UsartLogger::ENDL);

    bool serverChanged = false;
    if (esp.getProtocol() != NULL && esp.getServer() != NULL && esp.getPort() != NULL)
    {
        if (::strcmp(protocol, esp.getProtocol()) != 0 ||
            ::strcmp(server, esp.getServer()) != 0 ||
            ::strcmp(port, esp.getPort()) != 0)
        {
            USART_DEBUG("Connection parameters changed, reconnect" << UsartLogger::ENDL);
            serverChanged = true;
        }
    }
    esp.setProtocol(protocol);
    esp.setServer(server);
    esp.setPort(port);

    nextOperationTime = 0;
    if (espState == Esp8266::AsyncCmd::OFF)
    {
        espState = Esp8266::AsyncCmd::POWER_ON;
    }
    else if (espState == Esp8266::AsyncCmd::WAITING)
    {
        espState = serverChanged? Esp8266::AsyncCmd::RECONNECT : Esp8266::AsyncCmd::SEND_MSG_SIZE;
    }
}

void EspSender::periodic ()
{
    if (espState == Esp8266::AsyncCmd::OFF)
    {
        return;
    }

    time_ms currentTime = Rtc::getInstance()->getUpTimeMillisec();
    if (currentTime < nextOperationTime)
    {
        return;
    }

    if (espState == Esp8266::AsyncCmd::WAITING)
    {
        if (outputMessage == NULL && currentTime > turnOffTime)
        {
            espState = Esp8266::AsyncCmd::DISCONNECT;
        }
        else if (esp.isListening())
        {
            esp.periodic();
            if (esp.getInputMessageSize() > 0)
            {
                delayTurnOff();
            }
        }
        return;
    }

    if (!esp.isTransmissionStarted())
    {
        if (errorLed != NULL)
        {
            errorLed->turnOff();
        }
        const AsyncState * s = findState(espState);
        if (s == NULL)
        {
            USART_DEBUG("ESP state: state corrupted" << UsartLogger::ENDL);
            return;
        }

        if (s->cmd == Esp8266::AsyncCmd::POWER_ON)
        {
            turnOffTime = INFINITY_TIME;
        }
        if (s->cmd == Esp8266::AsyncCmd::SEND_MSG_SIZE)
        {
            if (outputMessage == NULL)
            {
                return;
            }
        }

        if (!esp.transmit(s->cmd))
        {
            USART_DEBUG("ESP state: " << s->description << " -> failed to start transmission" << UsartLogger::ENDL);
        }
    }
    else if (esp.isResponceAvailable())
    {
        const AsyncState * s = findState(espState);
        if (s == NULL)
        {
            USART_DEBUG("ESP state: state corrupted" << UsartLogger::ENDL);
            return;
        }
        if (esp.getResponce(s->cmd))
        {
            espState = s->okNextCmd;
            if (s->cmd == Esp8266::AsyncCmd::SEND_MESSAGE)
            {
                outputMessage = NULL;
                delayTurnOff();
            }
            stateReport(true, s->description);
        }
        else
        {
            espState = s->errorNextCmd;
            delayNextOperation();
            stateReport(false, s->description);
        }
        if (espState == Esp8266::AsyncCmd::POWER_OFF)
        {
            delayNextOperation();
        }
    }
    else
    {
        esp.periodic();
    }
}

void EspSender::stateReport (bool result, const char * description)
{
    if (result)
    {
        if (errorLed != NULL)
        {
            errorLed->turnOff();
        }
    }
    else
    {
        USART_DEBUG("ESP error: " << description << " -> ERROR" << UsartLogger::ENDL);
        if (errorLed != NULL)
        {
            errorLed->turnOn();
        }
    }
}

const EspSender::AsyncState * EspSender::findState (Esp8266::AsyncCmd st)
{
    for (auto & ss : asyncStates)
    {
        if (ss.cmd == st)
        {
            return &ss;
        }
    }
    return NULL;
}

/************************************************************************
 * Class NtpMessage
 ************************************************************************/

#define UNIX_OFFSET             2208988800L
#define ENDIAN_SWAP32(data)     ((data >> 24) | /* right shift 3 bytes */ \
                                ((data & 0x00ff0000) >> 8) | /* right shift 1 byte */ \
                                ((data & 0x0000ff00) << 8) | /* left shift 1 byte */ \
                                ((data & 0x000000ff) << 24)) /* left shift 3 bytes */

const char * NtpMessage::getRequest ()
{
    ::memset(&ntpPacket, 0, NTP_PACKET_SIZE);
    ntpPacket.flags = 0xe3;
    return (const char *)(&ntpPacket);
}

void NtpMessage::decodeResponce (const char * responce)
{
    ::memcpy(&ntpPacket, responce, NTP_PACKET_SIZE);
    ntpPacket.root_delay = ENDIAN_SWAP32(ntpPacket.root_delay);
    ntpPacket.root_dispersion = ENDIAN_SWAP32(ntpPacket.root_dispersion);
    ntpPacket.ref_ts_sec = ENDIAN_SWAP32(ntpPacket.ref_ts_sec);
    ntpPacket.ref_ts_frac = ENDIAN_SWAP32(ntpPacket.ref_ts_frac);
    ntpPacket.origin_ts_sec = ENDIAN_SWAP32(ntpPacket.origin_ts_sec);
    ntpPacket.origin_ts_frac = ENDIAN_SWAP32(ntpPacket.origin_ts_frac);
    ntpPacket.recv_ts_sec = ENDIAN_SWAP32(ntpPacket.recv_ts_sec);
    ntpPacket.recv_ts_frac = ENDIAN_SWAP32(ntpPacket.recv_ts_frac);
    ntpPacket.trans_ts_sec = ENDIAN_SWAP32(ntpPacket.trans_ts_sec);
    ntpPacket.trans_ts_frac = ENDIAN_SWAP32(ntpPacket.trans_ts_frac);
    time_t total_secs = ntpPacket.recv_ts_sec - UNIX_OFFSET; /* convert to unix time */;
    lastUpdateTime = total_secs + localOffsetSec;
    Rtc::getInstance()->setTimeSec(lastUpdateTime);
    USART_DEBUG("NTP time: " << Rtc::getInstance()->getLocalTime() << UsartLogger::ENDL);
}

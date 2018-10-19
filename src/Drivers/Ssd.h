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

#ifndef DRIVERS_SSD_H_
#define DRIVERS_SSD_H_

#include "../Spi.h"

namespace Stm32async
{
namespace Drivers
{

/** 
 * @brief Class that describes a seven segment display connected to a shift register IC.
 *        The shift register IC is connected via the SPI interface.
 */
class Ssd_74XX595 : public SharedDevice::DeviceClient
{
public:

    class SegmentsMask
    {
    public:
        unsigned char top;
        unsigned char rightTop;
        unsigned char rightBottom;
        unsigned char bottom;
        unsigned char leftBottom;
        unsigned char leftTop;
        unsigned char center;
        unsigned char dot;
        SegmentsMask ();
    };

    static constexpr int SEG_NUMBER = 5;

    Ssd_74XX595 (AsyncSpi & _spi, const HardwareLayout::Port & _csPort, uint32_t _csPin, bool _inverse);

    inline void setSegmentsMask (const SegmentsMask & sm)
    {
        this->sm = sm;
    }

    inline void start ()
    {
        csPin.start();
    }

    inline void stop ()
    {
        csPin.stop();
    }

    void putString (const char * str, const bool * dots, uint16_t segNumbers);

    void putDots (const bool * dots, uint16_t segNumbers);

    virtual bool onTransmissionFinished (SharedDevice::State state);

private:

    AsyncSpi & spi;
    IOPort csPin;
    SegmentsMask sm;
    uint8_t segData[SEG_NUMBER];
    bool inverse;

    char getBits (char c, bool dot) const;

};

} // end of namespace Drivers
} // end of namespace Stm32async

#endif

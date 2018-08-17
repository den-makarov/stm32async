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

#include "Ssd.h"

using namespace Stm32async::Drivers;

#define setBitToTrue(uInt8Val, bitNr)   (uInt8Val |= (1 << bitNr))
#define setBitToFalse(uInt8Val, bitNr)  (uInt8Val &= ~(1 << bitNr))

/************************************************************************
 * Class Ssd_74HC595_SPI
 ************************************************************************/
Ssd_74XX595::SegmentsMask::SegmentsMask ()
{
    top = 0;
    rightTop = 1;
    rightBottom = 2;
    bottom = 3;
    leftBottom = 4;
    leftTop = 5;
    center = 6;
    dot = 7;
}

Ssd_74XX595::Ssd_74XX595 (AsyncSpi & _spi, const HardwareLayout::Port & _csPort, uint32_t _csPin, bool _inverse) :
    spi { _spi },
    csPin { _csPort, _csPin, GPIO_MODE_OUTPUT_PP },
    sm {},
    inverse { _inverse }
{
    // empty
}

void Ssd_74XX595::putString (const char * str, const bool * dots, uint16_t segNumbers)
{
    if (segNumbers >= SEG_NUMBER)
    {
        return;
    }
    for (int i = 0; i < segNumbers; ++i)
    {
        bool d = dots == NULL ? false : dots[segNumbers - 1 - i];
        segData[i] = getBits(str[segNumbers - 1 - i], d);
        if (inverse)
        {
            segData[i] = ~segData[i];
        }
    }
    csPin.setLow();
    spi.transmit(this, &segData[0], segNumbers);
}

void Ssd_74XX595::putDots (const bool * dots, uint16_t segNumbers)
{
    if (segNumbers >= SEG_NUMBER)
    {
        return;
    }
    for (int i = 0; i < segNumbers; ++i)
    {
        bool d = inverse ? !dots[segNumbers - 1 - i] : dots[segNumbers - 1 - i];
        if (d)
        {
            setBitToTrue(segData[i], sm.dot);
        }
        else
        {
            setBitToFalse(segData[i], sm.dot);
        }
    }
    csPin.setLow();
    spi.transmit(this, &segData[0], segNumbers);
}

bool Ssd_74XX595::onTransmissionFinished (SharedDevice::State /*state*/)
{
    while (spi.isBusy());
    csPin.setHigh();
    return true;
}

char Ssd_74XX595::getBits (char c, bool dot) const
{
    char bits = 0;
    switch (c)
    {
    case '.':
        bits |= (1 << sm.dot);
        break;
    case '-':
        bits |= (1 << sm.center);
        break;
    case 0:
    case '0':
        bits = (1 << sm.top) | (1 << sm.rightTop) | (1 << sm.rightBottom) | (1 << sm.bottom)
               | (1 << sm.leftBottom) | (1 << sm.leftTop);
        break;
    case 1:
    case '1':
        bits = (1 << sm.rightTop) | (1 << sm.rightBottom);
        break;
    case 2:
    case '2':
        bits = (1 << sm.top) | (1 << sm.rightTop) | (1 << sm.center) | (1 << sm.leftBottom)
               | (1 << sm.bottom);
        break;
    case 3:
    case '3':
        bits = (1 << sm.top) | (1 << sm.rightTop) | (1 << sm.center) | (1 << sm.rightBottom)
               | (1 << sm.bottom);
        break;
    case 4:
    case '4':
        bits = (1 << sm.leftTop) | (1 << sm.center) | (1 << sm.rightTop) | (1 << sm.rightBottom);
        break;
    case 5:
    case '5':
        bits = (1 << sm.top) | (1 << sm.leftTop) | (1 << sm.center) | (1 << sm.rightBottom)
               | (1 << sm.bottom);
        break;
    case 6:
    case '6':
        bits = (1 << sm.top) | (1 << sm.leftTop) | (1 << sm.center) | (1 << sm.rightBottom)
               | (1 << sm.bottom) | (1 << sm.leftBottom);
        break;
    case 7:
    case '7':
        bits = (1 << sm.top) | (1 << sm.rightTop) | (1 << sm.rightBottom);
        break;
    case 8:
    case '8':
        bits = (1 << sm.top) | (1 << sm.rightTop) | (1 << sm.center) | (1 << sm.rightBottom)
               | (1 << sm.bottom) | (1 << sm.leftBottom) | (1 << sm.leftTop);
        break;
    case 9:
    case '9':
        bits = (1 << sm.top) | (1 << sm.rightTop) | (1 << sm.center) | (1 << sm.rightBottom)
               | (1 << sm.bottom) | (1 << sm.leftTop);
        break;
    default:
        break;
    }
    if (dot)
    {
        bits |= (1 << sm.dot);
    }
    return bits;
}

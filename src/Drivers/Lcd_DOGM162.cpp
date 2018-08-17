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

#include "Lcd_DOGM162.h"

using namespace Stm32async::Drivers;

/************************************************************************
 * Class Lcd_DOGM162
 ************************************************************************/
Lcd_DOGM162::Lcd_DOGM162(AsyncSpi & _spi,
                         const HardwareLayout::Port & _csPort, uint32_t _csPin,
                         const HardwareLayout::Port & _rsPort, uint32_t _rsPin,
                         bool _bias, uint8_t _contrast):
    spi { _spi },
    csPin { _csPort, _csPin, GPIO_MODE_OUTPUT_PP },
    rsPin { _rsPort, _rsPin, GPIO_MODE_OUTPUT_PP }
{
    bias = _bias? 0b00011100 : 0b00010100;
    contrast1 = (0b00110000 & _contrast) >> 4;
    contrast2 = (0b00001111 & _contrast);
    linesNumber = 2;
}


Stm32async::DeviceStart::Status Lcd_DOGM162::start (uint8_t n)
{
    csPin.start();
    csPin.setHigh();
    rsPin.start();
    rsPin.setHigh();
    init(n);
    return Stm32async::DeviceStart::Status::OK;
}


void Lcd_DOGM162::stop ()
{
    csPin.stop();
    rsPin.stop();
}


void Lcd_DOGM162::init (uint8_t n)
{
    linesNumber = n;
    rsPin.setLow();

    csPin.setLow();
    HAL_Delay(1);

    uint8_t lineMask = n == 1? 0b00000100 : 0b00001000;
    spi.putChar(0b00100001 | lineMask); // Function Set ; 8 Bit; 2Zeilen, Istr.Tab 1
    HAL_Delay(1);

    spi.putChar(bias); // Bias Set: BS=0, FX=0
    HAL_Delay(1);

    spi.putChar(0b01011100 | contrast1); // Power/ICON/Contrast: Icon=1, Bon=1, C5=1, C4=1
    HAL_Delay(1);

    spi.putChar(0b01110000 | contrast2); // Contrast Set: C3=1, C2=C1=C0=0
    HAL_Delay(1);

    spi.putChar(0b01101010); // Follower Ctrl: Fon=1, Rab2=0, Rab1=1, Rab0=0
    HAL_Delay(1);

    spi.putChar(0x0C); // DISPLAY ON: D=1, C=0, B=0
    HAL_Delay(1);

    spi.putChar(0x01); // CLEAR DISPLAY
    HAL_Delay(1);

    spi.putChar(0x06); // Entry mode set: I/D=1, S=0
    HAL_Delay(1);

    csPin.setHigh();
}


void Lcd_DOGM162::putString (const char * pData, uint16_t pSize)
{
    rsPin.setHigh();
    csPin.setLow();
    spi.transmit(this, (uint8_t *)pData, pSize);
}


void Lcd_DOGM162::writeData (bool isData, uint8_t data)
{
    rsPin.putBit(isData);
    csPin.setLow();
    HAL_Delay(1);
    spi.putChar(data);
    HAL_Delay(1);
    csPin.setHigh();
}


bool Lcd_DOGM162::onTransmissionFinished (SharedDevice::State /*state*/)
{
    while (spi.isBusy());
    csPin.setHigh();
    return true;
}

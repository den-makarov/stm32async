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

#include "IODevice.h"

using namespace Stm32async;

const char * DeviceStart::strings[] = {
    "Device started",
    "Can not initialize device",
    "Can not initialize TX DMA channel",
    "Can not initialize RX DMA channel",
    "SD Card is not inserted",
    "Can not initialize SD Wide Bus Operation",
    "Can not read SD Card status",
    "Can not link FAT FS driver",
    "Can not mount FAT FS volume",
    "Can not retrieve FAT FS volume label",
    "Can not retrieve FAT FS current directory",
    "Can not configure ACD channel",
    "Can not start ACD channel",
    "Can not configure DAC channel",
    "Can not start DAC channel",
    "Can not start timer",
    "Unknown error"
};

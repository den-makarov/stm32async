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

#ifndef STM32ASYNC_H_
#define STM32ASYNC_H_

#include "HardwareLayout/HardwareLayout.h"

#include <cstring>
#include <cstdlib>
#include <array>

/**
 * @brief Namespace containing classes that implement I/O devices
 */
namespace Stm32async
{

typedef int32_t duration_sec;
typedef uint64_t time_ms;

/**
 * @brief Helper define that allows us to declare a static "instance" attribute within a device
 */
#define DECLARE_STATIC_INSTANCE(name) \
    private: \
        static name * instance; \
    public: \
        static name * getInstance () { return instance; }


/**
 * @brief Template class providing operator () for converting type T argument
 *        to a string
 */
template<typename T, size_t size, const char * strings[]> class AsStringClass
{
public:
    /**
     * @brief Returns identifier for given enumeration value.
     */
    const char * & operator() (const T & val) const
    {
        if (val >= 0 && val < size)
        {
            return strings[val];
        }
        else
        {
            return strings[size];
        }
    }
};

} // end namespace
#endif

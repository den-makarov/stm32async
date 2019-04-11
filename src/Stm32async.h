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

typedef uint64_t time_ms;
typedef int64_t duration_ms;

#define UNDEFINED_PRIO __UINT32_MAX__
#define INFINITY_SEC __INT32_MAX__
#define INFINITY_TIME __UINT64_MAX__
#define FIRST_CALENDAR_YEAR 1900
#define MILLIS_IN_SEC 1000L

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

/**
 * @brief Template class providing operator () for converting a string argument
 *        to a value of type type T
 */
template<typename T, size_t size, const char * strings[]> class ConvertClass
{
public:
    /**
     * @brief Converts a string to an enumeration literal.
     *
     * @return True if conversion was successful.
     */
    bool operator() (const char * image, T& value) const
    {
        for (size_t i = 0; i < size; ++i)
        {
            if (::strcmp(image, strings[i]) == 0)
            {
                // everything's OK:
                value = static_cast<T>(i);
                return true;
            }
        }

        // not found:
        value = static_cast<T>(0);
        return false;
    }
};

} // end namespace
#endif

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

#if defined(STM32F4)
    #include "stm32f4xx.h"
#elif defined(STM32F1)
    #include "stm32f1xx.h"
#else
    #error "Please select first the target STM32Fxxx device used in your application (in stm32fxxx.h file)"
#endif

#ifndef STM32ASYNC_H_
#define STM32ASYNC_H_

/**
 * @brief Namespace containing classes that implement I/O devices
 */
namespace Stm32async
{

/**
 * @brief Helper define that allows us to declare a static "instance" attribute within a device
 */
#define DECLARE_STATIC_INSTANCE(name) \
    private: \
        static name * instance; \
    public: \
        static name * getInstance () { return instance; }

/**
 * @brief Helper define that allows us to declare a non-static "instance" attribute within a device
 */
#define DECLARE_INSTANCE(name) \
    private: \
        name * instance; \
    public: \
        name * getInstance () const { return instance; }

} // end namespace
#endif

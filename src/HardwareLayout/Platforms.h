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

#ifndef HARDWARE_LAYOUT_PLATFORMS_H_
#define HARDWARE_LAYOUT_PLATFORMS_H_

#if defined(STM32F4)

    /**
     * @brief Platform-dependent common declarations for series F4
     */
    #include "stm32f4xx.h"

    #define HAL_EXT_MAX_FLASH_LATENCY FLASH_LATENCY_7
    #define HAL_EXT_DMA_SET_CHANNEL(cfg, channel) (cfg.Init.Channel = channel)
    #define HAL_EXT_DMA_SET_FIFOMODE(cfg, mode) (cfg.Init.FIFOMode = mode)
    #define HAL_EXT_SetRtcTimer_IT(cfg, counter, prescaler) HAL_RTCEx_SetWakeUpTimer_IT(cfg, counter, prescaler)
    #define HAL_EXT_RtcTimerIRQHandler(cfg) HAL_RTCEx_WakeUpTimerIRQHandler(cfg)
    #define HAL_EXT_DeactivateRtcTimer(cfg) HAL_RTCEx_DeactivateWakeUpTimer(cfg)

    namespace Stm32async
    {
    namespace HardwareLayout
    {
        typedef struct
        {
            bool remap;
        } AFIO_TypeDef;

        typedef DMA_Stream_TypeDef DMA_Stream_Struct;

    } // end of namespace HardwareLayout
    } // end of namespace Stm32async

#elif defined(STM32F1)

    /**
     * @brief Platform-dependent common declarations for series F1
     */
    #include "stm32f1xx.h"

    #define GPIO_AF0_MCO 0U

    #define DMA_FIFOMODE_DISABLE 0U

    #define HAL_EXT_MAX_FLASH_LATENCY FLASH_LATENCY_2
    #define HAL_EXT_DMA_SET_CHANNEL(cfg, channel) UNUSED(channel)
    #define HAL_EXT_DMA_SET_FIFOMODE(cfg, mode) UNUSED(mode)
    #define HAL_EXT_SetRtcTimer_IT(cfg, counter, prescaler) HAL_RTCEx_SetSecond_IT(cfg)
    #define HAL_EXT_RtcTimerIRQHandler(cfg) HAL_RTCEx_RTCIRQHandler(cfg)
    #define HAL_EXT_DeactivateRtcTimer(cfg) HAL_RTCEx_DeactivateSecond(cfg)

    namespace Stm32async
    {
    namespace HardwareLayout
    {
        typedef struct
        {
            /**
            * @brief PLLSource: PLL entry clock source.
            *        This parameter must be a value of @ref RCC_PLL_Clock_Source
            *        Select value either RCC_PLLSOURCE_HSI_DIV2 or RCC_PLLSOURCE_HSE
            */
            uint32_t PLLSource;

             /**
             * @brief HSEPredivValue: The Prediv1 factor value (named PREDIV1 or PLLXTPRE in RM)
             *        This parameter can be a value of @ref RCCEx_Prediv1_Factor
             *        Select value from RCC_HSE_PREDIV_DIV2 to RCC_HSE_PREDIV_DIV16
             */
            uint32_t HSEPredivValue;

            /**
             * @brief Prediv1Source: The Prediv1 source available only for 105 and 107 group.
             *        This parameter can be a value of @ref RCCEx_Prediv1_Source
             *        Select value either RCC_PREDIV1_SOURCE_HSE or RCC_PREDIV1_SOURCE_PLL2
             */
            #if defined(STM32F105xC) || defined(STM32F107xC)
            uint32_t Prediv1Source;
            #endif /* STM32F105xC || STM32F107xC */

            /**
             * @brief PLLMUL: Multiplication factor for PLL VCO input clock
             *        This parameter must be a value of @ref RCCEx_PLL_Multiplication_Factor
             *        Select value from RCC_PLL_MUL2 to RCC_PLL_MUL16
             * @note  For STM32F105xC or STM32F107xC available options are only:
             *        RCC_PLL_MUL4 to RCC_PLL_MUL9 and RCC_PLL_MUL6_5
             */
            uint32_t PLLMUL;

            /**
             * @brief PLL2MUL: Multiplication factor for PLL2 VCO input clock
             *        This parameter must be a value of @ref RCCEx_PLL2_Multiplication_Factor
             *        Available values: RCC_PLL2_MULx where x is one of the options: 8-14, 16, 20
             */
            uint32_t PLL2MUL;

            /**
             * @brief HSEPrediv2Value: The Prediv2 factor value
             *        This parameter can be a value of @ref RCCEx_Prediv2_Factor
             *        Select value from RCC_HSE_PREDIV2_DIV1 to RCC_HSE_PREDIV2_DIV16
             */
            uint32_t HSEPrediv2Value;
        } SystemPllFactors;

        typedef DMA_Channel_TypeDef DMA_Stream_Struct;

    } // end of namespace HardwareLayout
    } // end of namespace Stm32async

#else

    #error "Please select first the target STM32Fxxx device used in your application (in stm32fxxx.h file)"

#endif

#endif

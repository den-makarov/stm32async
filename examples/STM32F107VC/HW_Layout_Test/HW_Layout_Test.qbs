import qbs

Project {
    minimumQbsVersion: "1.7.1"

    CppApplication {
        property string cmsisCoreVersion: "4.30"
        // Available options: 3.20 (02.15.13); 4.10 (03.18.15); 4.30 (10.20.15)

        property string targetDefinitionVersion: "4.2.0"
        // Available options: 3.4.0 (15.10.10); 4.0.0 (12.16.14); 4.2.0 (03.31.17)

        property bool USE_SEMIHOSTING: false
        property bool USE_HAL_DRIVER: true
        property bool STM3210C_EVAL_BOARD: true
        property bool stm32async: true

        property string SYS_LIB_PATH: "../CMSIS/system_stm32f1xx_v"
        property string CORE_LIB_PATH: "../CMSIS/core_v"

        property string BSP_SRC_PATH: "../../../device_support/BSP/stm3210c_eval"
        property string HAL_INC_PATH: "../../../device_support/STM32F1xx_HAL_Driver/Inc"
        property string HAL_SRC_PATH: "../../../device_support/STM32F1xx_HAL_Driver/Src"
        property string STM32ASYNC_PATH: "../stm32async"

        cpp.debugInformation: qbs.buildVariant == "debug" ? true : false
        cpp.optimization: qbs.buildVariant == "debug" ? "none" : "fast"
        cpp.warningLevel: "all"
        cpp.enableExceptions: false
        cpp.executableSuffix: ".elf"
        cpp.cxxLanguageVersion: "c++14"
        cpp.cLanguageVersion: "c11"
        cpp.positionIndependentCode: false

        cpp.defines: [
            //"__NO_SYSTEM_INIT", // comment this if using system initialization before call main
            STM3210C_EVAL_BOARD == true ? "USE_STM3210C_EVAL_BOARD_DRIVERS" : "DONT_USE_STM3210C_EVAL_BOARD_DRIVERS",
            USE_HAL_DRIVER == true ? "USE_HAL_DRIVER" : "DONT_USE_HAL_DRIVER",
            qbs.buildVariant == "debug" ? "DEBUG" : "RELEASE",
            "USE_STM3210C_EVAL",
            "STM32F1",
        ]

        cpp.driverFlags: [
            "-mthumb",
            "-mcpu=cortex-m3",
            "--specs=nano.specs",
            USE_SEMIHOSTING == true ? "--specs=rdimon.specs" : "--specs=nosys.specs"
        ]

        cpp.commonCompilerFlags: [
            "-fdata-sections",
            "-ffunction-sections"
        ]

        cpp.linkerFlags:[
            "-T",path+"/../linker/STM32F107VC.ld",
            "-Map="+path+"/memory.map"
        ]

        cpp.includePaths: [
            CORE_LIB_PATH+cmsisCoreVersion,
            SYS_LIB_PATH+targetDefinitionVersion,
            BSP_SRC_PATH,
            HAL_INC_PATH,
            STM32ASYNC_PATH
        ]

        Properties {
            condition: qbs.buildVariant === "release"
            cpp.driverFlags: outer.concat("-flto")
            cpp.linkerFlags: outer.concat("--gc-sections")
        }

        Group {
            name: "Base modules"
            files: [
                "src/main.cpp",
                //"src/stm32f1xx_it.c",
                "../startup/startup_ARMCM3.S",
                SYS_LIB_PATH+targetDefinitionVersion+"/system_stm32f1xx.c"
            ]
        }

        Group {
            name: "HAL_DRIVERS"
            condition: USE_HAL_DRIVER == true
            files: [
                HAL_SRC_PATH+"/stm32f1xx_hal.c",
                HAL_SRC_PATH+"/stm32f1xx_hal_rcc.c",
                HAL_SRC_PATH+"/stm32f1xx_hal_gpio.c",
                HAL_SRC_PATH+"/stm32f1xx_hal_spi.c",
                HAL_SRC_PATH+"/stm32f1xx_hal_i2c.c",
                HAL_SRC_PATH+"/stm32f1xx_hal_dma.c",
                HAL_SRC_PATH+"/stm32f1xx_hal_uart.c",
                HAL_SRC_PATH+"/stm32f1xx_hal_msp_user.c",
                HAL_SRC_PATH+"/stm32f1xx_hal_cortex.c"
            ]
        }

        Group {
            name: "STM3210C_EVAL_BOARD_DRIVERS"
            condition: STM3210C_EVAL_BOARD == true
            files: [
                BSP_SRC_PATH+"/stm3210c_eval.c",
                BSP_SRC_PATH+"/stm3210c_eval_io.c",
                //BSP_SRC_PATH+"/stm3210c_eval_audio.c",
                BSP_SRC_PATH+"/stm3210c_eval_lcd.c",
                BSP_SRC_PATH+"/Components/stmpe811/stmpe811.c",
                BSP_SRC_PATH+"/Components/ili9320/ili9320.c",
                BSP_SRC_PATH+"/Components/ili9325/ili9325.c"
                //BSP_SRC_PATH+"/stm3210c_eval_accelerometer.c",
            ]
        }

        Group {
            name: "STM32_ASYNC_IO_LIB"
            condition: stm32async == true
            files: [
                STM32ASYNC_PATH+"/IOPort.cpp",
                STM32ASYNC_PATH+"/SystemClock.cpp",
                STM32ASYNC_PATH+"/AsyncUsart.cpp",
                STM32ASYNC_PATH+"/SharedDevice.cpp",
                STM32ASYNC_PATH+"/UsartLogger.cpp",
                //STM32ASYNC_PATH+"/Rtc.cpp",
            ]
        }

        Group {     // Properties for the produced executable
            fileTagsFilter: "application"
            qbs.install: true
        }
    }
}

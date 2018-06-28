#ifndef MAIN_H
#define MAIN_H

#ifdef  USE_HAL_DRIVER
#include "stm32f1xx_hal.h"
//#include "stm32fxxx_hal.h"
#endif /* USE_HAL_DRIVER */

#ifdef  USE_STM3210C_EVAL_BOARD_DRIVERS
#include "stm3210c_eval.h"
#include "stm3210c_eval_lcd.h"
#endif /* USE_STM3210_EVAL_BOARD_DRIVERS */

#endif /* MAIN_H */

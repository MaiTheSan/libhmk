/**
  **************************************************************************
  * @file     at32f402_405_board.c
  * @brief    header file for at-start board. set of firmware functions to
  *           manage leds and push-button. initialize delay function.
  **************************************************************************
  *                       Copyright notice & Disclaimer
  *
  * The software Board Support Package (BSP) that is made available to
  * download from Artery official website is the copyrighted work of Artery.
  * Artery authorizes customers to use, copy, and distribute the BSP
  * software and its related documentation for the purpose of design and
  * development in conjunction with Artery microcontrollers. Use of the
  * software is governed by this copyright notice and the following disclaimer.
  *
  * THIS SOFTWARE IS PROVIDED ON "AS IS" BASIS WITHOUT WARRANTIES,
  * GUARANTEES OR REPRESENTATIONS OF ANY KIND. ARTERY EXPRESSLY DISCLAIMS,
  * TO THE FULLEST EXTENT PERMITTED BY LAW, ALL EXPRESS, IMPLIED OR
  * STATUTORY OR OTHER WARRANTIES, GUARANTEES OR REPRESENTATIONS,
  * INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT.
  *
  **************************************************************************
  */

#ifndef __AT32F402_405_BOARD_H
#define __AT32F402_405_BOARD_H

#ifdef __cplusplus
extern "C" {
#endif
#define AT_START_F405_V1
#include "stdio.h"
#include "common.h"

#include "at32f402_405.h"

/** @addtogroup AT32F402_405_board
  * @{
  */

/** @addtogroup BOARD
  * @{
  */

/** @defgroup BOARD_pins_definition
  * @{
  */

/**
  * this header include define support list:
  *   1. at-start-f402 v1.x board
  *   2. at-start-f405 v1.x board
  * if define AT_START_F402_V1, the header file support at-start-f402 v1.x board
  * if define AT_START_F405_V1, the header file support at-start-f405 v1.x board
  */

#if !defined (AT_START_F402_V1) && !defined (AT_START_F405_V1)
#error "please select first the board at-start device used in your application (in board_def.h file)"
#endif

extern  __IO uint32_t TickCount;

/**
  * @}
  */

/** @defgroup BOARD_exported_functions
  * @{
  */

/******************** functions ********************/
void board_init(void);
void board_error_handler(void);
void board_reset(void);

void timer_init(void);

/* delay function */
// void timer_init(void);

/* exported functions */
// void system_clock_config(void);
/**
  * @}
  */

/**
  * @}
  */
  //--------------------------------------------------------------------+
  // Flash Configuration
  //--------------------------------------------------------------------+

  // Flash size in bytes
  #if defined (AT32F402_405xC)
  #define FLASH_SIZE                       (256 * 1024)   /* 256KB flash */
  #define FLASH_SECTOR_SIZE                2048           /* 2KB sectors */
  #else
  #define FLASH_SIZE                       (128 * 1024)   /* 128KB flash */
  #define FLASH_SECTOR_SIZE                1024           /* 1KB sectors */
  #endif

#define AT32F402_405xC
#define FLASH_NUM_SECTORS                (FLASH_SIZE / FLASH_SECTOR_SIZE)
#define FLASH_EMPTY_VAL 0xFFFFFFFF

  //--------------------------------------------------------------------+
  // ADC Configuration
  //--------------------------------------------------------------------+

  // Number of ADC channels
#define ADC_NUM_CHANNELS 16

#if !defined(ADC_RESOLUTION)
  // ADC resolution in bits
#define ADC_RESOLUTION 12
#endif

  // Maximum ADC value
#define ADC_MAX_VALUE ((1 << ADC_RESOLUTION) - 1)

#if !defined(ADC_NUM_SAMPLE_CYCLES)
  // Number of sample cycles for each ADC conversion
#define ADC_NUM_SAMPLE_CYCLES 3
#endif

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif


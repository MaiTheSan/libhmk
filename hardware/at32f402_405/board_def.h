/*
 * This program is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include "common.h"

//--------------------------------------------------------------------+
// Flash Configuration
//--------------------------------------------------------------------+

//// Flash size in bytes
//#if defined (AT32F402_405xC)
//#define FLASH_SIZE                       (256 * 1024)   /* 256KB flash */
//#define FLASH_SECTOR_SIZE                2048           /* 2KB sectors */
//#else
//#define FLASH_SIZE                       (128 * 1024)   /* 128KB flash */
//#define FLASH_SECTOR_SIZE                1024           /* 1KB sectors */
//#endif

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

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
 * Created by Sawns on 3/1/2025.
 */

#include "hardware/hardware.h"
#include "board_def.h"
/* timer systick variable */
__IO uint32_t TickCount;

/**
  * @brief  initialize delay function
  * @param  none
  * @retval none
  */
void timer_init(void)
{
    /* configure systick */
    systick_clock_source_config(SYSTICK_CLOCK_SOURCE_AHBCLK_NODIV);
    /* Configure and initialize the SysTick for 1ms ticks */
    // SysTick->LOAD = (system_core_clock / 1000) - 1;
    SysTick->LOAD = (system_core_clock / 1000U) - 1;
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk;

    /* Reset counter */
    TickCount = 0;
}

/**
 * @brief  Get the current timer value in milliseconds
 * @param  None
 * @retval Current tick count in milliseconds
 */
uint32_t timer_read(void)
{
  return TickCount;
}


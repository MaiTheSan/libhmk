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

#include "crc32.h"
#include "eeconfig.h"
#include "hardware/hardware.h"
#include "tusb.h"
#include "wear_leveling.h"

int main(void) {
  board_init();

  crc32_init();
  flash_init();
  wear_leveling_init();
  eeconfig_init();

  analog_init();
  timer_init();

  tud_init(BOARD_TUD_RHPORT);

  while (1) {
    tud_task();

    analog_task();
  }

  return 0;
}

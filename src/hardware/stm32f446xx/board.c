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

#include "hardware/hardware.h"

#include "stm32f4xx_hal.h"
#include "tusb.h"

/**
 * @brief Initialize the clock
 *
 * @return None
 */
static void board_clock_init(void) {
  RCC_OscInitTypeDef rcc_osc_init = {0};
  RCC_ClkInitTypeDef rcc_clk_init = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  rcc_osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  rcc_osc_init.HSEState = RCC_HSE_ON;
  rcc_osc_init.PLL.PLLState = RCC_PLL_ON;
  rcc_osc_init.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  rcc_osc_init.PLL.PLLM = BOARD_HSE_VALUE / 1000000;
  rcc_osc_init.PLL.PLLN = 360;
  rcc_osc_init.PLL.PLLP = RCC_PLLP_DIV2;
  rcc_osc_init.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&rcc_osc_init) != HAL_OK)
    board_error_handler();

  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
    board_error_handler();

  rcc_clk_init.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                           RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  rcc_clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  rcc_clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;
  rcc_clk_init.APB1CLKDivider = RCC_HCLK_DIV4;
  rcc_clk_init.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&rcc_clk_init, FLASH_LATENCY_5) != HAL_OK)
    board_error_handler();
}

/**
 * @brief Initialize the USB
 *
 * @return None
 */
static void board_usb_init(void) {
  RCC_PeriphCLKInitTypeDef rcc_periph_clk_init = {0};
  GPIO_InitTypeDef gpio_init = {0};

  rcc_periph_clk_init.PeriphClockSelection = RCC_PERIPHCLK_CLK48;
  rcc_periph_clk_init.PLLSAI.PLLSAIM = 16;
  rcc_periph_clk_init.PLLSAI.PLLSAIN = 192;
  rcc_periph_clk_init.PLLSAI.PLLSAIQ = 2;
  rcc_periph_clk_init.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV4;
  rcc_periph_clk_init.PLLSAIDivQ = 1;
  rcc_periph_clk_init.Clk48ClockSelection = RCC_CLK48CLKSOURCE_PLLSAIP;
  if (HAL_RCCEx_PeriphCLKConfig(&rcc_periph_clk_init) != HAL_OK)
    board_error_handler();

#if defined(BOARD_USB_FS)
  __HAL_RCC_GPIOA_CLK_ENABLE();

  // USB D+/D-
  gpio_init.Pin = GPIO_PIN_11 | GPIO_PIN_12;
  gpio_init.Mode = GPIO_MODE_AF_PP;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio_init.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &gpio_init);

  __HAL_RCC_USB_OTG_FS_CLK_ENABLE();

  // Disable VBUS sensing
  USB_OTG_FS->GCCFG &= ~USB_OTG_GCCFG_VBDEN;
  USB_OTG_FS->GOTGCTL |= USB_OTG_GOTGCTL_BVALOEN;
  USB_OTG_FS->GOTGCTL |= USB_OTG_GOTGCTL_BVALOVAL;
#elif defined(BOARD_USB_HS)
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // USB D+/D-
  gpio_init.Pin = GPIO_PIN_14 | GPIO_PIN_15;
  gpio_init.Mode = GPIO_MODE_AF_PP;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio_init.Alternate = GPIO_AF12_OTG_HS_FS;
  HAL_GPIO_Init(GPIOB, &gpio_init);

  __HAL_RCC_USB_OTG_HS_CLK_ENABLE();

  // Disable VBUS sensing
  USB_OTG_HS->GCCFG &= ~USB_OTG_GCCFG_VBDEN;
  USB_OTG_HS->GOTGCTL |= USB_OTG_GOTGCTL_BVALOEN;
  USB_OTG_HS->GOTGCTL |= USB_OTG_GOTGCTL_BVALOVAL;
#else
#error "USB peripheral not defined"
#endif
}

void board_init(void) {
  HAL_Init();

  board_clock_init();
  board_usb_init();
}

void board_error_handler(void) {
  __disable_irq();
  while (1);
}

void board_reset(void) { NVIC_SystemReset(); }

uint32_t board_serial(uint16_t *buf) {
  static char serial_str[24 + 1];

  // Use the 96-bit unique ID as the serial number
  sprintf(serial_str, "%08" PRIX32 "%08" PRIX32 "%08" PRIX32, HAL_GetUIDw2(),
          HAL_GetUIDw1(), HAL_GetUIDw0());

  // Convert ASCII string into UTF-16-LE
  for (size_t i = 0; i < 24; i++)
    buf[i] = serial_str[i];

  return 24;
}

//--------------------------------------------------------------------+
// HAL Callbacks
//--------------------------------------------------------------------+

void SysTick_Handler(void) { HAL_IncTick(); }

void OTG_FS_IRQHandler(void) { tud_int_handler(0); }

void OTG_HS_IRQHandler(void) { tud_int_handler(1); }

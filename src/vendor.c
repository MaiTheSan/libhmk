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

#include "vendor.h"

#include "eeconfig.h"
#include "hardware/hardware.h"
#include "matrix.h"
#include "tusb.h"
#include "usb_descriptors.h"

#if defined(VENDOR_WEBUSB_URL)
#define WEBUSB_DESC_LEN (3 + sizeof(VENDOR_WEBUSB_URL) - 1)

// WebUSB descriptor with landing page URL
static const tusb_desc_webusb_url_t desc_url = {
    .bLength = WEBUSB_DESC_LEN,
    .bDescriptorType = TUSB_DESC_STRING,
    .bScheme = VENDOR_WEBUSB_URL_SCHEME,
    .url = VENDOR_WEBUSB_URL,
};
#endif

static uint8_t vendor_buffer[VENDOR_BUFFER_SIZE];

//--------------------------------------------------------------------+
// TinyUSB Callbacks
//--------------------------------------------------------------------+

bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage,
                                tusb_control_request_t const *request) {
  switch (request->bmRequestType_bit.type) {
  case TUSB_REQ_TYPE_VENDOR:
    switch (request->bRequest) {
#if defined(VENDOR_WEBUSB_URL)
    case VENDOR_REQUEST_WEBUSB:
      if (stage == CONTROL_STAGE_SETUP)
        return tud_control_xfer(rhport, request, (void *)&desc_url,
                                WEBUSB_DESC_LEN);
      // Nothing to do for other stages
      return true;
#endif

    case VENDOR_REQUEST_MICROSOFT:
      if (request->wIndex == 0x07 && stage == CONTROL_STAGE_SETUP)
        return tud_control_xfer(rhport, request, (void *)desc_ms_os_20,
                                MS_OS_20_DESC_LEN);
      // Nothing to do for other stages
      return true;

    default:
      break;
    }

  case TUSB_REQ_TYPE_CLASS:
    switch (request->bRequest) {
    case CLASS_REQUEST_FIRMWARE_VERSION:
      if (stage == CONTROL_STAGE_SETUP) {
        static const uint16_t res = FIRMWARE_VERSION;
        if (request->wLength < sizeof(res))
          // Request length is smaller than the response
          return false;
        return tud_control_xfer(rhport, request, (void *)&res, sizeof(res));
      }
      // Nothing to do for other stages
      return true;

    case CLASS_REQUEST_REBOOT:
      if (stage == CONTROL_STAGE_SETUP) {
        board_reset();
        return tud_control_status(rhport, request);
      }
      // Nothing to do for other stages
      return true;

    case CLASS_REQUEST_BOOTLOADER:
      // TODO: Implement bootloader request
      return true;

    case CLASS_REQUEST_FACTORY_RESET:
      if (stage == CONTROL_STAGE_SETUP) {
        if (!eeconfig_reset())
          // Failed to reset the configuration
          return false;
        return tud_control_status(rhport, request);
      }
      // Nothing to do for other stages
      return true;

    case CLASS_REQUEST_RECALIBRATE:
      if (stage == CONTROL_STAGE_SETUP) {
        matrix_recalibrate();
        return tud_control_status(rhport, request);
      }
      // Nothing to do for other stages
      return true;

    case CLASS_REQUEST_DEBUG:
      if (stage == CONTROL_STAGE_SETUP) {
        static class_res_debug_t res[NUM_KEYS];
        if (request->wLength < sizeof(res))
          // Request length is smaller than the response
          return false;
        for (uint32_t i = 0; i < NUM_KEYS; i++) {
          res[i].adc_value = key_matrix[i].adc_filtered;
          res[i].distance = key_matrix[i].distance;
        }
        return tud_control_xfer(rhport, request, (void *)res, sizeof(res));
      }
      // Nothing to do for other stages
      return true;

    case CLASS_REQUEST_GET_PROFILE:
      if (stage == CONTROL_STAGE_SETUP) {
        static uint8_t res;
        if (request->wLength < sizeof(res))
          // Request length is smaller than the response
          return false;
        res = eeconfig->current_profile;
        return tud_control_xfer(rhport, request, (void *)&res, sizeof(res));
      }
      // Nothing to do for other stages
      return true;

    case CLASS_REQUEST_GET_KEYMAP:
      if (stage == CONTROL_STAGE_SETUP) {
        if (request->wValue >= NUM_PROFILES ||
            request->wLength <
                sizeof(eeconfig->profiles[request->wValue].keymap))
          // Invalid profile number or request length
          return false;
        return tud_control_xfer(
            rhport, request, (void *)eeconfig->profiles[request->wValue].keymap,
            sizeof(eeconfig->profiles[request->wValue].keymap));
      }
      // Nothing to do for other stages
      return true;

    case CLASS_REQUEST_SET_KEYMAP:
      if (stage == CONTROL_STAGE_SETUP) {
        if (request->wValue >= NUM_PROFILES ||
            request->wLength !=
                sizeof(eeconfig->profiles[request->wValue].keymap) ||
            request->wLength > VENDOR_BUFFER_SIZE)
          // Invalid profile number or request length
          return false;
        return tud_control_xfer(rhport, request, vendor_buffer,
                                request->wLength);
      } else if (stage == CONTROL_STAGE_DATA) {
        if (!eeconfig_set_keymap(request->wValue, vendor_buffer))
          // Failed to set the keymap
          return false;
        return tud_control_status(rhport, request);
      }
      // Nothing to do for other stages
      return true;

    case CLASS_REQUEST_GET_ACTUATION_MAP:
      if (stage == CONTROL_STAGE_SETUP) {
        if (request->wValue >= NUM_PROFILES ||
            request->wLength <
                sizeof(eeconfig->profiles[request->wValue].actuation_map))
          // Invalid profile number or request length
          return false;
        return tud_control_xfer(
            rhport, request,
            (void *)eeconfig->profiles[request->wValue].actuation_map,
            sizeof(eeconfig->profiles[request->wValue].actuation_map));
      }
      // Nothing to do for other stages
      return true;

    case CLASS_REQUEST_SET_ACTUATION_MAP:
      if (stage == CONTROL_STAGE_SETUP) {
        if (request->wValue >= NUM_PROFILES ||
            request->wLength !=
                sizeof(eeconfig->profiles[request->wValue].actuation_map) ||
            request->wLength > VENDOR_BUFFER_SIZE)
          // Invalid profile number or request length
          return false;
        return tud_control_xfer(rhport, request, vendor_buffer,
                                request->wLength);
      } else if (stage == CONTROL_STAGE_DATA) {
        if (!eeconfig_set_actuation_map(request->wValue, vendor_buffer))
          // Failed to set the actuation map
          return false;
        return tud_control_status(rhport, request);
      }
      // Nothing to do for other stages
      return true;

    case CLASS_REQUEST_GET_ADVANCED_KEYS:
      if (stage == CONTROL_STAGE_SETUP) {
        if (request->wValue >= NUM_PROFILES ||
            request->wLength <
                sizeof(eeconfig->profiles[request->wValue].advanced_keys))
          // Invalid profile number or request length
          return false;
        return tud_control_xfer(
            rhport, request,
            (void *)eeconfig->profiles[request->wValue].advanced_keys,
            sizeof(eeconfig->profiles[request->wValue].advanced_keys));
      }
      // Nothing to do for other stages
      return true;

    case CLASS_REQUEST_SET_ADVANCED_KEYS:
      if (stage == CONTROL_STAGE_SETUP) {
        if (request->wValue >= NUM_PROFILES ||
            request->wLength !=
                sizeof(eeconfig->profiles[request->wValue].advanced_keys) ||
            request->wLength > VENDOR_BUFFER_SIZE)
          // Invalid profile number or request length
          return false;
        return tud_control_xfer(rhport, request, vendor_buffer,
                                request->wLength);
      } else if (stage == CONTROL_STAGE_DATA) {
        if (!eeconfig_set_advanced_keys(request->wValue, vendor_buffer))
          // Failed to set the advanced keys
          return false;
        return tud_control_status(rhport, request);
      }
      // Nothing to do for other stages
      return true;

    default:
      break;
    }
    break;

  default:
    break;
  }

  // Stall unknown request
  return false;
}

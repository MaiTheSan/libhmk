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
#include "at32f402_405_conf.h"

#if defined (AT32F402_405xC)
#define FLASH_SECTOR_SIZE                2048   /* 2KB sectors for AT32F402_405xC */
#else
#define FLASH_SECTOR_SIZE                1024   /* 1KB sectors for other AT32F402_405 */
#endif

/* Define total flash size */
#if defined (AT32F402_405xC)
#define FLASH_SIZE                       (256 * 1024)   /* 256KB flash for AT32F402_405xC */
#else
#define FLASH_SIZE                       (128 * 1024)   /* 128KB flash for other AT32F402_405 */
#endif

/* Calculate total number of sectors */
#define FLASH_NUM_SECTORS                (FLASH_SIZE / FLASH_SECTOR_SIZE)

/* Static array to hold sector sizes for consistent API with STM32 implementation */
static const uint32_t flash_sector_sizes[FLASH_NUM_SECTORS];

void flash_init(void) {
    /* Initialize the sector sizes array */
    for (int i = 0; i < FLASH_NUM_SECTORS; i++) {
        /* All sectors are the same size in AT32F402_405 */
        ((uint32_t *)flash_sector_sizes)[i] = FLASH_SECTOR_SIZE;
    }
}

uint32_t flash_sector_size(uint32_t sector) {
    return sector < FLASH_NUM_SECTORS ? FLASH_SECTOR_SIZE : 0;
}

bool flash_erase(uint32_t sector) {
    if (sector >= FLASH_NUM_SECTORS)
        return false;

    flash_status_type status;
    uint32_t sector_address = FLASH_BASE + (sector * FLASH_SECTOR_SIZE);

    /* Unlock flash */
    flash_unlock();

    /* Erase the sector */
    status = flash_sector_erase(sector_address);

    /* Lock flash */
    flash_lock();

    return (status == FLASH_OPERATE_DONE) ? true : false;
}

bool flash_read(uint32_t addr, void *buf, uint32_t len) {
    if (addr + len * 4 > FLASH_SIZE)
        return false;

    /* For AT32, len is in 32-bit words (same as STM32 implementation) */
    memcpy(buf, (void *)(FLASH_BASE + addr), len * 4);

    return true;
}

bool flash_write(uint32_t addr, const void *buf, uint32_t len) {
    if (addr + len * 4 > FLASH_SIZE)
        return false;

    const uint32_t *buf32 = buf;
    bool success = true;
    flash_status_type status;

    /* Unlock flash */
    flash_unlock();

    /* Write data word by word */
    for (uint32_t i = 0; i < len; i++) {
        status = flash_word_program(FLASH_BASE + addr + i * 4, buf32[i]);
        if (status != FLASH_OPERATE_DONE) {
            success = false;
            break;
        }
    }

    /* Lock flash */
    flash_lock();

    return success;
}

/* Alternative implementation for byte/halfword access if needed */
//bool flash_write_halfword(uint32_t addr, const uint16_t *buf, uint32_t len) {
//    if (addr + len * 2 > FLASH_SIZE)
//        return false;
//
//    bool success = true;
//    flash_status_type status;
//
//    /* Unlock flash */
//    flash_unlock();
//
//    /* Write data halfword by halfword */
//    for (uint32_t i = 0; i < len; i++) {
//        status = flash_halfword_program(FLASH_BASE + addr + i * 2, buf[i]);
//        if (status != FLASH_OPERATE_DONE) {
//            success = false;
//            break;
//        }
//    }
//
//    /* Lock flash */
//    flash_lock();
//
//    return success;
//}

bool flash_write_byte(uint32_t addr, const uint8_t *buf, uint32_t len) {
    if (addr + len > FLASH_SIZE)
        return false;

    bool success = true;
    flash_status_type status;

    /* Unlock flash */
    flash_unlock();

    /* Write data byte by byte */
    for (uint32_t i = 0; i < len; i++) {
        status = flash_byte_program(FLASH_BASE + addr + i, buf[i]);
        if (status != FLASH_OPERATE_DONE) {
            success = false;
            break;
        }
    }

    /* Lock flash */
    flash_lock();

    return success;
}
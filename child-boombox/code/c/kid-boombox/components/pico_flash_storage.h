#ifndef PICO_FLASH_STORAGE_H
#define PICO_FLASH_STORAGE_H

#include "pico/stdio.h"

namespace storage
{
    typedef struct {
        uint16_t value;
        uint32_t checksum;
    } uint16_checked_data_t;

    typedef struct {
        uint16_checked_data_t volume;
    } saved_data_t;

    void save_volume_to_flash(uint16_t val);
    uint16_t read_saved_volume(void);
}

#endif // PICO_FLASH_STORAGE_H
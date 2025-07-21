#include <string.h>
#include <stdio.h>
#include "pico_flash_storage.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "hardware/regs/addressmap.h"

extern char __flash_binary_end; // provided by linker

namespace storage {
    class FlashStorage {
        private:
            const uintptr_t __flash_binary_end_addr = (uintptr_t) &__flash_binary_end;
            const uintptr_t ALIGNED_FLASH_BINARY_END_ADDR = align_addr_to_sector(__flash_binary_end_addr);
            saved_data_t *const data = (saved_data_t*) ALIGNED_FLASH_BINARY_END_ADDR;
            const uintptr_t ALIGNED_FLASH_OFFSET = ALIGNED_FLASH_BINARY_END_ADDR - XIP_BASE;

            // Helper to align address upwards to the next sector boundary
            static uintptr_t align_addr_to_sector(uintptr_t addr) {
                uintptr_t offset = addr % FLASH_SECTOR_SIZE;
                return (offset == 0) ? addr : addr + (FLASH_SECTOR_SIZE - offset);
            }

            bool is_valid_flash_address(uintptr_t address, size_t required_size) {
                return address >= XIP_BASE &&
                    address + required_size <= (XIP_BASE + PICO_FLASH_SIZE_BYTES);
            }

        public:
            FlashStorage() {
                hard_assert(is_valid_flash_address(ALIGNED_FLASH_BINARY_END_ADDR, FLASH_SECTOR_SIZE));
                printf("FLASH variables: %08x %08x %08x\n", __flash_binary_end_addr, ALIGNED_FLASH_BINARY_END_ADDR, ALIGNED_FLASH_OFFSET);
            }

            static uint32_t calculate_checksum(uint16_t value) {
                return (uint32_t)value ^ 0xDEADBEEF;
            }

            // dont call often - pico has about 100,000 write cycles
            void save_data_to_flash(saved_data_t data_to_save) {
                hard_assert(sizeof(data_to_save) < FLASH_PAGE_SIZE);
                uint8_t page_buf[FLASH_PAGE_SIZE];
                memset(page_buf, 0xFF, FLASH_PAGE_SIZE);
                memcpy(page_buf, &data_to_save, sizeof(data_to_save));

                uint32_t status = save_and_disable_interrupts();
                flash_range_erase(ALIGNED_FLASH_OFFSET, FLASH_SECTOR_SIZE);
                flash_range_program(ALIGNED_FLASH_OFFSET, page_buf, FLASH_PAGE_SIZE);
                restore_interrupts(status);
                printf("FLASH data saved 0x%04x %d\n", data_to_save.volume.value, data_to_save.volume.checksum);
            }

            saved_data_t* read_saved_data() {
                return data;  // Read the value — basically reading from flash directly
            }
    };
    
    static FlashStorage flash_storage = FlashStorage();

    void save_volume_to_flash(uint16_t val) {
        saved_data_t data_to_save = { { val, FlashStorage::calculate_checksum(val) } };
        flash_storage.save_data_to_flash(data_to_save);
    }

   
    uint16_t read_saved_volume(void) {
        saved_data_t *data = flash_storage.read_saved_data();
        printf("0x%04x %d\n", data->volume.value, data->volume.checksum);
        if (data->volume.checksum == FlashStorage::calculate_checksum(data->volume.value)) {
            return data->volume.value;
        }
        printf("Warning: cannot load saved volume\n");
        return 0; // Or handle error case as appropriate
    }

}


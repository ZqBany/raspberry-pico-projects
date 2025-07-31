#include <vector>
#include "ff.h"
#include "sd_handler.h"
#include "sd_card.h"
#include "SPI/sd_card_spi.h"
#include "f_util.h"
#include "hw_config.h"
#include "my_rtc.h"

#define R1_IDLE_STATE 1 << 0 // sc_card_spi.c -> spi_r1_response_t

namespace sd_handler
{
    struct sd_file_t { FIL fil; };
    static volatile bool initialized = false;

    class SDHandler {
        private:
            static std::vector<spi_t *> spis;             // SPI H/W components
            static std::vector<sd_spi_if_t *> spi_ifs;    // SPI Interfaces
            static spi_t *p_spi;
            static FATFS fs;

        public:
            static std::vector<sd_card_t *> sd_cards;     // SD Card Sockets

            SDHandler() {}

            int sd_card_init() {
                time_init();
                sleep_ms(25);

                spi_t *spi_p = new spi_t();
                assert(spi_p);
                spi_p->hw_inst = spi0;  // RP2040 SPI component
                spi_p->sck_gpio = SD_SCK_PIN;    // GPIO number (not Pico pin number)
                spi_p->mosi_gpio = SD_MOSI_PIN;
                spi_p->miso_gpio = SD_MISO_PIN;
                spi_p->set_drive_strength = true;
                spi_p->mosi_gpio_drive_strength = GPIO_DRIVE_STRENGTH_4MA;
                spi_p->sck_gpio_drive_strength = GPIO_DRIVE_STRENGTH_2MA;
                spi_p->baud_rate = 12 * 1000 * 1000;  // Actual frequency: 10416666
                spis.push_back(spi_p);

                sd_spi_if_t *spi_if_p = new sd_spi_if_t();
                assert(spi_if_p);

                spi_if_p->spi = spi_p;  // Pointer to the SPI driving this card
                spi_if_p->ss_gpio = SD_CS_PIN;    // The SPI slave select GPIO for this SD card
                spi_ifs.push_back(spi_if_p);

                sd_card_t *sd_card_p = new sd_card_t();
                assert(sd_card_p);
                sd_card_p->type = SD_IF_SPI;
                sd_card_p->spi_if_p = spi_if_p;  // Pointer to the SPI interface driving this card
                sd_card_p->use_card_detect = false;
                sd_card_p->card_detect_gpio = 0;
                sd_card_p->card_detected_true = -1;  // What the GPIO read returns when a card is present
                sd_card_p->card_detect_use_pull = false;
                sd_card_p->card_detect_pull_hi = false;
                sd_cards.push_back(sd_card_p);

                printf("Init SD card driver\n");
                sd_init_driver();

                sd_card_t *pSD = sd_get_by_num(0);
                char const * const drive_prefix = sd_get_drive_prefix(pSD);
                printf("Trying to mount drive [%s]\n", drive_prefix);
                FRESULT fr = f_mount(&pSD->state.fatfs, drive_prefix, 1);
                if (fr != FR_OK) {
                    printf("Error: mounting SD card drive failed %s (%d)\n", FRESULT_str(fr), fr);
                    return false;
                }
                fr = f_chdrive(drive_prefix);
                if (fr != FR_OK) {
                    printf("Error: cannot change SD card drive %s (%d)\n", FRESULT_str(fr), fr);
                    return false;
                }

                if (FF_FS_READONLY != 0 || FF_FS_LOCK == 0) {
                    printf("Error: configuration must allow read/write and define file locks\n");
                    return false;
                }

                printf("SD card mounted successfully\n");
                return PICO_OK;
            }

            void sd_card_deinit() {
                sd_card_t *pSD = sd_get_by_num(0);
                char const * const drive_prefix = sd_get_drive_prefix(pSD);
                FRESULT fr = f_unmount(drive_prefix);
                if (fr != FR_OK) {
                    printf("Error: unmounting SD card drive failed %s (%d)\n", FRESULT_str(fr), fr);
                }
                pSD->state.mounted = false;
                pSD->state.m_Status |= STA_NOINIT;
                uint32_t result = sd_go_idle_state(pSD);
                if (R1_IDLE_STATE == result) {
                    printf("SD card put to idle successfully: [%s]\n", drive_prefix);
                }
                sleep_ms(10);
            }

            bool file_exists(char *filename) {
                FRESULT fr;
                FILINFO fno;
                fr = f_stat(filename, &fno);
                if (fr == FR_OK) {
                    return true;
                }
                return false;
            }

            bool open_file_for_read(char *filename, sd_file_t*& file) {
                if (file != nullptr) {
                    printf("ERROR: passed file must be nullptr\n");
                    return false;
                }
                file = new sd_file_t;
                if (!file) {
                    return false;
                }
                FRESULT fr;
                fr = f_open(&file->fil, filename, FA_READ | FA_OPEN_EXISTING);
                if (fr != FR_OK) {
                    printf("Error: cannot open file for reading %s:  %s (%d)\n", filename, FRESULT_str(fr), fr);
                    delete file;
                    file = nullptr;
                    return false;
                }
                return true;
            }

            bool seek_file_position(FIL* file, FSIZE_t file_pointer) {
                FRESULT fr = f_lseek(file, file_pointer);
                if (fr != FR_OK) {
                    printf("Error: cannot seek to file start: %s (%d)\n", FRESULT_str(fr), fr);
                    return false;
                }
                return true;
            }

            UINT read_bytes(FIL* file, void *buffer, UINT bytes_to_read) {
                FRESULT fr;
                UINT bytes_read;
                fr = f_read(file,
                            buffer,
                            bytes_to_read,
                            &bytes_read);

                if (fr != FR_OK) {
                    printf("Error: while reading file: %s (%d)\n", FRESULT_str(fr), fr);
                    return 0;
                }

                return bytes_read;
            }

            UINT fill_buffer(FIL* file, int16_t *buffer, UINT bytes_to_read) {
                return read_bytes(file, buffer, bytes_to_read);
            }

            

            bool close_file(FIL* file) {
                FRESULT fr;
                UINT bytes_read;
                fr = f_close(file);
                return fr == FR_OK;
            }
    };

    std::vector<spi_t *> SDHandler::spis = {};
    std::vector<sd_spi_if_t *> SDHandler::spi_ifs = {};
    std::vector<sd_card_t *> SDHandler::sd_cards = {};

    static SDHandler sd_handler = SDHandler();

    void initialize_module() {
        if (!initialized) {
            initialized = true;
            sleep_ms(25);

            sd_handler.sd_card_init();

            sleep_ms(5); // w8t for stabilization
        }
    }

    void deinitialize_module() {
        if (initialized) {
            sd_handler.sd_card_deinit();
            initialized = false;
            sleep_ms(5);
        }
    }

    bool file_exists(char *filename) {
        return sd_handler.file_exists(filename);
    }

    bool open_file_for_read(char *filename, sd_file_t*& file) {
        return sd_handler.open_file_for_read(filename, file);
    }

    size_t current_file_position(sd_file_t* file) {
        return f_tell(&file->fil);
    }

    bool seek_file_position(sd_file_t* file, size_t file_pointer) {
        return sd_handler.seek_file_position(&file->fil, file_pointer);
    }

    size_t fill_buffer(sd_file_t* file, int16_t *buffer, size_t bytes_to_read) {
        return sd_handler.fill_buffer(&file->fil, buffer, bytes_to_read);
    }

    size_t read_bytes(sd_file_t* file, void* buffer, size_t bytes_to_read) {
        return sd_handler.read_bytes(&file->fil, buffer, bytes_to_read);
    }
    
    bool close_file(sd_file_t*& file) {
        if (file != nullptr) {
            bool success = sd_handler.close_file(&file->fil);
            delete file;
            file = nullptr;
            return success;
        }
        return false;
    }
}

size_t sd_get_num() { return sd_handler::SDHandler::sd_cards.size(); }

sd_card_t *sd_get_by_num(size_t num) {
    if (num <= sd_get_num()) {
        return sd_handler::SDHandler::sd_cards[num];
    } else {
        return NULL;
    }
}
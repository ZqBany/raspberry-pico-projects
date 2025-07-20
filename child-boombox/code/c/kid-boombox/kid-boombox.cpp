#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/audio_i2s.h"
#include "barcode/scanner.h"
#include "sd_card.h"
#include "SPI/sd_card_spi.h"
#include "ff.h"
#include "f_util.h"
#include "hw_config.h"
#include "hardware/pio.h"
#include "my_rtc.h"
#include "pico/sleep.h"
#include "tusb.h"
#include "pico/stdio_usb.h"
#include "hardware/watchdog.h"
#include "hardware/adc.h"
#include "hardware/flash.h"

#define START_PLAYING_MSG 84
#define STOP_PLAYING_MSG 85
#define CHANGE_VOLUME_MSG 86

#define SD_SPI_PORT spi0
#define SD_CS_PIN 17
#define SD_SCK_PIN 18
#define SD_MOSI_PIN 19
#define SD_MISO_PIN 16

#define WAKE_GPIO 20
#define REPLAY_GPIO 22
#define VOL_GPIO 15

#define LED_GREEN_GPIO 14
#define LED_YELLOW_GPIO 13

#define SLEEP_TIMEOUT_US 60'000'000 // 1 minute

#define STATUS_LED_MS 100

// Struktura nagłówka WAV
typedef struct {
    // RIFF section
    char riff_header[4];
    uint32_t wav_size;
    char wave_header[4];

    // Format section
    char fmt_header[4];
    uint32_t fmt_chunk_size;
    uint16_t audio_format;
    uint16_t num_channels;
    uint32_t sample_rate;
    uint32_t byte_rate;
    uint16_t sample_alignment;
    uint16_t bit_depth;

    // 3rd section - data or LIST
    char data_header[4];
    uint32_t data_size;
} wav_header_t;

typedef struct {
    char section_header[4];
    uint32_t section_size;
} wav_section_t;

int init_led(void) {
    // A device like Pico that uses a GPIO for the LED will define PICO_DEFAULT_LED_PIN
    // so we can use normal GPIO functionality to turn the led on and off
    // gpio_init(PICO_DEFAULT_LED_PIN);
    // gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    gpio_init(LED_GREEN_GPIO);
    gpio_set_dir(LED_GREEN_GPIO, GPIO_OUT);

    gpio_init(LED_YELLOW_GPIO);
    gpio_set_dir(LED_YELLOW_GPIO, GPIO_OUT);
    return PICO_OK;
}

struct BlinkCounter {
    int count;
};

static bool blink_timer_running = false;

bool turn_off_status_led(repeating_timer_t *rt) {
    struct BlinkCounter *blink_counter = (struct BlinkCounter *) rt->user_data;
    if (blink_counter->count == 0) {
        gpio_put(LED_YELLOW_GPIO, 0);  // turn LED off finally
        blink_timer_running = false;
        return false;  // cancel repeating timer
    }

    // Toggle LED, decrement count on every second call
    static bool led_on = false;
    led_on = !led_on;
    gpio_put(LED_YELLOW_GPIO, led_on ? 1 : 0);

    // Count toggles as "half blink", adjust if needed
    if (!led_on) {
        blink_counter->count--;
    }
    return true;  // keep timer repeating
}

void blink_status_led(uint8_t times) {
    static repeating_timer_t timer;
    static struct BlinkCounter counter = { 0 };

    counter.count += times;

    if (!blink_timer_running) {
        blink_timer_running = true;
        gpio_put(LED_YELLOW_GPIO, 1);     // Turn LED on
        add_repeating_timer_ms(-STATUS_LED_MS, turn_off_status_led, &counter, &timer); // set timer
    }
}

int init_buttons(void) {
    gpio_init(WAKE_GPIO);
    gpio_set_dir(WAKE_GPIO, GPIO_IN);
    gpio_pull_down(WAKE_GPIO);

    gpio_init(REPLAY_GPIO);
    gpio_set_dir(REPLAY_GPIO, GPIO_IN);
    gpio_pull_down(REPLAY_GPIO);

    gpio_init(VOL_GPIO);
    gpio_set_dir(VOL_GPIO, GPIO_IN);
    gpio_pull_down(VOL_GPIO);
    return PICO_OK;
}

void add_spi(spi_t *const spi);
void add_spi_if(sd_spi_if_t *const spi_if);
void add_sd_card(sd_card_t *const sd_card);

static spi_t *p_spi;
static FATFS fs;
static wav_header_t wav_header;

enum AUDIO {
        INSERT_CARD = -1 ,
        SUCCESS = -2,
        HELLO = -3,
        BYE = -4
};

enum AUDIO_VOLUME {
        VOL_100 = 0xFFFF,
        VOL_80 = 0xCCCC,
        VOL_60 = 0x9999,
        VOL_40 = 0x6666,
        VOL_20 = 0x3333
};

const AUDIO_VOLUME VOLUME_VALUES[] = {VOL_100, VOL_80, VOL_60, VOL_40, VOL_20};
const size_t volume_values_length = sizeof(VOLUME_VALUES) / sizeof(VOLUME_VALUES[0]);
bool INCREASE_VOLUME_FLAG = true;

AUDIO_VOLUME VOL = VOL_100;
AUDIO_VOLUME SAVED_VOL = VOL_100;

AUDIO_VOLUME cycle_volume() {
    AUDIO_VOLUME previous = VOL;
    uint8_t blink_times = 1;
    
    for(int i = 0; i < volume_values_length; i++) {
        if (previous == VOLUME_VALUES[i]) {
            if ((i == 0 && INCREASE_VOLUME_FLAG) || (i + 1 == volume_values_length && !INCREASE_VOLUME_FLAG)) {
                INCREASE_VOLUME_FLAG = !INCREASE_VOLUME_FLAG;
            }
            int new_value_index = INCREASE_VOLUME_FLAG ? i - 1: i + 1;
            if (new_value_index == 0 || new_value_index == volume_values_length - 1) {
                blink_times = 2; 
            }
            VOL = VOLUME_VALUES[new_value_index];
            break;
        }
    }
    blink_status_led(blink_times);
    return previous;
}

struct uint16_checked_data {
    uint16_t value;
    uint32_t checksum;
};

// Helper to align address upwards to the next sector boundary
static uintptr_t align_addr_to_sector(uintptr_t addr) {
    uintptr_t offset = addr % FLASH_SECTOR_SIZE;
    return (offset == 0) ? addr : addr + (FLASH_SECTOR_SIZE - offset);
}

bool is_valid_flash_address(uintptr_t address, size_t required_size) {
    return address >= XIP_BASE &&
           address + required_size <= (XIP_BASE + PICO_FLASH_SIZE_BYTES);
}

extern char __flash_binary_end; // provided by linker
const uintptr_t __flash_binary_end_addr = (uintptr_t) &__flash_binary_end;
const uintptr_t ALIGNED_FLASH_BINARY_END_ADDR = align_addr_to_sector(__flash_binary_end_addr);
const struct uint16_checked_data* data = (const struct uint16_checked_data*) ALIGNED_FLASH_BINARY_END_ADDR;
const uintptr_t ALIGNED_FLASH_OFFSET = ALIGNED_FLASH_BINARY_END_ADDR - XIP_BASE;

uint32_t calculate_checksum(uint16_t value) {
    return (uint32_t)value ^ 0xDEADBEEF;
}

void save_volume_to_flash(uint16_t val) {
    struct uint16_checked_data data = { val, calculate_checksum(val) };
    uint8_t page_buf[FLASH_PAGE_SIZE];
    memset(page_buf, 0xFF, FLASH_PAGE_SIZE);
    memcpy(page_buf, &data, sizeof(data));

    uint32_t status = save_and_disable_interrupts();
    flash_range_erase(ALIGNED_FLASH_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(ALIGNED_FLASH_OFFSET, page_buf, FLASH_PAGE_SIZE);
    restore_interrupts(status);
}

// Read the value — basically reading from flash directly
AUDIO_VOLUME read_saved_volume(void) {
    printf("%d %d\n", data->value, data->checksum);
    if (data->checksum == calculate_checksum(data->value))
        return static_cast<AUDIO_VOLUME>(data->value);
    return VOL_100; // Or handle error case as appropriate
}

// Odczyt nagłówka pliku WAV
bool read_wav_header(FIL* file, wav_header_t* header) {
    UINT bytes_read;
    FRESULT fr = f_read(file, header, sizeof(wav_header_t), &bytes_read);
    
    if (fr != FR_OK || bytes_read != sizeof(wav_header_t)) {
        printf("Błąd odczytu nagłówka WAV\n");
        return false;
    }

    if (memcmp(header->riff_header, "RIFF", 4) != 0) {
        printf("Nieprawidłowy format %.*s - not RIFF\n", 4, header->riff_header);
        return false;
    }

    if (memcmp(header->wave_header, "WAVE", 4) != 0) {
        printf("Nieprawidłowy format - not WAVE file\n");
        return false;
    }

    if (memcmp(header->fmt_header, "fmt", 3) != 0) {
        printf("Nieprawidłowy format - no format section found\n");
        return false;
    }

    if (memcmp(header->data_header, "data", 4) != 0) {
        if (memcmp(header->data_header, "LIST", 4) == 0) {
            wav_section_t next_section_header;
            fr = f_lseek(file, f_tell(file) + header->data_size);
            if (fr != FR_OK) {
                printf("Nieprawidłowy format - cannot seek after LIST section\n");
                return false;
            }
            fr = f_read(file, &next_section_header, sizeof(wav_section_t), &bytes_read);
            if (fr != FR_OK || bytes_read != sizeof(wav_section_t)) {
                printf("Błąd odczytu sekcji data nagłówka\n");
                return false;
            }
            if (memcmp(next_section_header.section_header, "data", 4) == 0) {
                memcpy(header->data_header, next_section_header.section_header, 4);
                header->data_size = next_section_header.section_size;
            }  
        }
    }

    if (memcmp(header->data_header, "data", 4) != 0) {
        printf("Nieprawidłowy format %.*s - no data section found\n", 4, header->data_header);
        return false;
    }
    
    if (header->audio_format != 1) {
        printf("Obsługiwany jest tylko format PCM\n");
        return false;
    }

    if(header->fmt_chunk_size != 16) 
    {
        printf("Nieprawidłowy format - format section size must be 16.");
        return false;                          
    }

    if((header->num_channels != 2)) // && (header->num_channels != 1)
    {
        printf("Nieprawidłowy format - only stereo permitted.");
        return false;   
    }
    if(header->sample_rate != 44100) 
    {
        printf("Nieprawidłowy format - sample rate cannot must be 44100");
        return false;                       
    }
    if((header->bit_depth != 16)) // && (header->bit_depth != 8)
    {
        printf("Nieprawidłowy format - only 16 bits per sample permitted.");
        return false;                        
    }
    
    printf("Parametry pliku WAV:\n");
    printf("- Częstotliwość próbkowania: %lu Hz\n", header->sample_rate);
    printf("- Liczba kanałów: %u\n", header->num_channels);
    printf("- Głębokość bitowa: %u\n", header->bit_depth);
    printf("- Rozmiar danych: %lu bajtów\n", header->data_size);
    
    return true;
}

#define SAMPLES_PER_BUFFER 512
#define AUDIO_I2S_DMA_CHANNEL 0
#define AUDIO_SHUTDOWN_PIN 21 // Connected via 470kOhm resistor to MAX98357A2 SD pin

static audio_format_t audio_format = {
            .sample_freq = 44100,
            .format = AUDIO_BUFFER_FORMAT_PCM_S16,
            .channel_count = 2,
    };

struct audio_buffer_pool *init_audio() {
    printf("Zmienne %d %d %d %d\n", PICO_AUDIO_I2S_DATA_PIN, PICO_AUDIO_I2S_CLOCK_PIN_BASE, PICO_AUDIO_I2S_CLOCK_PINS_SWAPPED, AUDIO_SHUTDOWN_PIN);

    gpio_init(AUDIO_SHUTDOWN_PIN);
    gpio_set_dir(AUDIO_SHUTDOWN_PIN, GPIO_OUT);
    gpio_put(AUDIO_SHUTDOWN_PIN, true);

    static struct audio_buffer_format producer_format = {
            .format = &audio_format,
            .sample_stride = 4 // bytes per audio frame - 16-bit stereo = 2 channels × 2 bytes
    };

    struct audio_buffer_pool *producer_pool = audio_new_producer_pool(&producer_format, 3,
                                                                      SAMPLES_PER_BUFFER); // todo correct size
    bool __unused ok;
    const struct audio_format *output_format;
    if (dma_channel_is_claimed(AUDIO_I2S_DMA_CHANNEL)) {
        panic("PicoAudio: Change DMA channel.\n");
    }
    
    struct audio_i2s_config config = {
            .data_pin = PICO_AUDIO_I2S_DATA_PIN, 
            .clock_pin_base = PICO_AUDIO_I2S_CLOCK_PIN_BASE,
            .dma_channel = AUDIO_I2S_DMA_CHANNEL,
            .pio_sm = 0,
    };
    
    output_format = audio_i2s_setup(&audio_format, &config);
    if (!output_format) {
        panic("PicoAudio: Unable to open audio device.\n");
    }

    ok = audio_i2s_connect(producer_pool);
    if (!ok) {
        panic("PicoAudio: Unable to connect to producer pool.\n");
    }

    printf("Audio I2S zainicjalizowane pomyślnie\n");
    return producer_pool;
}

void deinit_audio() {
    audio_i2s_set_enabled(false);
    gpio_put(AUDIO_SHUTDOWN_PIN, false);
}

int sd_card_init() {
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
    add_spi(spi_p);

    sd_spi_if_t *spi_if_p = new sd_spi_if_t();
    assert(spi_if_p);

    spi_if_p->spi = spi_p;  // Pointer to the SPI driving this card
    spi_if_p->ss_gpio = SD_CS_PIN;    // The SPI slave select GPIO for this SD card
    add_spi_if(spi_if_p);

    sd_card_t *sd_card_p = new sd_card_t();
    assert(sd_card_p);
    sd_card_p->type = SD_IF_SPI;
    sd_card_p->spi_if_p = spi_if_p;  // Pointer to the SPI interface driving this card
    sd_card_p->use_card_detect = false;
    sd_card_p->card_detect_gpio = 0;
    sd_card_p->card_detected_true = -1;  // What the GPIO read returns when a card is present
    sd_card_p->card_detect_use_pull = false;
    sd_card_p->card_detect_pull_hi = false;
    add_sd_card(sd_card_p);

    sd_init_driver();

    sd_card_t *pSD = sd_get_by_num(0);
    char const * const drive_prefix = sd_get_drive_prefix(pSD);
    FRESULT fr = f_mount(&pSD->state.fatfs, drive_prefix, 1);
    if (fr != FR_OK) {
        printf("Błąd montowania karty SD: %s (%d)\n", FRESULT_str(fr), fr);
        return false;
    }
    fr = f_chdrive(drive_prefix);
    if (fr != FR_OK) {
        printf("Błąd zmiany drive: %s (%d)\n", FRESULT_str(fr), fr);
        return false;
    }

    printf("Karta SD zamontowana pomyślnie\n");
    return PICO_OK;
}

#define R1_IDLE_STATE 1 << 0 // sc_card_spi.c -> spi_r1_response_t

void sd_card_deinit() {
    sd_card_t *pSD = sd_get_by_num(0);
    char const * const drive_prefix = sd_get_drive_prefix(pSD);
    FRESULT fr = f_unmount(drive_prefix);
    if (fr != FR_OK) {
        printf("Błąd odmontowania karty SD: %s (%d)\n", FRESULT_str(fr), fr);
    }
    pSD->state.mounted = false;
    pSD->state.m_Status |= STA_NOINIT;
    uint32_t result = sd_go_idle_state(pSD);
    if (R1_IDLE_STATE == result) {
        printf("Karta uśpiona pomyślnie: [%s]\n", drive_prefix);
    }
    sleep_ms(10);
}

void power_led_put(bool led_on) {
    //gpio_put(PICO_DEFAULT_LED_PIN, led_on);
    gpio_put(LED_GREEN_GPIO, led_on);
}

struct Core1State {
    volatile bool TERMINATE = false;
    volatile bool READY = false;
    volatile bool FULL_IDLE = false;
    volatile bool CORE_0_REQUEST_DORMANT = false;
    volatile bool DORMANT_READY = false;
    volatile int REQUESTED_FILE_ID = 0;
};

static Core1State CORE_1_STATE;
static struct audio_buffer_pool *ap;

enum Core1Job {
    IDLE, PLAYING, GATHERING
};

bool open_audio_file(int *file_id, char *filename, FIL *file, FSIZE_t *audio_beginning) {
    FRESULT fr;
    if (CORE_1_STATE.REQUESTED_FILE_ID > 0) {
        DIR dj;
        FILINFO fno;
        // sprintf(file_pattern, "%i_*.wav", CORE_1_STATE.FILE_ID);
        // fr = f_findfirst(&dj, &fno, "", file_pattern);
        // f_closedir(&dj);
        // if (fr == FR_OK && fno.fname) {  
        //     printf("Full file name: %s\n", fno.fname);
        // } else {
        //     printf("Missing file: file id %d\n", CORE_1_STATE.FILE_ID);
        //     continue;
        // }
        //strncpy(filename, fno.fname, 50);
        sprintf(filename, "%i.wav", CORE_1_STATE.REQUESTED_FILE_ID);
    } else {
        switch(CORE_1_STATE.REQUESTED_FILE_ID){
            case INSERT_CARD: strncpy(filename, "insert_card.wav", 50); break;
            case SUCCESS: strncpy(filename, "success.wav", 50); break;
            case HELLO: strncpy(filename, "hello.wav", 50); break;
            case BYE: strncpy(filename, "bye.wav", 50); break;
            default: return false;
        }
    }
    fr = f_open(file, filename, FA_READ | FA_OPEN_EXISTING);
    if (fr != FR_OK) {
        printf("Nie można otworzyć pliku %s:  %s (%d)\n", filename, FRESULT_str(fr), fr);
        return false;
    }
    // Odczytaj nagłówek WAV
    if (!read_wav_header(file, &wav_header)) {
        f_close(file);
        return false;
    }
    if (wav_header.sample_rate != audio_format.sample_freq) {
        printf("Warning: Sample rate mismatch - reconfigure I2S\n");
        f_close(file);
        return false;
    }
    *file_id = CORE_1_STATE.REQUESTED_FILE_ID;
    *audio_beginning = f_tell(file);
    return true;
}

void adjust_volume(int16_t *samples, uint32_t samples_count, uint16_t num_channels) {
    for (int i = 0; i < samples_count * num_channels; i++) {
        int32_t scaled = (int32_t)samples[i] * VOL;  // multiply sample by volume scale
        scaled >>= 16;  // scale back down by shifting right 16 bits
        // Clamp to int16_t range if necessary
        if (scaled > 32767) scaled = 32767;
        else if (scaled < -32768) scaled = -32768;
        samples[i] = (int16_t)scaled;
    }
}

void core1_main() {
        bool data_available;
        uint32_t received_data;
        Core1Job job = IDLE;
        UINT bytes_read;
        FRESULT fr;
        FILINFO finfo;
        int file_id = 0;
        char file_pattern[50];
        char filename[50];
        FIL file;
        FSIZE_t audio_beginning;
        uint32_t last_busy = time_us_32();

        sleep_ms(25); // wait until modules initializes on power up??

        ap = init_audio();
        sleep_ms(5);

        int rc = sd_card_init();
        hard_assert(rc == PICO_OK);
        sleep_ms(5);

        CORE_1_STATE.REQUESTED_FILE_ID = HELLO;
        if (open_audio_file(&file_id, filename, &file, &audio_beginning)) {
            audio_i2s_set_enabled(true);
            while (true) {
                struct audio_buffer *buffer = take_audio_buffer(ap, true);
                int16_t *samples = (int16_t *) buffer->buffer->bytes;
                    
                fr = f_read(&file, 
                        samples,
                        buffer->max_sample_count * sizeof(int16_t) * wav_header.num_channels, 
                        &bytes_read);

                if (fr == FR_OK) {
                    buffer->sample_count = bytes_read / (sizeof(int16_t) * wav_header.num_channels);
                    adjust_volume(samples, buffer->sample_count, wav_header.num_channels);
                } else {
                    audio_i2s_set_enabled(false);
                    buffer->sample_count = 0;
                }
                give_audio_buffer(ap, buffer);
                last_busy = time_us_32();

                if (fr != FR_OK || bytes_read == 0) {
                    audio_i2s_set_enabled(false);
                    f_close(&file);
                    printf("[Core#1] End hello msg\n");
                    break;
                }
            }
        }
        CORE_1_STATE.REQUESTED_FILE_ID = 0;

        CORE_1_STATE.READY = true;
        
        while (!CORE_1_STATE.TERMINATE) {
            while (CORE_1_STATE.FULL_IDLE || CORE_1_STATE.DORMANT_READY) {
                sleep_us(SAMPLES_SLEEP_US);
                if (CORE_1_STATE.CORE_0_REQUEST_DORMANT && !CORE_1_STATE.DORMANT_READY) {
                    CORE_1_STATE.REQUESTED_FILE_ID = BYE;
                    if (open_audio_file(&file_id, filename, &file, &audio_beginning)) {
                        audio_i2s_set_enabled(true);
                        while (true) {
                            struct audio_buffer *buffer = take_audio_buffer(ap, true);
                            int16_t *samples = (int16_t *) buffer->buffer->bytes;
                                
                            fr = f_read(&file, 
                                    samples,
                                    buffer->max_sample_count * sizeof(int16_t) * wav_header.num_channels, 
                                    &bytes_read);

                            if (fr == FR_OK) {
                                buffer->sample_count = bytes_read / (sizeof(int16_t) * wav_header.num_channels);
                                adjust_volume(samples, buffer->sample_count, wav_header.num_channels);
                            } else {
                                audio_i2s_set_enabled(false);
                                buffer->sample_count = 0;
                            }
                            give_audio_buffer(ap, buffer);
                            last_busy = time_us_32();

                            if (fr != FR_OK || bytes_read == 0) {
                                audio_i2s_set_enabled(false);
                                f_close(&file);
                                printf("[Core#1] End bye msg\n");
                                break;
                            }
                        }
                    }
                    CORE_1_STATE.REQUESTED_FILE_ID = 0;
                    sd_card_deinit();
                    deinit_audio();
                    CORE_1_STATE.DORMANT_READY = true;
                }
            }
            if (job == IDLE) {
                data_available = multicore_fifo_pop_timeout_us(10*SAMPLES_SLEEP_US, &received_data);
                if (data_available) {
                    if (received_data == START_COLLECTING_DATA_MSG) {
                        job = GATHERING;
                        printf("[Core#1] Change job to gathering\n");
                    } else if (received_data == START_PLAYING_MSG) {
                        if (!open_audio_file(&file_id, filename, &file, &audio_beginning)) {
                            continue;
                        }
                        audio_i2s_set_enabled(true);
                        job = PLAYING;
                        printf("[Core#1] Change job to playing: %s\n", filename);
                    } else if (received_data == STOP_PLAYING_MSG) {
                        // DO NOTHING
                    } else if (received_data == CHANGE_VOLUME_MSG) {
                        AUDIO_VOLUME old_vol = cycle_volume();
                        printf("[Core#1] Change volume: 0x%04x (%d %%) -> %04x (%d %%)\n", old_vol, 100*old_vol/VOL_100, VOL, 100*VOL/VOL_100);
                    } 
                    last_busy = time_us_32();
                } else {
                    if (time_us_32() - last_busy > SLEEP_TIMEOUT_US) {
                        CORE_1_STATE.FULL_IDLE = true;
                        printf("[Core#1] Core1 - dormant ready\n");
                        continue;
                    }
                    sleep_us(SAMPLES_SLEEP_US);
                }
            }
            if (job == PLAYING) {
                while (job == PLAYING) {
                    data_available = multicore_fifo_rvalid();
                    if (data_available) {
                        received_data = multicore_fifo_pop_blocking();
                        if (received_data == START_COLLECTING_DATA_MSG) {
                            audio_i2s_set_enabled(false);
                            f_close(&file);
                            job = GATHERING;
                            printf("[Core#1] Change job to gathering - stop playing sound - collecting msg came\n");
                            break;
                        } else if (received_data == START_PLAYING_MSG) {
                            audio_i2s_set_enabled(false);
                            if (CORE_1_STATE.REQUESTED_FILE_ID == file_id) { // restart file
                                fr = f_lseek(&file, audio_beginning);
                                if (fr != FR_OK) {
                                    printf("[Core#1] Change job to idle - error - cannot seek to file start\n");
                                    f_close(&file);
                                    job = IDLE;
                                    break;
                                }
                                printf("[Core#1] Restart playing: %s\n", filename);
                                audio_i2s_set_enabled(true);
                                blink_status_led(1);
                            } else { // change file
                                f_close(&file);
                                if (!open_audio_file(&file_id, filename, &file, &audio_beginning)) {
                                    continue;
                                }
                                printf("[Core#1] Start playing: %s\n", filename);
                                audio_i2s_set_enabled(true);
                            }
                        }  else if (received_data == STOP_PLAYING_MSG) {
                            audio_i2s_set_enabled(false);
                            f_close(&file);
                            job = IDLE;
                            printf("[Core#1] Change job to idle - stop playing msg came\n");
                            break;
                        } else if (received_data == CHANGE_VOLUME_MSG) {
                            AUDIO_VOLUME old_vol = cycle_volume();
                            printf("[Core#1] Change volume: 0x%04x (%d %%) -> %04x (%d %%)\n", old_vol, 100*old_vol/VOL_100, VOL, 100*VOL/VOL_100);
                        } 
                    }
                    struct audio_buffer *buffer = take_audio_buffer(ap, false);
                    if (buffer != NULL) {
                        int16_t *samples = (int16_t *) buffer->buffer->bytes;
                        UINT bytes_TO_read = buffer->max_sample_count * sizeof(int16_t) * wav_header.num_channels;
                            
                        fr = f_read(&file, 
                                samples,
                                bytes_TO_read, 
                                &bytes_read);

                        if (fr == FR_OK) {
                            buffer->sample_count = bytes_read / (sizeof(int16_t) * wav_header.num_channels);
                            adjust_volume(samples, buffer->sample_count, wav_header.num_channels);
                        } else {
                            audio_i2s_set_enabled(false);
                            buffer->sample_count = 0;
                        }
                        give_audio_buffer(ap, buffer);
                        last_busy = time_us_32();

                        if (fr != FR_OK || bytes_read < bytes_TO_read) {
                            audio_i2s_set_enabled(false);
                            f_close(&file);
                            last_busy = time_us_32();
                            job = IDLE;
                            printf("[Core#1] Change job to idle - %s\n", fr != FR_OK ? "Read error" : "End of file");
                            break;
                        }
                    } else {
                        sleep_us(SAMPLES_SLEEP_US);
                    }
                }
            }
            if (job == GATHERING) {
                multicore_fifo_drain();
                barcode::gather_barcode_raw_data();
                job = IDLE;
                printf("[Core#1] Change job to idle - end of gathering code\n");
                last_busy = time_us_32();
            }
        }
    }

void disable_usb_stdio() {
    #if LIB_PICO_STDIO_USB
    stdio_flush();
    stdio_set_driver_enabled(&stdio_usb, false);
    #endif
}

void enable_usb_stdio() {
    #if LIB_PICO_STDIO_USB
    stdio_set_driver_enabled(&stdio_usb, true);
    stdio_flush();
    #endif
}

void prepare_for_sleep() {
    CORE_1_STATE.CORE_0_REQUEST_DORMANT = true;
    gpio_set_irq_enabled(REPLAY_GPIO, GPIO_IRQ_EDGE_RISE, false);
    gpio_set_irq_enabled(VOL_GPIO, GPIO_IRQ_EDGE_RISE, false);
    barcode::deinitialize_module();
    while(!CORE_1_STATE.DORMANT_READY) {
        sleep_ms(5);
    }
    if (VOL != SAVED_VOL) {
        printf("Saving volume to flash: %08x\n", VOL);
        save_volume_to_flash(VOL);
    }
    power_led_put(false);
    gpio_put(LED_YELLOW_GPIO, 0);
    sleep_ms(1);
    uart_default_tx_wait_blocking();
    sleep_run_from_xosc();
    uart_default_tx_wait_blocking();
    disable_usb_stdio();
}

void sleep() {
    sleep_goto_dormant_until_edge_high(WAKE_GPIO);
}

bool is_wake_up_pressed_debounced() {
    if (gpio_get(WAKE_GPIO) == 0) {
        sleep_ms(20);       
        if (gpio_get(WAKE_GPIO) == 0) {
            return false;
        }
    }
    return true;
}

bool is_long_pressed() {
    if(!is_wake_up_pressed_debounced()) return false;  // Released early

    const uint32_t interval = 100;
    const uint32_t total_duration = 2'000;
    uint32_t elapsed = 0;

    while (elapsed < total_duration) {
        sleep_ms(interval);
        elapsed += interval;
        if (!is_wake_up_pressed_debounced()) return false;  // Released early
    }
    return true;  // Held pressed for 5 seconds
}

void restore_minimal_after_sleep() {
    sleep_power_up();
    int rc = init_buttons();
    hard_assert(rc == PICO_OK);
    sleep_ms(10); // allow GPIO input stabilization
    gpio_get(WAKE_GPIO); // ignore first reading
}

bool should_wake_up() {
    restore_minimal_after_sleep();
    return is_long_pressed();
}

void full_wake_up() {
    CORE_1_STATE.FULL_IDLE = false;
    enable_usb_stdio();
    power_led_put(true);
    // initialize everything
}

void restart_pico() {
    watchdog_enable(STATUS_LED_MS, 1);
}

bool enter_sleep_if_both_cores_ready() {
    printf("core 0 ready for dormant\n");
    if (CORE_1_STATE.FULL_IDLE) {
        printf("core 1 ready dormant\n");
        do {
            prepare_for_sleep();
            sleep();
        } while(!should_wake_up());
        blink_status_led(1);
        restart_pico();
        return true;
    }
    return false;
}

uint32_t last_replay_ts_us = 0;

void replay_button_callback() {
    uint32_t now_us = time_us_32();
    if (now_us - last_replay_ts_us > 1'000'000) { // 1 second between replay msg
        multicore_fifo_push_timeout_us(START_PLAYING_MSG, SAMPLES_SLEEP_US);
        last_replay_ts_us = now_us;
    }
}

uint32_t last_volume_ts_us = 0;

void volume_button_callback() {
    uint32_t now_us = time_us_32();
    if (now_us - last_volume_ts_us > 100'000) { // 100 milisecond between volume change msg
        multicore_fifo_push_timeout_us(CHANGE_VOLUME_MSG, SAMPLES_SLEEP_US);
        last_volume_ts_us = now_us;
    }
}

void gpio_irq_callback(uint gpio, uint32_t events) {
    if (gpio == REPLAY_GPIO) {
        replay_button_callback();
    } else if (gpio == VOL_GPIO) {
        volume_button_callback();
    }
}

int main()
{
    stdio_init_all();
    time_init();
    int rc = init_led();
    hard_assert(rc == PICO_OK);
    blink_status_led(1);
    while (!stdio_usb_connected()) { sleep_ms(10); } // wait for monitor to connect

    adc_init();
    sleep_ms(5);
    adc_gpio_init(29);
    sleep_ms(5);
    adc_select_input(3); // Select ADC input 3 (GPIO29)
    sleep_ms(5);
    const float conversion_factor = 3 * 3.3f / (1 << 12); // 3.3V reference / 12-bit range,  Pico divides VSYS by 3 for the ADC
    float vsys_voltage = adc_read() * conversion_factor; // Voltage at divider
    const float empty_battery = 3.2f;
    printf("VSYS voltage: %.2f V\n", vsys_voltage);
    if (vsys_voltage <= empty_battery) {
        printf("Battery low - enter dormant mode");
        do {
            prepare_for_sleep();
            sleep();
        } while(true);
    }

    hard_assert(is_valid_flash_address(ALIGNED_FLASH_BINARY_END_ADDR, FLASH_SECTOR_SIZE));
    printf("FLASH variables: %08x %08x %08x\n", __flash_binary_end_addr, ALIGNED_FLASH_BINARY_END_ADDR, ALIGNED_FLASH_OFFSET);

    SAVED_VOL = read_saved_volume();
    printf("Loaded volume: 0x%04x (%d %%)\n", SAVED_VOL, 100*SAVED_VOL/VOL);
    VOL = SAVED_VOL;

    rc = init_buttons();
    hard_assert(rc == PICO_OK);
    barcode::initialize_module();
    sleep_ms(5);

    power_led_put(true);
    sleep_ms(5);

    multicore_launch_core1(core1_main);
    sleep_ms(50);

    CORE_1_STATE.REQUESTED_FILE_ID = 0;

    barcode::initialize_baseline();

    while (!CORE_1_STATE.READY) {
        sleep_ms(25);
    }

    gpio_set_irq_callback(gpio_irq_callback);
    gpio_set_irq_enabled(REPLAY_GPIO, GPIO_IRQ_EDGE_RISE, true);
    gpio_set_irq_enabled(VOL_GPIO, GPIO_IRQ_EDGE_RISE, true);
    irq_set_enabled(IO_IRQ_BANK0, true);

    printf("system initialized\n");
    
    while(true) {
        uint8_t timeout_cnt = 0;
        while(barcode::wait_for_missing_card()) {
            if (timeout_cnt > 60) {
                if (enter_sleep_if_both_cores_ready()) {
                    timeout_cnt = 0;
                }
            } else {
                timeout_cnt++;
            }
        }
        multicore_fifo_push_blocking(STOP_PLAYING_MSG);
        timeout_cnt = 0;
        while(barcode::wait_for_card()) {
            if (timeout_cnt > 60) {
                if (enter_sleep_if_both_cores_ready()) {
                    timeout_cnt = 0;
                }
            } else {
                timeout_cnt++;
            }
        }
        CORE_1_STATE.FULL_IDLE = false;
        int code = barcode::read_barcode();
        if (code != -1) {
            CORE_1_STATE.REQUESTED_FILE_ID = code;
            multicore_fifo_push_blocking(START_PLAYING_MSG);
            blink_status_led(1);
        } else {
            CORE_1_STATE.REQUESTED_FILE_ID = INSERT_CARD;
            multicore_fifo_push_blocking(START_PLAYING_MSG);
        }
        sleep_ms(50);
    }
}
#include <vector>
#include "audio_player.h"
#include "pico/stdlib.h"
#include "sd_card.h"
#include "SPI/sd_card_spi.h"
#include "ff.h"
#include "f_util.h"
#include "hw_config.h"
#include "pico/audio_i2s.h"
#include "my_rtc.h"

#define R1_IDLE_STATE 1 << 0 // sc_card_spi.c -> spi_r1_response_t

namespace audio {
    const AUDIO_VOLUME VOLUME_VALUES[] = {VOL_100, VOL_80, VOL_60, VOL_40, VOL_20};
    const size_t volume_values_length = sizeof(VOLUME_VALUES) / sizeof(VOLUME_VALUES[0]);
    bool INCREASE_VOLUME_FLAG = true;


    class WavReader {
        public:
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

            typedef struct {
                bool success;
                wav_header_t *wav_header;
            } wav_read_result;

        private:    
            wav_header_t wav_header;
            static const wav_read_result FAILED_RESULT;
        
        public:
            WavReader() {}

             // Odczyt nagłówka pliku WAV
            wav_read_result read_wav_header(FIL* file) {
                UINT bytes_read;
                FRESULT fr = f_read(file, &wav_header, sizeof(wav_header_t), &bytes_read);
                
                if (fr != FR_OK || bytes_read != sizeof(wav_header_t)) {
                    printf("Error reading WAV header\n");
                    return FAILED_RESULT;
                }

                if (memcmp(wav_header.riff_header, "RIFF", 4) != 0) {
                    printf("Incorrect WAV format [%.*s] - not RIFF\n", 4, wav_header.riff_header);
                    return FAILED_RESULT;
                }

                if (memcmp(wav_header.wave_header, "WAVE", 4) != 0) {
                    printf("Incorrect WAV format - not WAVE file\n");
                    return FAILED_RESULT;
                }

                if (memcmp(wav_header.fmt_header, "fmt", 3) != 0) {
                    printf("Incorrect WAV format - no format section found\n");
                    return FAILED_RESULT;
                }

                if (memcmp(wav_header.data_header, "data", 4) != 0) {
                    if (memcmp(wav_header.data_header, "LIST", 4) == 0) {
                        wav_section_t next_section_header;
                        fr = f_lseek(file, f_tell(file) + wav_header.data_size);
                        if (fr != FR_OK) {
                            printf("Incorrect WAV format - cannot seek after LIST section\n");
                            return FAILED_RESULT;
                        }
                        fr = f_read(file, &next_section_header, sizeof(wav_section_t), &bytes_read);
                        if (fr != FR_OK || bytes_read != sizeof(wav_section_t)) {
                            printf("Error reading WAV data header\n");
                            return FAILED_RESULT;
                        }
                        if (memcmp(next_section_header.section_header, "data", 4) == 0) {
                            memcpy(wav_header.data_header, next_section_header.section_header, 4);
                            wav_header.data_size = next_section_header.section_size;
                        }  
                    }
                }

                if (memcmp(wav_header.data_header, "data", 4) != 0) {
                    printf("Incorrect WAV format %.*s - no data section found\n", 4, wav_header.data_header);
                    return FAILED_RESULT;
                }
                
                if (wav_header.audio_format != 1) {
                    printf("Invalid WAV format - handling only AUDIO_BUFFER_FORMAT_PCM_S16\n");
                    return FAILED_RESULT;
                }

                if(wav_header.fmt_chunk_size != 16) 
                {
                    printf("Invalid WAV format - format section size must be 16.\n");
                    return FAILED_RESULT;                          
                }

                if((wav_header.num_channels != audio_format.channel_count))
                {
                    printf("Invalid WAV format - handling only %d channels (1 - mono / 2- stereo).\n", audio_format.channel_count);
                    return FAILED_RESULT;   
                }
                if(wav_header.sample_rate != audio_format.sample_freq) 
                {
                    printf("Invalid WAV format - sample frequency must be [%d].\n", audio_format.sample_freq);
                    return FAILED_RESULT;                       
                }
                if((wav_header.bit_depth != 16))
                {
                    printf("Invalid WAV format - PCM_S16 must have 16 bits per sample.");
                    return FAILED_RESULT;                        
                }
                
                printf("WAV parameters:\n");
                printf("- Sample rate: %lu Hz\n", wav_header.sample_rate);
                printf("- Channels number: %u\n", wav_header.num_channels);
                printf("- Bit depth: %u\n", wav_header.bit_depth);
                printf("- Data size: %lu bytes\n", wav_header.data_size);
                
                return wav_read_result { true, &wav_header };
            }
    };

    const WavReader::wav_read_result WavReader::FAILED_RESULT = { false, nullptr };
    // void add_spi(spi_t *const spi);
    // void add_spi_if(sd_spi_if_t *const spi_if);
    // void add_sd_card(sd_card_t *const sd_card);

    class SDReader {
        private:
            static std::vector<spi_t *> spis;             // SPI H/W components
            static std::vector<sd_spi_if_t *> spi_ifs;    // SPI Interfaces
            static spi_t *p_spi;
            static FATFS fs;
            FIL opened_file;
            char opened_filename[50];
            static bool file_is_open;
            FSIZE_t audio_beginning;
            WavReader::wav_header_t *wav_header;
            WavReader wav_reader;

        public:
            static std::vector<sd_card_t *> sd_cards;     // SD Card Sockets

            SDReader(): wav_reader() {}

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

            bool open_audio_file(char *filename) {
                FRESULT fr;
                if (file_is_open) {
                    f_close(&opened_file);
                    file_is_open = false;
                }
                fr = f_open(&opened_file, filename, FA_READ | FA_OPEN_EXISTING);
                if (fr != FR_OK) {
                    printf("Error: cannot open file %s:  %s (%d)\n", filename, FRESULT_str(fr), fr);
                    return false;
                }
                file_is_open = true;
                // Odczytaj nagłówek WAV
                WavReader::wav_read_result result = wav_reader.read_wav_header(&opened_file);
                if (!result.success) {
                    f_close(&opened_file);
                    file_is_open = false;
                    return false;
                }
                wav_header = result.wav_header;
                if (wav_header->sample_rate != audio_format.sample_freq) {
                    printf("Warning: Sample rate mismatch - reconfigure I2S\n");
                    f_close(&opened_file);
                    file_is_open = false;
                    return false;
                }
                strncpy(opened_filename, filename, 50);
                audio_beginning = f_tell(&opened_file);
                return true;
            }

            bool rewind_opened_file_to_audio_beginning() {
                if (!is_file_opened()) {
                    printf("Warning: Trying to rewind file without opened file\n");
                    return false;
                }
                FRESULT fr = f_lseek(&opened_file, audio_beginning);
                if (fr != FR_OK) {
                    printf("Error: cannot seek to file start %s: %s (%d)\n", opened_filename, FRESULT_str(fr), fr);
                    close_file();
                    return false;
                }
                return true;
            }

            int get_channels_number() {
                if (!file_is_open) {
                    return -1;
                }
                return wav_header->num_channels;
            }

            int get_audio_bytes() {
                if (!file_is_open) {
                    return -1;
                }
                return wav_header->data_size;
            }

            bool is_file_opened() {
                return file_is_open;
            }

            UINT fill_buffer(int16_t *buffer, UINT bytes_to_read) {
                FRESULT fr;
                UINT bytes_read;
                if (!file_is_open) {
                    printf("Warning: Tried to fill buffer without opened file\n");
                    return 0;
                }
                fr = f_read(&opened_file,
                            buffer,
                            bytes_to_read,
                            &bytes_read);

                if (fr != FR_OK) {
                    f_close(&opened_file);
                    file_is_open = false;
                    printf("Error: while reading file %s: %s (%d)", opened_filename, FRESULT_str(fr), fr);
                    return 0;
                }

                return bytes_read;
            }

            bool close_file() {
                if (file_is_open) {
                    f_close(&opened_file);
                    file_is_open = false;
                    return true;
                }
                return false;
            }
    };

    bool SDReader::file_is_open = false;
    std::vector<spi_t *> audio::SDReader::spis = {};
    std::vector<sd_spi_if_t *> audio::SDReader::spi_ifs = {};
    std::vector<sd_card_t *> audio::SDReader::sd_cards = {};

    class I2SAudioPlayer {
        private:
            audio::AUDIO_VOLUME VOL = audio::VOL_100;
            audio_buffer_pool *buffer_pool;
            SDReader sd_reader;
            FSIZE_t bytes_played = 0;

            void init_audio() {
                printf("Variables %d %d %d %d\n", PICO_AUDIO_I2S_DATA_PIN, PICO_AUDIO_I2S_CLOCK_PIN_BASE, PICO_AUDIO_I2S_CLOCK_PINS_SWAPPED, AUDIO_SHUTDOWN_PIN);

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

                printf("Audio I2S inicjalized successfully\n");
                buffer_pool = producer_pool;
            }

            void deinit_audio() {
                audio_i2s_set_enabled(false);
                gpio_put(AUDIO_SHUTDOWN_PIN, false);
            }
        public:
            I2SAudioPlayer(): sd_reader() {}

            void initialize(AUDIO_VOLUME saved_volume) {
                VOL = saved_volume;
                init_audio();
                sleep_ms(5);

                int rc = sd_reader.sd_card_init();
                hard_assert(rc == PICO_OK);
            }

            void deinit() {
                sd_reader.sd_card_deinit();
                deinit_audio();
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

            AUDIO_VOLUME cycle_volume() {
                AUDIO_VOLUME previous = VOL;
                uint8_t blink_times = 1;
                
                for(int i = 0; i < volume_values_length; i++) {
                    if (previous == VOLUME_VALUES[i]) {
                        if ((i == 0 && INCREASE_VOLUME_FLAG) || (i + 1 == volume_values_length && !INCREASE_VOLUME_FLAG)) {
                            INCREASE_VOLUME_FLAG = !INCREASE_VOLUME_FLAG;
                        }
                        int new_value_index = INCREASE_VOLUME_FLAG ? i - 1: i + 1;
                        VOL = VOLUME_VALUES[new_value_index];
                        if (is_edge_volume(VOL)) {
                            blink_times = 1;
                        }
                        break;
                    }
                }
                return previous;
            }

            AUDIO_VOLUME current_volume() {
                return VOL;
            }

            bool is_edge_volume(audio::AUDIO_VOLUME volume) {
                return volume == VOLUME_VALUES[0] || volume == VOLUME_VALUES[volume_values_length - 1];
            }

            bool is_current_vol_edge_volume() {
                return is_edge_volume(VOL);
            }

            bool open_audio_file(char *filename) {
                bool opened = sd_reader.open_audio_file(filename);
                if (opened) {
                    audio_i2s_set_enabled(true);
                    bytes_played = 0;
                }
                return opened;
            }

            bool rewind_opened_file() {
                audio_i2s_set_enabled(false);
                bool success = sd_reader.rewind_opened_file_to_audio_beginning();
                if (success) {
                    audio_i2s_set_enabled(true);
                    bytes_played = 0;
                }
                return success;
            }

            play_result_t play_next_chunk(bool blocking) {
                if (!sd_reader.is_file_opened()) {
                    printf("Warning: Trying to play next chunk without opened file\n");
                    return play_result_t { false, false };
                }
                FSIZE_t bytes_remaining = sd_reader.get_audio_bytes() - bytes_played;
                if (bytes_remaining > 0) {
                    struct audio_buffer *buffer = take_audio_buffer(buffer_pool, blocking);
                    if (buffer != NULL) {
                        uint16_t num_channels = sd_reader.get_channels_number();
                        int16_t *samples = (int16_t *) buffer->buffer->bytes;
                        UINT bytes_TO_read = buffer->max_sample_count * sizeof(int16_t) * num_channels;
                        UINT bytes_TO_read_limited = (bytes_TO_read <= bytes_remaining) ? bytes_TO_read : (UINT) bytes_remaining;
                        UINT bytes_read = sd_reader.fill_buffer(samples, bytes_TO_read_limited);
                        if (bytes_read > 0) {
                            buffer->sample_count = bytes_read / (sizeof(int16_t) * num_channels);
                            adjust_volume(samples, buffer->sample_count, num_channels);
                            bytes_played = bytes_played + bytes_read;
                            bytes_remaining = sd_reader.get_audio_bytes() - bytes_played;
                        } else {
                            audio_i2s_set_enabled(false);
                            buffer->sample_count = 0;
                        }
                        give_audio_buffer(buffer_pool, buffer);

                        if (bytes_read < bytes_TO_read_limited || bytes_remaining <= 0) {
                            if (bytes_remaining > 0) {
                                printf("Warning: File ended before audio_bytes read\n");
                            }
                            audio_i2s_set_enabled(false);
                            sd_reader.close_file();
                            return play_result_t { false, true };
                        }

                        return play_result_t { bytes_remaining > 0, true };
                    }
                }
                return play_result_t { bytes_remaining > 0, false };
            }

            void play_whole_file() {
                if (sd_reader.is_file_opened()) {
                    while(play_next_chunk(true).next_chunk_available) {}
                }   
            }

            bool close_audio_file() {
                audio_i2s_set_enabled(false);
                return sd_reader.close_file();
            }            
    };

    static I2SAudioPlayer i2s_sd_audio_player = I2SAudioPlayer();

    void initialize_module(AUDIO_VOLUME saved_volume) {
        sleep_ms(25);

        i2s_sd_audio_player.initialize(saved_volume);

        sleep_ms(5); // w8t for stabilization
    }

    void deinitialize_module() {
        i2s_sd_audio_player.deinit();

        sleep_ms(5);
    }

    bool open_file(char *filename) {
        return i2s_sd_audio_player.open_audio_file(filename);
    }

    bool rewind_to_audio_beginning() {
        return i2s_sd_audio_player.rewind_opened_file();
    }

    play_result_t play_next_chunk(bool blocking) {
        return i2s_sd_audio_player.play_next_chunk(blocking);
    }

    void blocking_play_whole_file() {
        i2s_sd_audio_player.play_whole_file();
    }

    bool close_opened_file() {
        return i2s_sd_audio_player.close_audio_file();
    }

    AUDIO_VOLUME cycle_volume() {
        return i2s_sd_audio_player.cycle_volume();
    }

    AUDIO_VOLUME current_volume() {
        return i2s_sd_audio_player.current_volume();
    }

    bool is_current_volume_edge_volume() {
        return i2s_sd_audio_player.is_current_vol_edge_volume();
    }
}

size_t sd_get_num() { return audio::SDReader::sd_cards.size(); }

sd_card_t *sd_get_by_num(size_t num) {
    if (num <= sd_get_num()) {
        return audio::SDReader::sd_cards[num];
    } else {
        return NULL;
    }
}
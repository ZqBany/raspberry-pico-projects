#ifndef AUDIO_PLAYER_H
#define AUDIO_PLAYER_H

#include "pico/audio_i2s.h"

#define SD_SPI_PORT spi0
#define SD_CS_PIN 17
#define SD_SCK_PIN 18
#define SD_MOSI_PIN 19
#define SD_MISO_PIN 16

#define SAMPLES_PER_BUFFER 512
#define AUDIO_I2S_DMA_CHANNEL 0
#define AUDIO_SHUTDOWN_PIN 21 // Connected via 470kOhm resistor to MAX98357A2 SD pin

namespace audio
{   
    enum AUDIO_VOLUME {
        VOL_100 = 0xFFFF,
        VOL_80 = 0xCCCC,
        VOL_60 = 0x9999,
        VOL_40 = 0x6666,
        VOL_20 = 0x3333
    };

    static const audio_format_t audio_format = {
        .sample_freq = 44100,
        .format = AUDIO_BUFFER_FORMAT_PCM_S16,
        .channel_count = 2,
    };

    typedef struct {
        bool next_chunk_available;
        bool buffer_processed;
    } play_result_t;

    void initialize_module(AUDIO_VOLUME saved_volume);
    void deinitialize_module();
    bool file_exists(char *filename);
    bool open_file(char *filename);
    bool rewind_to_audio_beginning();
    play_result_t play_next_chunk(bool blocking);
    void blocking_play_whole_file();
    bool close_opened_file();
    AUDIO_VOLUME cycle_volume();
    AUDIO_VOLUME current_volume();
    bool is_current_volume_edge_volume();
}

#endif // AUDIO_PLAYER_H
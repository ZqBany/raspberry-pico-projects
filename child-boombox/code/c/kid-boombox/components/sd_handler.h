#ifndef SD_HANDLER_H
#define SD_HANDLER_H

#include <type_traits>

#define SD_SPI_PORT spi0
#define SD_CS_PIN 17
#define SD_SCK_PIN 18
#define SD_MOSI_PIN 19
#define SD_MISO_PIN 16

namespace sd_handler
{
    struct sd_file_t;

    void initialize_module();
    void deinitialize_module();
    bool file_exists(char *filename);
    bool open_file_for_read(char *filename, sd_file_t* file);
    size_t current_file_position(sd_file_t* file);
    bool seek_file_position(sd_file_t* file, size_t file_pointer);
    size_t fill_buffer(sd_file_t* file, int16_t *buffer, size_t bytes_to_read);
    size_t read_bytes(sd_file_t* file, void* buffer, size_t bytes_to_read);
    bool close_file(sd_file_t* file);
    
    template <typename T>
    size_t read_struct(sd_file_t* file, T* struct_to_fill) {
        static_assert(std::is_trivial<T>::value, "T must be Plain Old Data (trivial check failed)");
        static_assert(std::is_standard_layout<T>::value, "T must be Plain Old Data (standard-layout check failed)");
        return read_bytes(file, struct_to_fill, sizeof(T));
    };
}

#endif // SD_HANDLER_H
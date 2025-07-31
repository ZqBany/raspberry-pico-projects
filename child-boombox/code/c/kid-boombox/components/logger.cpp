
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "logger.h"
#include "sd_handler.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#if ENABLE_FILE_LOGGING
#include "ff.h"
#endif

namespace logger {

    // Static member definitions
    LogLevel Logger::current_log_level = LOG_LEVEL_INFO;
    LogOutput Logger::output_mode = OUTPUT_BOTH;
    
    bool Logger::include_timestamp = true;
    bool Logger::include_level = true;
    volatile bool Logger::initialized = false;
    char Logger::format_buffer[256];
    
    const char* Logger::level_strings[] = {
        "ERROR", "WARN", "INFO", "DEBUG", "TRACE"
    };

    #if ENABLE_FILE_LOGGING
    void* Logger::                                                                                                                                    log_file = nullptr;
    const char* Logger::LOG_FILENAME = "logger.txt";

    // Check if logger.txt exceeds size limit
    bool Logger::is_file_oversized() {
        FILINFO file_info;
        FRESULT fr = f_stat(LOG_FILENAME, &file_info);
        
        if (fr == FR_OK) {
            return file_info.fsize >= MAX_FILE_SIZE;
        }
        return false;
    }

    // Rotate log files: logger.txt -> logger_1.txt -> logger_2.txt -> ... -> logger_4.txt
    bool Logger::rotate_log_files() {
        char old_name[64];
        char new_name[64];
        FRESULT fr;

        // Remove the oldest file (logger_4.txt)
        snprintf(old_name, sizeof(old_name), "logger_%d.txt", MAX_LOG_FILES - 1);
        f_unlink(old_name); // Ignore result - file might not exist

        // Shift files: logger_3.txt -> logger_4.txt, logger_2.txt -> logger_3.txt, etc.
        for (int i = MAX_LOG_FILES - 2; i >= 1; i--) {
            snprintf(old_name, sizeof(old_name), "logger_%d.txt", i);
            snprintf(new_name, sizeof(new_name), "logger_%d.txt", i + 1);
            
            // Check if old file exists before renaming
            FILINFO file_info;
            if (f_stat(old_name, &file_info) == FR_OK) {
                fr = f_rename(old_name, new_name);
                if (fr != FR_OK) {
                    printf("Failed to rename %s to %s, error: %d\n", old_name, new_name, fr);
                }
            }
        }

        // Rename current logger.txt to logger_1.txt
        fr = f_rename(LOG_FILENAME, "logger_1.txt");
        if (fr != FR_OK) {
            printf("Failed to rename current log file, error: %d\n", fr);
            return false;
        }

        return true;
    }

    // Initialize logger - called automatically on first log
    bool Logger::initialize_file() {
        if (initialized) {
            return log_file != nullptr;
        }

        // Use static allocation instead of dynamic allocation
        static FIL file_storage;
        log_file = &file_storage;

        // Check if logger.txt exists and is oversized (ONLY during initialization)
        if (is_file_oversized()) {
            printf("Log file oversized, rotating files...\n");
            
            // Rotate existing files
            if (!rotate_log_files()) {
                printf("Failed to rotate log files\n");
                log_file = nullptr;
                initialized = true;
                return false;
            }
        }

        // Open logger.txt for append or create new
        FRESULT fr = f_open(static_cast<FIL*>(log_file), LOG_FILENAME, FA_WRITE | FA_OPEN_APPEND);
        
        if (fr == FR_OK) {
            initialized = true;
            printf("Logger initialized successfully\n");
            return true;
        } else {
            printf("Failed to open log file: %s, error: %d\n", LOG_FILENAME, fr);
            log_file = nullptr;
            initialized = true;
            return false;
        }
    }
    #endif

    // Get microsecond timestamp using Pico SDK
    uint64_t Logger::get_timestamp_us() {
        return to_us_since_boot(get_absolute_time());
    }

    bool Logger::initialize() {
        #if ENABLE_FILE_LOGGING
        sd_handler::initialize_module();
        return initialize_file();
        #endif
        initialized = true;
        return initialized;
    }


    // Core logging implementation
    void Logger::log_core(LogLevel level, const char* format, va_list args) {
        // Early return for filtered log levels
        if (level > current_log_level) {
            return;
        }

        // Auto-initialize on first use
        if (!initialized) {
            initialize();
        }

        char* buffer_pos = format_buffer;
        size_t remaining = sizeof(format_buffer);
        int written = 0;

        // Add level prefix if enabled
        if (include_level) {
            written = snprintf(buffer_pos, remaining, "[%s] ", level_strings[level]);
            if (written > 0 && written < (int)remaining) {
                buffer_pos += written;
                remaining -= written;
            }
        }

        // Add timestamp if enabled (microseconds since boot)
        if (include_timestamp && remaining > 20) {
            uint64_t timestamp_us = get_timestamp_us();
            written = snprintf(buffer_pos, remaining, "[%llu] ", timestamp_us);
            if (written > 0 && written < (int)remaining) {
                buffer_pos += written;
                remaining -= written;
            }
        }

        // Add the actual message
        if (remaining > 2) {
            written = vsnprintf(buffer_pos, remaining - 1, format, args);
            if (written > 0) {
                buffer_pos += (written < (int)(remaining - 1)) ? written : (remaining - 1);
            }
        }

        // Ensure null termination and add newline if not present
        size_t msg_len = buffer_pos - format_buffer;
        if (msg_len > 0 && msg_len < sizeof(format_buffer) - 1) {
            if (format_buffer[msg_len - 1] != '\n') {
                format_buffer[msg_len] = '\n';
                format_buffer[msg_len + 1] = '\0';
                msg_len++;
            }
        }

        // Output based on configuration
        if (output_mode & OUTPUT_PRINTF) {
            printf("%s", format_buffer);
            fflush(stdout);
        }

        #if ENABLE_FILE_LOGGING
        if ((output_mode & OUTPUT_FILE) && log_file) {
            UINT bytes_written;
            FIL* file_ptr = static_cast<FIL*>(log_file);
            FRESULT fr = f_write(file_ptr, format_buffer, msg_len, &bytes_written);
            if (fr == FR_OK) {
                f_sync(file_ptr); // Ensure data is written to SD card
            } else {
                printf("Failed to write to log file, error: %d\n", fr);
            }
        }
        #endif
    }

    // Public method implementations
    void Logger::set_level(LogLevel level) {
        current_log_level = level;
    }

    LogLevel Logger::get_level() {
        return current_log_level;
    }

    void Logger::set_output(LogOutput output) {
        output_mode = output;
    }

    void Logger::enable_timestamp(bool enable) {
        include_timestamp = enable;
    }

    void Logger::enable_level_prefix(bool enable) {
        include_level = enable;
    }

    void Logger::log(LogLevel level, const char* format, ...) {
        va_list args;
        va_start(args, format);
        log_core(level, format, args);
        va_end(args);
    }

    void Logger::log_array(LogLevel level, const char* message, uint16_t* array_ptr, uint16_t array_size) {
        if (level > current_log_level) return;
        if (!initialized) initialize();

        const size_t buf_max = sizeof(format_buffer);
        char* buffer_pos = format_buffer;
        size_t remaining = buf_max;

        // Add header info
        int written = 0;
        if (include_level) {
            written = snprintf(buffer_pos, remaining, "[%s] ", level_strings[level]);
            buffer_pos += written; remaining -= written;
        }

        if (include_timestamp) {
            uint64_t timestamp_us = get_timestamp_us();
            written = snprintf(buffer_pos, remaining, "[%llu] ", timestamp_us);
            buffer_pos += written; remaining -= written;
        }

        written = snprintf(buffer_pos, remaining, "%s [", message);
        buffer_pos += written; remaining -= written;

        // Array loop with buffer check
        for (int i = 0; i < array_size; ++i) {
            // Estimate max written for one array entry (e.g., "%u, " = max 7 chars for uint16_t + comma+space)
            if (remaining < 8) {
                // Flush buffer
                *buffer_pos = 0; // Null-terminate
                if (output_mode & OUTPUT_PRINTF) {
                    printf("%s", format_buffer);
                }
                #if ENABLE_FILE_LOGGING
                if ((output_mode & OUTPUT_FILE) && log_file) {
                    // File write
                    UINT bytes_written;
                    FIL* file_ptr = static_cast<FIL*>(log_file);
                    FRESULT fr = f_write(file_ptr, format_buffer, buffer_pos - format_buffer, &bytes_written);
                    if (fr == FR_OK) f_sync(file_ptr);
                }
                #endif

                // Reset buffer
                buffer_pos = format_buffer;
                remaining = buf_max;
            }
            written = snprintf(buffer_pos, remaining, "%u%s", array_ptr[i], (i < array_size - 1) ? "," : "");
            buffer_pos += written; remaining -= written;
        }

        // Finish array and flush remainder
        if (remaining > 2) {
            written = snprintf(buffer_pos, remaining, "]\n");
            buffer_pos += written; remaining -= written;
        }
        if (output_mode & OUTPUT_PRINTF) {
            printf("%s", format_buffer);
            fflush(stdout);
        }
        #if ENABLE_FILE_LOGGING
        if ((output_mode & OUTPUT_FILE) && log_file) {
            // File write
            UINT bytes_written;
            FIL* file_ptr = static_cast<FIL*>(log_file);
            FRESULT fr = f_write(file_ptr, format_buffer, buffer_pos - format_buffer, &bytes_written);
            if (fr == FR_OK) f_sync(file_ptr);
        }
        #endif
    }

    void Logger::shutdown() {
        #if ENABLE_FILE_LOGGING
        if (log_file) {
            f_close(static_cast<FIL*>(log_file));
            log_file = nullptr;
        }
        #endif
        initialized = false;
    }

    void initialize_module() {
        // init on first log
    }

    void deinitialize_module() {
        Logger::shutdown();
    }

    LogLevel get_level() {
        return Logger::get_level();
    }
}
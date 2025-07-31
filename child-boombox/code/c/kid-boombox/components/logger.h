#ifndef LOGGER_H
#define LOGGER_H

#define ENABLE_FILE_LOGGING true

#include <stdio.h>
#include <stdint.h>

namespace logger {
    enum LogLevel {
        LOG_LEVEL_ERROR = 0,
        LOG_LEVEL_WARN  = 1,
        LOG_LEVEL_INFO  = 2,
        LOG_LEVEL_DEBUG = 3,
        LOG_LEVEL_TRACE = 4
    };

    enum LogOutput {
        OUTPUT_PRINTF = 1,
        OUTPUT_FILE   = 2,
        OUTPUT_BOTH   = 3
    };

    class Logger {
        private:
            static LogLevel current_log_level;
            static LogOutput output_mode;
            static bool include_timestamp;
            static bool include_level;
            static volatile bool initialized;
            
            #if ENABLE_FILE_LOGGING
            static const uint32_t MAX_FILE_SIZE = 10 * 1024 * 1024; // 10 MB
            static const int MAX_LOG_FILES = 5;
            static const char* LOG_FILENAME;
            static void* log_file; // Using void* to avoid including ff.h in header
            #endif
            
            // Fixed-size buffer for memory efficiency
            static char format_buffer[256];
            static const char* level_strings[];

            static uint64_t get_timestamp_us();
            #if ENABLE_FILE_LOGGING
            static bool is_file_oversized();
            static bool rotate_log_files();
            static bool initialize_file();
            #endif
            
            static bool initialize();
            static void log_core(LogLevel level, const char* format, va_list args) 
                __attribute__((format(printf, 2, 0)));

        public:
            static void set_level(LogLevel level);
            static LogLevel get_level();
            static void set_output(LogOutput output);
            static void enable_timestamp(bool enable);
            static void enable_level_prefix(bool enable);
            static void shutdown();
            
            // Main logging function with printf-style format checking
            static void log(LogLevel level, const char* format, ...) 
                __attribute__((format(printf, 2, 3)));

            static void log_array(LogLevel level, const char* message, uint16_t *array_ptr, uint16_t array_size);
    };

    void initialize_module();
    void deinitialize_module();
    LogLevel get_level();
}

// Convenience macros for easy usage
#define LOG_ERROR(...) logger::Logger::log(logger::LOG_LEVEL_ERROR, __VA_ARGS__)
#define LOG_WARN(...)  logger::Logger::log(logger::LOG_LEVEL_WARN, __VA_ARGS__)
#define LOG_INFO(...)  logger::Logger::log(logger::LOG_LEVEL_INFO, __VA_ARGS__)
#define LOG_DEBUG(...) logger::Logger::log(logger::LOG_LEVEL_DEBUG, __VA_ARGS__)
#define LOG_TRACE(...) logger::Logger::log(logger::LOG_LEVEL_TRACE, __VA_ARGS__)

// Conditional compilation support
#ifdef DISABLE_LOGGING
    #undef LOG_ERROR
    #undef LOG_WARN
    #undef LOG_INFO
    #undef LOG_DEBUG
    #undef LOG_TRACE
    
    #define LOG_ERROR(...) do {} while(0)
    #define LOG_WARN(...)  do {} while(0)
    #define LOG_INFO(...)  do {} while(0)
    #define LOG_DEBUG(...) do {} while(0)
    #define LOG_TRACE(...) do {} while(0)
#endif

#endif // LOGGER_H
# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)
set(PICO_EXTRAS_PATH "${CMAKE_CURRENT_LIST_DIR}/lib/pico-extras")
include(${PICO_EXTRAS_PATH}/external/pico_extras_import.cmake)

project(kid-boombox C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1
add_executable(kid-boombox 
        components/barcode_scanner.cpp
        components/audio_player.cpp
        components/pico_flash_storage.cpp
        components/vsys_voltage_reader.cpp
        components/sd_handler.cpp  
        kid-boombox.cpp)

add_subdirectory(lib/no-OS-FatFS-SD-SDIO-SPI-RPi-Pico/src build)

#add_compile_definitions(USE_PRINTF USE_DBG_PRINTF) # Debug msg for no-OS-FatFS

pico_set_program_name(kid-boombox "kid-boombox")
pico_set_program_version(kid-boombox "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(kid-boombox 0)
pico_enable_stdio_usb(kid-boombox 1)

# Add the standard library to the build
target_link_libraries(kid-boombox
        pico_stdlib
        pico_multicore
        hardware_flash
        hardware_sync
        hardware_pio
        hardware_dma
        hardware_irq
        hardware_sleep
        hardware_watchdog
        hardware_adc
        pico_audio_i2s
        no-OS-FatFS-SD-SDIO-SPI-RPi-Pico)
        

# Add the standard include files to the build
target_include_directories(kid-boombox PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/components
)

target_compile_definitions(kid-boombox PRIVATE
    USE_AUDIO_I2S=1
    PICO_AUDIO_I2S_MONO_OUTPUT=0
    PICO_AUDIO_I2S_MONO_INPUT=0
    # PICO_AUDIO_I2S_DATA_PIN=28 # default: GPIO28 -> I2S DATA (DIN)
    # PICO_AUDIO_I2S_CLOCK_PIN_BASE=26 # default: GPIO26 -> I2S SCLK/MCLK/BCLK, GPIO27 -> I2S LRCLK/WS
    # PICO_AUDIO_I2S_CLOCK_PINS_SWAPPED=0
)

pico_add_extra_outputs(kid-boombox)


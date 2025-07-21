#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "components/barcode_scanner.h"
#include "components/audio_player.h"
#include "components/pico_flash_storage.h"
#include "hardware/pio.h"
#include "pico/sleep.h"
#include "tusb.h"
#include "pico/stdio_usb.h"
#include "hardware/watchdog.h"
#include "hardware/adc.h"


#define START_PLAYING_MSG 84
#define STOP_PLAYING_MSG 85
#define CHANGE_VOLUME_MSG 86

#define WAKE_GPIO 20
#define REPLAY_GPIO 22
#define VOL_GPIO 15

#define LED_GREEN_GPIO 14
#define LED_YELLOW_GPIO 13

#define SLEEP_TIMEOUT_US 60'000'000 // 1 minute

#define STATUS_LED_MS 100

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



enum AUDIO {
        INSERT_CARD = -1 ,
        SUCCESS = -2,
        HELLO = -3,
        BYE = -4
};

static audio::AUDIO_VOLUME SAVED_VOL = audio::VOL_100;

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

bool open_audio_file(int *file_id, char *filename) {
    if (CORE_1_STATE.REQUESTED_FILE_ID > 0) {
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
    bool success = audio::open_file(filename);
    if (success) {
        *file_id = CORE_1_STATE.REQUESTED_FILE_ID;
    }
    return success;
}

void cycle_volume() {
    audio::AUDIO_VOLUME old_vol = audio::cycle_volume();
    audio::AUDIO_VOLUME new_vol = audio::current_volume();
    printf("[Core#1] Change volume: 0x%04x (%d %%) -> %04x (%d %%)\n", old_vol, 100*old_vol/audio::VOL_100, new_vol, 100*new_vol/audio::VOL_100);
    blink_status_led(audio::is_current_volume_edge_volume() ? 2 : 1);
}


void core1_main() {
    bool data_available;
    uint32_t received_data;
    Core1Job job = IDLE;
    int file_id = 0;
    char file_pattern[50];
    char filename[50];
    uint32_t last_busy = time_us_32();

    uint16_t saved_volume = storage::read_saved_volume();
    if (saved_volume != 0) {
        SAVED_VOL =  static_cast<audio::AUDIO_VOLUME>(saved_volume);
    } else {
        SAVED_VOL = audio::VOL_100;
    }
    printf("Loaded volume: 0x%04x (%d %%)\n", SAVED_VOL, 100*SAVED_VOL/audio::VOL_100);
    audio::initialize_module(SAVED_VOL);

    CORE_1_STATE.REQUESTED_FILE_ID = HELLO;
    if (open_audio_file(&file_id, filename)) {
        audio::blocking_play_whole_file();
        printf("[Core#1] End hello msg\n");
    }
    
    CORE_1_STATE.REQUESTED_FILE_ID = 0;

    CORE_1_STATE.READY = true;
    
    while (!CORE_1_STATE.TERMINATE) {
        while (CORE_1_STATE.FULL_IDLE || CORE_1_STATE.DORMANT_READY) {
            sleep_us(SAMPLES_SLEEP_US);
            if (CORE_1_STATE.CORE_0_REQUEST_DORMANT && !CORE_1_STATE.DORMANT_READY) {
                CORE_1_STATE.REQUESTED_FILE_ID = BYE;
                if (open_audio_file(&file_id, filename)) {
                    audio::blocking_play_whole_file();
                    printf("[Core#1] End bye msg\n");
                }
                CORE_1_STATE.REQUESTED_FILE_ID = 0;
                audio::deinitialize_module();
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
                    if (!open_audio_file(&file_id, filename)) {
                        continue;
                    }
                    job = PLAYING;
                    printf("[Core#1] Change job to playing: %s\n", filename);
                } else if (received_data == STOP_PLAYING_MSG) {
                    // DO NOTHING
                } else if (received_data == CHANGE_VOLUME_MSG) {
                    cycle_volume();
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
                        audio::close_opened_file();
                        job = GATHERING;
                        printf("[Core#1] Change job to gathering - stop playing sound - collecting msg came\n");
                        break;
                    } else if (received_data == START_PLAYING_MSG) {
                        audio_i2s_set_enabled(false);
                        if (CORE_1_STATE.REQUESTED_FILE_ID == file_id) { // restart file
                            bool success = audio::rewind_to_audio_beginning();
                            if (!success) {
                                printf("[Core#1] Change job to idle - error - cannot seek to file start\n");
                                job = IDLE;
                                break;
                            }
                            printf("[Core#1] Restart playing: %s\n", filename);
                            blink_status_led(1);
                        } else { // change file
                            audio::close_opened_file();
                            if (!open_audio_file(&file_id, filename)) {
                                continue;
                            }
                            printf("[Core#1] Start playing: %s\n", filename);
                        }
                    }  else if (received_data == STOP_PLAYING_MSG) {
                        audio::close_opened_file();
                        job = IDLE;
                        printf("[Core#1] Change job to idle - stop playing msg came\n");
                        break;
                    } else if (received_data == CHANGE_VOLUME_MSG) {
                        cycle_volume();
                    } 
                }
                audio::play_result_t play_chunk_result = audio::play_next_chunk(false);
                last_busy = time_us_32();
                if (!play_chunk_result.next_chunk_available) {
                    job = IDLE;
                    printf("[Core#1] Change job to idle - last audio chunk played\n");
                    break;
                } else if (!play_chunk_result.buffer_processed) {
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
    gpio_set_irq_enabled(WAKE_GPIO, GPIO_IRQ_EDGE_RISE, false);
    barcode::deinitialize_module();
    while(!CORE_1_STATE.DORMANT_READY) {
        sleep_ms(5);
    }
    audio::AUDIO_VOLUME VOL = audio::current_volume();
    if (VOL != SAVED_VOL) {
        printf("Saving volume to flash: 0x%04x (%d %%)\n", VOL, 100*VOL/audio::VOL_100);
        storage::save_volume_to_flash(VOL);
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
    if (now_us - last_volume_ts_us > 250'000) { // 250 milisecond between volume change msg
        multicore_fifo_push_timeout_us(CHANGE_VOLUME_MSG, SAMPLES_SLEEP_US);
        last_volume_ts_us = now_us;
    }
}

uint32_t last_stop_ts_us = 0;

void stop_button_callback() {
    uint32_t now_us = time_us_32();
    if (now_us - last_stop_ts_us > 1'000'000) { // 1 second between replay msg
        multicore_fifo_push_timeout_us(STOP_PLAYING_MSG, SAMPLES_SLEEP_US);
        last_stop_ts_us = now_us;
    }
}

void gpio_irq_callback(uint gpio, uint32_t events) {
    if (gpio == REPLAY_GPIO) {
        replay_button_callback();
    } else if (gpio == VOL_GPIO) {
        volume_button_callback();
    } else if (gpio == WAKE_GPIO) {
        stop_button_callback();
    }
}

int main()
{
    stdio_init_all();
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
    gpio_set_irq_enabled(WAKE_GPIO, GPIO_IRQ_EDGE_RISE, true);
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
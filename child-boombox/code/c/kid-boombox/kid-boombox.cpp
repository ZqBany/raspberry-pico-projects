#include <stdio.h>
#include <string.h>
#include <limits.h>
#include <algorithm>
#include <cmath>
#include <vector>
#include "pico/stdlib.h"
#include "pico/multicore.h"

#define LED_BRIGHTNESS_LEVEL 7 // 7 
#define CONTROL_PULSE_DELAY_US 2
#define MEASUREMENT_CHARGE_TIME_US 10
#define UINT16_T_MAX UINT16_MAX
#define SAMPLES_SLEEP_US 50
#define START_COLLECTING_DATA_MSG 42
#define SCALE 1000

typedef enum {
    LOG_LEVEL_ERROR = 0,
    LOG_LEVEL_WARNING,
    LOG_LEVEL_INFO,
    LOG_LEVEL_DEBUG,
    LOG_LEVEL_TRACE
} LogLevel;

static LogLevel current_log_level = LOG_LEVEL_DEBUG;

const char *log_level_strings[] = {
    "ERROR",
    "WARN",
    "INFO",
    "DEBUG",
    "TRACE"
};

void log_message(LogLevel level, const char *message) {
    if (level > current_log_level) {
        return;
    }
    printf("[%s] %s\n", log_level_strings[level], message);
}

void log_message(LogLevel level, uint16_t value) {
    if (level > current_log_level) {
        return;
    }
    printf("[%s] %u\n", log_level_strings[level], (unsigned int) value);
}


class QTRDecoder {
    private:    
        uint8_t sensor_pin;
        uint8_t control_pin;
        uint32_t PIN_MASK;
        uint32_t last_led_off_us;
    
    public:
        QTRDecoder(uint8_t sensor_pin, uint8_t control_pin) 
            : sensor_pin(sensor_pin), 
              control_pin(control_pin),
              last_led_off_us(0) {}

        void init() {
            // Initialize sensor pin
            gpio_init(sensor_pin);
            gpio_set_dir(sensor_pin, GPIO_OUT);
            gpio_disable_pulls(sensor_pin);
            // gpio_pull_down(sensor_pin);
            gpio_put(sensor_pin, false);
            
            // Initialize control pin
            gpio_init(control_pin);
            gpio_set_dir(control_pin, GPIO_OUT);
            gpio_pull_down(control_pin);
            gpio_put(control_pin, true);
            sleep_ms(5);
            gpio_put(control_pin, false);                          // Default Low (LED turn off)
            sleep_ms(5);
        }
        
        void reset_led() {
            log_message(LOG_LEVEL_TRACE, "reset led");
            gpio_put(control_pin, false);
            sleep_us(1200);  // Note that turning the LEDs off with a >1 ms pulse and then back on resets them to full current.
        }
        
        void led_off() {
            log_message(LOG_LEVEL_TRACE, "led off");
            last_led_off_us = time_us_32();
            gpio_put(control_pin, false);
        }
        
        uint32_t led_on_level(uint8_t level) {
            if (gpio_get(control_pin)) {
                // We are turning on dimmable emitters that are already on. To avoid messing
                // up the dimming level, we have to turn the emitters off and back on. This
                // means the turn-off delay will happen even if wait = false was passed to
                // emittersOn(). (Driver min is 1 ms.)
                reset_led();
            }
            level = MAX(0, MIN(31, level));  // Lowest brightness at 31 level
            uint32_t status = save_and_disable_interrupts();
            gpio_put(control_pin, true);
            uint32_t led_on_start = time_us_32();
            if (level > 0) {
                for (uint8_t i = 0; i < level; i++) {
                    busy_wait_us(CONTROL_PULSE_DELAY_US);
                    gpio_put(control_pin, false);
                    busy_wait_us(CONTROL_PULSE_DELAY_US);
                    gpio_put(control_pin, true); // Leave high at end
                }
            }
            restore_interrupts(status);
            return led_on_start;
        }
        
        void led_on() {
            if (false && time_us_32() - last_led_off_us <= 900) {
                log_message(LOG_LEVEL_TRACE, "led on via pin");
                gpio_put(control_pin, 1);
            } else {
                log_message(LOG_LEVEL_TRACE, "led on via pwm dimm");
                uint32_t led_on_start = led_on_level(LED_BRIGHTNESS_LEVEL);
                // Make sure it's been at least 300 us since the emitter pin was first set
                // high before returning. (Driver min is 250 us.) Some time might have
                // already passed while we set the dimming level.
                while (time_us_32() - led_on_start < 300) {
                    busy_wait_us_32(10);
                }
            }
        }

        uint16_t read_sensor_raw() {
            uint32_t status = save_and_disable_interrupts();
            gpio_set_dir(sensor_pin, GPIO_OUT);
            gpio_put(sensor_pin, true);
            busy_wait_us(10);
            uint32_t start = time_us_32();
            gpio_set_dir(sensor_pin, GPIO_IN);
            
            while(gpio_get(sensor_pin) != 0) {
                tight_loop_contents();
            }
            restore_interrupts(status);
            uint16_t value = time_us_32() - start;
            gpio_set_dir(sensor_pin, GPIO_OUT);
            gpio_put(sensor_pin, false);
            log_message(LOG_LEVEL_TRACE, value);
            return value;
        }
};

class Window {
    private:
        uint8_t size;
        uint8_t oldest_idx;
        int newest_idx;
        bool is_full;
        uint16_t* buffer;
    public:
        Window(uint8_t size): size(size) {
            buffer = new uint16_t[size];
            oldest_idx = 0;
            newest_idx = -1;
        }

        int append(uint16_t item) 
        {
            int result = -1;
            if (filled()) {
                result = buffer[oldest_idx];
                oldest_idx = (oldest_idx + 1) % size;
            } else if (!empty() && (newest_idx + 1) % size == oldest_idx) {
                is_full = true;
                result = buffer[oldest_idx];
                oldest_idx = (oldest_idx + 1) % size;
            }
            newest_idx = (newest_idx + 1) % size;
            buffer[newest_idx] = item;
            return result;
        }

        uint16_t newest() const
        {
            return buffer[newest_idx];
        }

        uint16_t oldest() const
        {
            return buffer[oldest_idx];
        }

        bool filled() const {
            return is_full;
        }

        bool empty() const {
            return newest_idx == -1;
        }

        int maximum() const {
            if (empty()) {
                return -1;
            }
            uint16_t result = 0;
            uint16_t start_idx = oldest_idx;
            uint16_t end_idx = newest_idx;
            if (filled()) {
                start_idx = 0;
                end_idx = size - 1;
            }
            for (int idx = start_idx; idx <= end_idx; idx++) {
                uint16_t value = buffer[idx];
                if (value > result) {
                    result = value;
                }
            }
            return result;
        }

        int minimum() const {
            if (empty()) {
                return -1;
            }
            uint16_t result = UINT16_T_MAX;
            uint16_t start_idx = oldest_idx;
            uint16_t end_idx = newest_idx;
            if (filled()) {
                start_idx = 0;
                end_idx = size - 1;
            }
            for (int idx = start_idx; idx <= end_idx; idx++) {
                uint16_t value = buffer[idx];
                if (value < result) {
                    result = value;
                }
            }
            return result;
        }

        int median() const {
            if (empty()) {
                return -1;
            }
            uint16_t N = size;
            if (!filled()) {
                N = newest_idx;
            }
            uint16_t median_position = N / 2;
            // fast median: https://stackoverflow.com/a/33325864
            for (size_t i = 0; i < N; i++) {
                auto x = buffer[i];
                //count number of "less" and "equal" elements
                int cntLess = 0, cntEq = 0;
                for (size_t j = 0; j < N; j++) {
                    cntLess += buffer[j] < x;
                    cntEq += buffer[j] == x;
                }
                if ((unsigned int)(median_position - cntLess) < cntEq)
                    return x;
            }
            return -1; // should never happen
        }
};

enum Extremum {
    NONE, MINIMUM, MAXIMUM
};

struct IndexedExtremum {
    Extremum extremum;
    uint16_t index;
};

class DerivativeExtrema {
    private:
        uint8_t window_size = 21;
        Window window = Window(window_size);
        uint8_t noise_threshold;
        int32_t prev_dy = 0;
        uint8_t window_midpoint = (window_size - 1) / 2;
    public:
        DerivativeExtrema(uint8_t noise_threshold): noise_threshold(noise_threshold) {};

        IndexedExtremum add_point(uint16_t y, uint16_t current_index) {
            window.append(y);
            if (!window.filled()) {
                log_message(LOG_LEVEL_TRACE, "Window not filled");
                return {NONE, 0};
            }
            int32_t current_dy = static_cast<int32_t>(window.newest()) - static_cast<int32_t>(window.oldest());
            if (abs(current_dy) <= noise_threshold) {
                log_message(LOG_LEVEL_TRACE, abs(current_dy));
                return {NONE, 0};
            }
            Extremum extrema_detected = NONE;
            if (prev_dy != 0) {
                if (prev_dy > 0 && current_dy <= 0) {
                    extrema_detected = MAXIMUM;
                } else if (prev_dy < 0 && current_dy >= 0) {
                    extrema_detected = MINIMUM;
                }
            }

            if (extrema_detected != NONE) {
                uint16_t extremum_index = current_index - window_midpoint;
                log_message(LOG_LEVEL_TRACE, "extremum detected");
                prev_dy = 0;
                return {extrema_detected, extremum_index};
            } else {
                prev_dy = current_dy;
                return {NONE, 0};
            }
        }
};

int pico_led_init(void) {
    // A device like Pico that uses a GPIO for the LED will define PICO_DEFAULT_LED_PIN
    // so we can use normal GPIO functionality to turn the led on and off
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    return PICO_OK;
}

void pico_set_led(bool led_on) {
    gpio_put(PICO_DEFAULT_LED_PIN, led_on);
}

struct BarcodeDataGathererState {
    static constexpr uint32_t MAX_SAMPLES = 3000;
    uint16_t raw_data_buffer[MAX_SAMPLES];
    volatile uint16_t raw_data_idx = 0;
    volatile bool COLLECTING = false;
    volatile bool CORE_1_RUNNING = false;
    volatile bool TERMINATE_CORE_1 = false;
};

static BarcodeDataGathererState STATE;
    
static QTRDecoder qtr_decoder = QTRDecoder(9, 8);
    
void core1_barcode_raw_data_gatherer() {
    uint32_t received_data;
    while (!STATE.TERMINATE_CORE_1) {
        bool data_available = multicore_fifo_pop_timeout_us(10*SAMPLES_SLEEP_US, &received_data);
        if (data_available && received_data == START_COLLECTING_DATA_MSG) {
            STATE.CORE_1_RUNNING = true;
            while (STATE.COLLECTING && STATE.raw_data_idx < STATE.MAX_SAMPLES) {
                STATE.raw_data_buffer[STATE.raw_data_idx] = qtr_decoder.read_sensor_raw();
                STATE.raw_data_idx++;
                sleep_us(SAMPLES_SLEEP_US);
            }
            STATE.CORE_1_RUNNING = false;
        } else {
            sleep_us(SAMPLES_SLEEP_US/10);
        }
    }
}

struct IndexedValue {
    uint16_t index;
    uint16_t value;
};

IndexedValue slice_min(uint16_t data[], uint16_t from_idx, uint16_t to_idx) {
    uint16_t min = UINT16_T_MAX;
    uint16_t min_idx = 0;
    uint16_t value = 0;
    for (uint16_t i = from_idx; i < to_idx; i++) {
        value = data[i];
        if (value < min) {
            min_idx = i;
            min = value;
        }
    }
    return IndexedValue { min_idx, min };
}

IndexedValue data_min(uint16_t from_idx, uint16_t to_idx) {
    return slice_min(STATE.raw_data_buffer, from_idx, to_idx);
}

IndexedValue slice_max(uint16_t data[], uint16_t from_idx, uint16_t to_idx) {
    uint16_t max = 0;
    uint16_t max_idx = 0;
    uint16_t value = 0;
    for (uint16_t i = from_idx; i < to_idx; i++) {
        value = data[i];
        if (value > max) {
            max_idx = i;
            max = value;
        }
    }
    return IndexedValue { max_idx, max };
}

IndexedValue data_max(uint16_t from_idx, uint16_t to_idx) {
    return slice_max(STATE.raw_data_buffer, from_idx, to_idx);
}



struct SignalData {
    uint16_t *array_ptr;
    uint16_t size;
};

class BarcodeDataGatherer {
    private:
        uint8_t expected_bits = 10;
        int baseline = -1;

        bool baseline_undefined() {
            return baseline == -1;
        }

        void determine_baseline() {
            uint16_t min = UINT16_T_MAX;
            // qtr_decoder._led_on();
            for (int i = 0; i < 5; i++) {
                uint16_t value = qtr_decoder.read_sensor_raw();
                if (value >= 50 && value < min) {
                    min = value;
                }
                sleep_ms(10);
            }
            // qtr_decoder._led_off();
            baseline = min;
            log_message(LOG_LEVEL_DEBUG, baseline);
            char str[50];
            sprintf(str, "baseline determined %d", baseline);
            log_message(LOG_LEVEL_INFO, str);
        }

        void wait_for_card_not_present() {
            // qtr_decoder._reset_led();
            log_message(LOG_LEVEL_DEBUG, "waiting for card not present");
            uint8_t missing_reached_ctr = 0;
            uint16_t value = 0;
            while (true) {
                // qtr_decoder._led_on();
                value = qtr_decoder.read_sensor_raw();
                // qtr_decoder._led_off();
                if (value >= baseline*0.9) {
                    missing_reached_ctr += 1;
                    if (missing_reached_ctr >= 3) {
                        break;
                    }
                } else {
                    missing_reached_ctr = 0;
                }
                sleep_ms(30);
            }
        }

        uint16_t wait_for_card_present() {
            // qtr_decoder._reset_led();
            log_message(LOG_LEVEL_DEBUG, "waiting for card present");
            uint16_t value = 0;
            while (true) {
                // qtr_decoder._led_on();
                value = qtr_decoder.read_sensor_raw();
                //qtr_decoder._led_off();
                if (value < baseline * 0.8) {
                    return value;
                }
                sleep_us(SAMPLES_SLEEP_US);
            }
        }

        void wait_for_sample_ready(uint16_t raw_data_processed_idx) {
            while (STATE.raw_data_idx <= raw_data_processed_idx) {
                sleep_us(5);
            }
        }
    public:
        SignalData gather_raw_barcode_data() {
            qtr_decoder.led_on();
            STATE.raw_data_idx = 0;
            STATE.COLLECTING = true;
            if (baseline_undefined()) {
                determine_baseline();
            }
            wait_for_card_not_present();
            uint16_t break_value = wait_for_card_present();
            multicore_fifo_push_blocking(START_COLLECTING_DATA_MSG);

            // qtr_decoder._led_on();
            uint16_t raw_data_processed_idx = 0;
            uint16_t starting_idx = 0;
            uint16_t last_max_extremum = UINT16_T_MAX;
            uint16_t delayed_min_value = baseline;
            uint16_t value = 0;
            
            char str[50];
            sprintf(str, "gathering signal %d", break_value);
            log_message(LOG_LEVEL_INFO, str);
            DerivativeExtrema extrema_detector = DerivativeExtrema(2);
            uint8_t maxima_counter = 0;
            while (raw_data_processed_idx < STATE.MAX_SAMPLES) {
                wait_for_sample_ready(raw_data_processed_idx);
                value = STATE.raw_data_buffer[raw_data_processed_idx];
                IndexedExtremum extremum = extrema_detector.add_point(value, raw_data_processed_idx); // add median from last X points maybe 3/5/7?
                if (extremum.extremum == MAXIMUM) {
                    sprintf(str, "maxima detected %i", raw_data_processed_idx);
                    log_message(LOG_LEVEL_DEBUG, str);
                    last_max_extremum = raw_data_processed_idx;
                    maxima_counter += 1;
                }
                if (raw_data_processed_idx > 50 && value >= baseline * 0.8) {
                    // handle signal end
                }
                if (raw_data_processed_idx > 1 && starting_idx >= raw_data_processed_idx - 3 &&
                    (value <= STATE.raw_data_buffer[raw_data_processed_idx - 1] || 
                        value < data_min(MAX(0, raw_data_processed_idx - 3), raw_data_processed_idx - 1).value)) {
                            starting_idx = raw_data_processed_idx - 1; 
                } else if (starting_idx < raw_data_processed_idx - 5 && raw_data_processed_idx - 25 > starting_idx) {
                    delayed_min_value = MIN(delayed_min_value, STATE.raw_data_buffer[raw_data_processed_idx - 25]);
                }

                if (maxima_counter >= 9 && raw_data_processed_idx - last_max_extremum > 50 && value <= delayed_min_value) {
                    break;
                }

                raw_data_processed_idx += 1;

                if (raw_data_processed_idx >= STATE.MAX_SAMPLES) {
                    break;
                }
            }
            STATE.COLLECTING = false;
            qtr_decoder.led_off();
            log_message(LOG_LEVEL_DEBUG, "signal gathered");
            log_message(LOG_LEVEL_DEBUG, maxima_counter);

            if (raw_data_processed_idx < expected_bits * 10) {
                // handle too short data
            }

            uint16_t end_idx = raw_data_processed_idx - 1;
            uint16_t signal_min = data_min(starting_idx, last_max_extremum).value;
            for (uint16_t idx = last_max_extremum; idx < end_idx; idx++) {
                if (STATE.raw_data_buffer[idx] <= signal_min) {
                    end_idx = idx;
                    break;
                }
            }
            uint16_t signal_max = data_max(starting_idx, end_idx).value;
            for (uint16_t idx = 0; idx <= starting_idx; idx++) {
                if (STATE.raw_data_buffer[idx] <= signal_max) {
                    starting_idx = idx;
                    if (idx > 0 && STATE.raw_data_buffer[idx - 1] <= signal_max * 1.03) {
                        starting_idx = idx - 1;
                    }
                    break;
                }
            }
            uint16_t signal_data_size = end_idx + 1 - starting_idx;
            uint16_t (*signal_data) = &STATE.raw_data_buffer[starting_idx];
            if (signal_data_size < expected_bits * 10) {
                // handle too short data
            }
            return SignalData { signal_data, signal_data_size };
        };
};

struct Range{
    uint16_t start;
    uint16_t end;
};

bool compare_ranges(const Range &a, const Range &b)
{
    if (a.start == b.start) {
        return a.end < b.end;
    }
    return a.start < b.start;
}

bool compare_ranges_reversed(const Range &a, const Range &b)
{
    if (a.start == b.start) {
        return a.end > b.end;
    }
    return a.start > b.start;
}

class DataPreprocessor {
    private:
        Range verify_and_expand(uint16_t data[], uint16_t N, uint16_t idx, uint16_t look_ahead, uint16_t threshold) {
            uint16_t base = data[idx];
            uint16_t start = idx;
            uint16_t end = idx + look_ahead;

            for (uint16_t j = idx; j <= end; j++) {
                if (abs(data[j] - base) > threshold) {
                    return Range { 0, 0 }; // invalid plateau
                }
            }

            // expand left
            while (start > 0 && abs(data[start - 1] - base) <= threshold) {
                start -= 1;
            }

            // expand right
            while (end < N - 1 && abs(data[end + 1] - base) <= threshold) {
                end += 1;
            }

            return Range { start, end };
        }

        std::vector<Range> merge_plateaus(std::vector<Range> plateaus) {
            std::vector<Range> merged_plateaus;

            if (plateaus.empty()) {
                return merged_plateaus;
            }

            std::sort(plateaus.begin(), plateaus.end(), compare_ranges);

            merged_plateaus.push_back(plateaus[0]);

            for (uint16_t plat_idx = 1; plat_idx < plateaus.size(); plat_idx++) {
                Range last = merged_plateaus.back();
                Range current = plateaus[plat_idx];
                if (current.start <= last.end + 1) {
                    last.end = MAX(last.end, current.end);
                } else {
                    merged_plateaus.push_back(current);
                }
            }

            return merged_plateaus;
        }

        uint16_t downsample_plateaus_inplace(uint16_t data[], uint16_t N, std::vector<Range> plateaus, uint16_t target_width, float min_width_ratio) {
            uint16_t prev_end = 0;
            uint16_t plateau_reduction = 0;

            for (uint16_t plat_idx = 0; plat_idx < plateaus.size(); plat_idx++) {
                Range plat = plateaus[plat_idx];
                uint16_t plat_length = plat.end - plat.start + 1;
                if (plat_length > target_width) {
                    plateau_reduction += plat_length - target_width;
                }
            }

            uint16_t compensated_target_width = int((N - plateau_reduction) * min_width_ratio);
            plateau_reduction = 0;
            for (uint16_t plat_idx = 0; plat_idx < plateaus.size(); plat_idx++) {
                Range plat = plateaus[plat_idx];
                uint16_t plat_length = plat.end - plat.start + 1;
                if (plat_length > compensated_target_width) {
                    plateau_reduction += plat_length - compensated_target_width;
                }
            }

            compensated_target_width = int((N - plateau_reduction) * min_width_ratio);
            uint16_t filtered_data_idx = 0;
            for (uint16_t plat_idx = 0; plat_idx < plateaus.size(); plat_idx++) {
                Range plat = plateaus[plat_idx];
                // add non plateau data
                for (uint16_t idx = prev_end; idx < plat.start; idx++) {
                    if (filtered_data_idx != idx) {
                        data[filtered_data_idx] = data[idx];
                    }
                    filtered_data_idx++;
                }

                // process plateau
                uint16_t plateau_len = plat.end - plat.start + 1;
                if (plateau_len > compensated_target_width) {
                    // too long plateau to shrinken evenly
                    uint16_t step = (plateau_len - 1) / (compensated_target_width - 1);
                    for (uint16_t remapped_idx = 0; remapped_idx < compensated_target_width; remapped_idx++) {
                        data[filtered_data_idx] = data[plat.start + int(remapped_idx * step)];
                        filtered_data_idx++;
                    }
                } else {
                    // plateau unchanged
                    for (uint16_t idx = plat.start; idx <= plat.end; idx++) {
                        if (filtered_data_idx != idx) {
                            data[filtered_data_idx] = data[idx];
                        }
                        filtered_data_idx++;
                    }
                }
                prev_end = plat.end + 1;
            }

            // add remaining data
            for (uint16_t idx = prev_end; idx < N; idx++) {
                if (filtered_data_idx != idx) {
                    data[filtered_data_idx] = data[idx];
                }
                filtered_data_idx++;
            }

            return filtered_data_idx;
        }
        
        uint16_t filter_plateaus_inplace(uint16_t data[], uint16_t N, uint16_t threshold, float min_width_ratio = 0.1) {
            uint16_t look_ahead = MAX(1, int(N*min_width_ratio));
            std::vector<Range> plateaus;
            for (uint16_t idx = 0; idx < N - look_ahead; idx++) {
                if (abs(data[idx] - data[idx + look_ahead]) <= threshold) {
                    Range range = verify_and_expand(data, N, idx, look_ahead, threshold);
                    if (range.end - range.start + 1 >= look_ahead) {
                        plateaus.push_back(range);
                        idx = range.end; // Skip processed plateau
                    }
                }
            }
            merge_plateaus(plateaus);
            uint16_t filtered_N = downsample_plateaus_inplace(data, N, plateaus, look_ahead, min_width_ratio);
            return filtered_N;
        }

        void normalize_data_inplace(uint16_t data[], uint16_t N) {
            uint16_t min_val = slice_min(data, 0, N).value;
            log_message(LOG_LEVEL_TRACE, min_val);
            uint16_t max_val = slice_max(data, 0, N).value;
            log_message(LOG_LEVEL_TRACE, max_val);
            uint16_t denom = max_val - min_val;
            for (uint16_t idx = 0; idx < N; idx++) {
                data[idx] =  SCALE *(data[idx] - min_val) / denom;
            }
        }

        void median_filter_inplace(uint16_t data[], uint16_t N, uint16_t window_size=11) {
            if (window_size % 2 == 0) {
                window_size += 1;
            }
            uint16_t window_half = window_size / 2;
            Window window = Window(window_size);
            for (uint16_t idx = 0; idx < N; idx++) {
                uint16_t value = data[idx];
                if (idx == 0) {
                    for (uint16_t i = 0; i < window_half; i++) {
                        window.append(data[idx]); // extend window for first values with first value
                    }
                }
                window.append(data[idx]);
                if (idx - window_half - 1 >= 0) {
                    data[idx - window_half - 1] = window.median();
                }
            }
            for (uint16_t idx = N - window_half - 1; idx < N; idx++) {
                window.append(data[N-1]); // extend window for last values with last value
                data[idx] = window.median();
            }
        }

    public:
        uint16_t smoothen_inplace(uint16_t data[], uint16_t N) {
            normalize_data_inplace(data, N);
            log_message(LOG_LEVEL_DEBUG, "normalization completed");
            median_filter_inplace(data, N, 7);
            log_message(LOG_LEVEL_DEBUG, "median filter completed");
            uint16_t filtered_N = filter_plateaus_inplace(data, N, int(SCALE*0.02));
            log_message(LOG_LEVEL_DEBUG, "filter plateaus completed");
            return filtered_N;
        } 
};

class BarcodeDecoder {
    private:
        uint16_t expand_left(uint16_t data[], uint16_t N, uint16_t left, uint16_t window_size, uint16_t window_size_threshold, bool shrink_edge_plateaus) {
            uint16_t expanded_left = left;
            uint16_t last_changed = MAX(0, left - 1);
            for (uint16_t idx = MAX(0, left - 1); idx >= 0; idx--) {
                if (data[idx] > data[idx + 1]) {
                    if (data[idx] > data[idx + 1] + int(SCALE*0.025)) {
                        break;
                    }
                    last_changed = idx;
                    if (slice_min(data, MAX(0, idx - window_size), idx).value >= data[idx]) {
                        break;
                    }
                } else if (data[idx] < data[idx + 1]) {
                    last_changed = idx;
                }
                if (data[idx] < window_size_threshold && (last_changed - idx) > window_size) {
                    break;
                }
                expanded_left = idx;
            }

            if (shrink_edge_plateaus) {
                for (uint16_t idx = expanded_left; idx < left; idx++) {
                    if (data[idx + 1] > data[idx]) {
                        break;
                    }
                    expanded_left = idx;
                }
            }
            return expanded_left;
        }

        uint16_t expand_right(uint16_t data[], uint16_t N, uint16_t right, uint16_t window_size, uint16_t window_size_threshold, bool shrink_edge_plateaus) {
            uint16_t expanded_right = right;
            uint16_t last_changed = MIN(N - 1, right + 1);
            for (int idx = MIN(N - 1, right + 1); idx < N; idx++) {
                if (data[idx] > data[idx - 1]) {
                    if (data[idx] > data[idx - 1] + int(SCALE*0.025)) {
                        break;
                    }
                    last_changed = idx;
                    if (slice_min(data, idx + 1, MIN(idx + window_size, N)).value >= data[idx]) {
                        break;
                    }
                } else if (data[idx] < data[idx - 1]) {
                    last_changed = idx;
                }
                if (data[idx] < window_size_threshold && (idx - last_changed) > window_size) {
                    break;
                }
                expanded_right = idx;
            }

            if (shrink_edge_plateaus) {
                for (uint16_t idx = expanded_right; idx > right; idx--) {
                    if (data[idx - 1] > data[idx]) {
                        break;
                    }
                    expanded_right = idx;
                }
            }
            return expanded_right;
        }

        Range extend_bit_range(uint16_t data[], uint16_t N, uint16_t left, uint16_t right, uint16_t window_size_threshold, bool shrink_edge_plateaus) {
            uint16_t window_size = int(N*0.025);
            return Range {
                expand_left(data, N, left, window_size, window_size_threshold, shrink_edge_plateaus),
                expand_right(data, N, right, window_size, window_size_threshold, shrink_edge_plateaus)
            };
        }

        int is_index_locked(uint16_t index, std::vector<Range> locked_idx) {
            uint16_t locked_size = locked_idx.size();
            for (uint16_t locked_range_idx = 0; locked_range_idx < locked_size; locked_range_idx++) {
                Range locked_range = locked_idx[locked_range_idx];
                if (index >= locked_range.start && index <= locked_range.end) {
                    return locked_range.end;
                }
            }
            return -1;
        }

        std::vector<uint16_t> find_intersections(uint16_t data[], uint16_t N, uint16_t threshold, std::vector<Range> locked_idx) {
            uint16_t locked_size = locked_idx.size();
            std::vector<uint16_t> intersections;
            for (uint16_t idx = 1; idx < N; idx++) {
                int locked_end = is_index_locked(idx, locked_idx);
                if (locked_end != -1) {
                    idx = (uint16_t) locked_end;
                    log_message(LOG_LEVEL_DEBUG, idx);
                    continue;
                }
                if ((data[idx] >= threshold && data[idx-1] < threshold) ||
                        (data[idx] <= threshold && data[idx-1] > threshold)) {
                    intersections.push_back(idx);
                }
            }
            log_message(LOG_LEVEL_DEBUG, intersections.size());
            return intersections;
        }

        uint16_t bit_width(Range bit) {
            return bit.end - bit.start;
        }
        
        void adjust_boundaries(uint16_t data[], uint16_t N, std::vector<Range> bits) {
            int bits_n = sizeof(bits)/sizeof(Range);
            std::sort(bits.begin(), bits.end(), compare_ranges);

            for (int idx = 0; idx + 1 < bits_n; idx++) {
                Range previous = bits[idx];
                Range next = bits[idx + 1];
                if (previous.end > next.start) {
                    uint16_t diff = previous.end - next.start;
                    previous.end = previous.end - int(diff / 2);
                    next.start = previous.end + 1;
                } else if (next.start > previous.end) {
                    uint16_t diff = next.start - previous.end;
                    uint16_t min_surrounding_bit_width = MIN(bit_width(previous), bit_width(next));
                    if (diff > 1 && (diff < 0.2 * min_surrounding_bit_width || 
                        (diff < 0.8 * min_surrounding_bit_width && slice_min(data, previous.end, next.start).value == slice_max(data, previous.end, next.start).value))) {
                            previous.end = previous.end + int(diff / 2);
                            next.start = previous.end + 1;
                    }
                }
            }
        }

        uint16_t calc_min_bit_width(std::vector<Range> bits) {
            uint16_t min = UINT16_MAX;
            for (uint16_t idx = 0; idx < bits.size(); idx++) {
                uint16_t width = bit_width(bits[idx]);
                if (width < min) {
                    min = width;
                }
            }
            return min;
        }

        uint16_t calc_max_bit_width(std::vector<Range> bits) {
            uint16_t max = 0;
            for (uint16_t idx = 0; idx < bits.size(); idx++) {
                uint16_t width = bit_width(bits[idx]);
                if (width > max) {
                    max = width;
                }
            }
            return max;
        }

    public:
        std::vector<Range> find_bits(uint16_t data[], uint16_t N) {
            DerivativeExtrema extremum_finder = DerivativeExtrema(15);
           
            std::vector<IndexedExtremum> signal_extremums;
            std::vector<Range> bits_positions;
            for (uint16_t idx = 0; idx < N; idx++) {
                IndexedExtremum local = extremum_finder.add_point(data[idx], idx);
                if (local.extremum != NONE) {
                    signal_extremums.push_back(local);
                }
            }
            if (!signal_extremums.empty()) {
                for (int idx = signal_extremums.size() - 1; idx >= 0; idx--) {
                    IndexedExtremum extremum = signal_extremums.at(idx);
                    if (extremum.extremum == MAXIMUM && bits_positions.size() < 10) {
                        bits_positions.push_back(extend_bit_range(data, N, extremum.index, extremum.index, 3, true));
                    }
                }
            }
            // [TODO] DerivativeExtrema
            // -- find 10 last maximums
            // -- find minimumums between them and their middles
            // -- if found 9 -> try to mirror first edge and treat it as a bit?
            return bits_positions;
        }

        uint16_t decode_barcode(uint16_t data[], uint16_t N) {
            std::vector<Range> all_bits = find_bits(data, N);
            char str[50];
            sprintf(str, "found %d bits", all_bits.size());
            log_message(LOG_LEVEL_INFO, str);
            if (current_log_level == LOG_LEVEL_DEBUG) {
                char str[300] = "";
                for(int i = 0; i < all_bits.size(); i++) {
                    char temp[16];
                    sprintf(temp, "(%d, %d)", all_bits[i].start, all_bits[i].end);  // Assuming all_bits[i] is an integer
                    strcat(str, temp);
                    if (i < all_bits.size() - 1) {
                        strcat(str, ", ");
                    }
                }
                log_message(LOG_LEVEL_DEBUG, str);
            }
            uint16_t number = 0;
            char binary_number[] = "0000000000";
            for (uint16_t idx = 0; idx < all_bits.size(); idx++) {
                uint16_t bit_height_diff = slice_max(data, all_bits[idx].start, all_bits[idx].end).value - slice_min(data, all_bits[idx].start, all_bits[idx].end + 1).value;
                uint16_t bit = 0;
                if (bit_height_diff > int(SCALE*0.6)) {
                    bit = 1;
                }
                if (bit != 1 && (idx == 0 || idx == 1)) {
                    // handle first 2 bits always 1
                }
                if (bit == 1) {
                    binary_number[all_bits.size() - 1 - idx] = '1';
                }
                number = number*2 + bit;
            }
            log_message(LOG_LEVEL_INFO, binary_number);
            return number;
        }
};

int main()
{
    stdio_init_all();
    while (!stdio_usb_connected()) { tight_loop_contents(); } // wait for monitor to connect

    int rc = pico_led_init();
    hard_assert(rc == PICO_OK);
    qtr_decoder.init();
    sleep_ms(5);
    
    multicore_launch_core1(core1_barcode_raw_data_gatherer);
    pico_set_led(true);
    sleep_ms(5);

    BarcodeDataGatherer gatherer = BarcodeDataGatherer();
    DataPreprocessor preprocessor = DataPreprocessor();
    BarcodeDecoder barcode_decoder = BarcodeDecoder();
    log_message(LOG_LEVEL_INFO, "system initialized");

    while (true) {
        SignalData data = gatherer.gather_raw_barcode_data();
        log_message(LOG_LEVEL_DEBUG, "signal gathered");
        log_message(LOG_LEVEL_DEBUG, data.size);
        uint16_t smoothen_N = preprocessor.smoothen_inplace(data.array_ptr, data.size);
        log_message(LOG_LEVEL_DEBUG, "signal smoothened");
        for (int i = 0; i < smoothen_N; i++) {
            printf("%u, ", data.array_ptr[i]);
        }
        printf("\n");
        uint16_t value = barcode_decoder.decode_barcode(data.array_ptr, smoothen_N);
        log_message(LOG_LEVEL_DEBUG, "barcode decoded");
        log_message(LOG_LEVEL_INFO, value);
        qtr_decoder.led_off();
        sleep_ms(50);
    }
}
#include <stdint.h>
#include <limits.h>
#include <algorithm>
#include <cmath>
#include <vector>
#include "logger.h"
#include "barcode_scanner.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"

#define FUNC_TIMEOUT_US 1'000'000 // 1 second

namespace barcode {
    void log_data(const char* message, uint16_t *array_ptr, uint16_t array_size) {
        logger::Logger::log_array(logger::LOG_LEVEL_INFO, message , array_ptr, array_size);
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

            void deinit() {
                gpio_put(sensor_pin, false);
                gpio_disable_pulls(sensor_pin);
                gpio_set_dir(sensor_pin, GPIO_IN);
                gpio_put(control_pin, false);                          // Default Low (LED turn off)
                gpio_disable_pulls(control_pin);
                sleep_ms(5);
            }
            
            void reset_led() {
                LOG_TRACE("reset led");
                gpio_put(control_pin, false);
                sleep_us(1200);  // Note that turning the LEDs off with a >1 ms pulse and then back on resets them to full current.
            }
            
            void led_off() {
                LOG_TRACE("led off");
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
                    LOG_TRACE("led on via pin");
                    gpio_put(control_pin, 1);
                } else {
                    LOG_TRACE("led on via pwm dimm");
                    uint32_t led_on_start = led_on_level(LED_BRIGHTNESS_LEVEL);
                    // Make sure it's been at least 300 us since the emitter pin was first set
                    // high before returning. (Driver min is 250 us.) Some time might have
                    // already passed while we set the dimming level.
                    while (time_us_32() - led_on_start < 300) {
                        busy_wait_us_32(10);
                    }
                }
            }

            uint16_t __time_critical_func(read_sensor_raw)() { // Raspberry Pi Programmable I/O (PIO) state machines ?
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
                LOG_TRACE("%u", value);
                return value;
            }
    };

    static QTRDecoder qtr_decoder = QTRDecoder(9, 8);

    class Window {
        private:
            uint8_t size;
            uint8_t oldest_idx;
            int newest_idx;
            bool is_full;
            uint16_t* buffer;
            uint8_t window_midpoint;
            uint32_t running_sum = 0;
            double running_mean = 0.0;
            double M2 = 0.0;  // Sum of squared differences from mean

            void updateIncrementalStats(uint16_t new_value, uint16_t old_value, bool removing_old) {
                running_sum = running_sum + new_value;
                if (removing_old) {
                    running_sum = running_sum - old_value;
                }

                uint8_t buffer_size = (filled()) ? size : newest_idx + 1;
        
                if (removing_old) {
                    // Remove old value from statistics
                    double delta_old = old_value - running_mean;
                    running_mean = running_mean + (new_value - old_value) / buffer_size;
                    double delta_new = new_value - running_mean;
                    M2 = M2 - delta_old * (old_value - running_mean) + delta_new * (new_value - running_mean);
                } else {
                    // Add new value to statistics
                    double delta = new_value - running_mean;
                    running_mean += delta / running_sum;
                    double delta2 = new_value - running_mean;
                    M2 += delta * delta2;
                }
            }
        public:
            Window(uint8_t size_param) {
                if (size_param % 2 == 0) {
                    size = size_param + 1;
                } else {
                    size = size_param;
                }
                buffer = new uint16_t[size];
                oldest_idx = 0;
                newest_idx = -1;
                window_midpoint = size / 2;
                is_full = false;
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
                updateIncrementalStats(item, result, result != -1);
                return result;
            }

            void print() {
                uint8_t buffer_filled_size = (filled()) ? size : newest_idx + 1;
                for (uint8_t val_idx = 0; val_idx < buffer_filled_size; val_idx++) {
                    uint8_t index = (oldest_idx + val_idx) % size;
                    uint16_t value = buffer[index];
                    LOG_INFO("%u, ", value);
                }
                LOG_INFO("\n");
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

            uint8_t midpoint() const {
                if (filled()) {
                    return window_midpoint;
                } else {
                    return newest_idx / 2;
                }
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
                    N = newest_idx + 1;
                }
                uint16_t median_position = midpoint();
                
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

            double mean() const {
                uint8_t buffer_filled_size = (filled()) ? size : newest_idx + 1;
                return running_sum / buffer_filled_size;
            }

            double stddev(double mean) const {
                double sq_sum = 0.0;
                uint8_t buffer_filled_size = (filled()) ? size : newest_idx + 1;
                for (uint8_t val_idx = 0; val_idx < buffer_filled_size; val_idx++) {
                    uint8_t index = (oldest_idx + val_idx) % size;
                    double diff = buffer[index] - mean;
                    sq_sum += diff * diff;
                }
                return std::sqrt(sq_sum / buffer_filled_size);
            }

            double stddev_fast() const {
                uint8_t buffer_size = (filled()) ? size : newest_idx + 1;
                if (buffer_size <= 1) return 0.0;
                
                double variance = M2 / buffer_size;
                return std::sqrt(variance);
            }
    };

    enum Extremum {
        NONE, MINIMUM, MAXIMUM
    };

    struct IndexedExtremum {
        Extremum extremum;
        int index;
    };

    struct BarcodeDataGathererState {
        static constexpr uint32_t MAX_SAMPLES = 6'000;
        uint16_t raw_data_buffer[MAX_SAMPLES];
        volatile uint16_t raw_data_idx = 0;
        volatile bool COLLECTING = false;
        volatile bool RAW_GATHERER_RUNNING = false;
        volatile bool FIRST_MINIMUM_REACHED = false;
    };

    static BarcodeDataGathererState STATE;

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

    enum MODE {
        GATHER, DETECT
    };

    class WindowExtremumDetector {
        public:
            WindowExtremumDetector(MODE mode, uint16_t *data_ptr, uint8_t window_size = 51, double smooth_factor = 0.2, double noise_threshold = 0.1)
                : mode(mode),
                window_size(window_size),
                window(Window(window_size)),
                smooth_factor(smooth_factor),
                noise_threshold(noise_threshold),
                noise_level(0.0),
                all_time_min(UINT16_T_MAX),
                last_processed_idx(-1),
                half_window(window_size / 2),
                data_ptr(data_ptr) {}

            IndexedExtremum add_point(uint16_t y, uint16_t current_index) {
                window.append(y);
                // Track absolute minimum for reference
                if (mode == GATHER) {
                   if (y < all_time_min) {
                        all_time_min = y;
                    }
                }

                if (current_index < half_window) {
                    return { NONE, 0 };
                }

                const int window_middle = current_index - half_window;
                IndexedExtremum extremum = process_index(window_middle);
                last_processed_idx = window_middle;

                return extremum;
            }

            IndexedExtremum find_missed_extremum(IndexedExtremum local_extremum, uint16_t starting_idx, bool use_threshold = true) {
                uint16_t minimum_extremum_diff = 10;
                if (mode == DETECT) {
                    minimum_extremum_diff = 100;
                }
                if (local_extremum.extremum == MAXIMUM) {
                    IndexedValue minimum = slice_min(data_ptr, starting_idx, local_extremum.index);
                    if (minimum.value + minimum_extremum_diff < data_ptr[local_extremum.index] && (use_threshold && passes_minimum_threshold(minimum.value))) {
                        return { MINIMUM, minimum.index };
                    }
                } else {
                    IndexedValue maximum = slice_max(data_ptr, starting_idx, local_extremum.index);
                    if (data_ptr[local_extremum.index] + minimum_extremum_diff < maximum.value && (use_threshold && passes_maximum_threshold(maximum.value))) {
                        return { MAXIMUM, maximum.index };
                    }
                }
                return { NONE, 0 };
            }

        private:
            MODE mode;
            uint8_t window_size;
            uint8_t half_window;
            double smooth_factor, noise_threshold;
            double noise_level;
            uint16_t all_time_min;
            int last_processed_idx;
            Window window;
            uint16_t *data_ptr;

            // Process a single index for extremum detection
            IndexedExtremum process_index(int idx) {
                IndexedExtremum result{NONE, -1};

                int local_median = window.mean(); // maybe median would be better but signal is gut enough to approximate?
                double local_std = window.stddev_fast();

                // Exponential smoothing for noise estimate
                noise_level = 0.95 * noise_level + 0.05 * local_std;

                uint16_t current_val = data_ptr[idx];

                // Maximum detection
                if ((current_val == window.maximum()) &&
                    (current_val - local_median > noise_threshold * local_std) &&
                    passes_maximum_threshold(current_val)) {
                    result = {MAXIMUM, idx};
                }
                // Minimum detection
                else if ((current_val == window.minimum()) &&
                        (local_median - current_val > noise_threshold * local_std) &&
                        passes_minimum_threshold(current_val)) {
                    result = {MINIMUM, idx};
                }

                return result;
            }

            bool passes_maximum_threshold(uint16_t value) {
                if (mode == GATHER) {
                    return value > all_time_min * 1.2;
                }
                return value >= SCALE * 0.25;
            }


            bool passes_minimum_threshold(uint16_t value) {
                if (mode == GATHER) {
                    return value < all_time_min * 1.4;
                }
                return value < SCALE * 0.275;
            }
    };


    void __time_critical_func(gather_barcode_raw_data)() {
        Window median_window = Window(5);
        Window same_value_lengths = Window(7);
        STATE.RAW_GATHERER_RUNNING = true;
        uint16_t value, median, last_median;
        uint16_t same_value_median = 0, value_cnt = 0;
        uint16_t downsample_val, downsample_cnt = 0, downsample_step = 1;
        uint16_t minimal_downsample_step = 1;
        uint32_t last_median_change = time_us_32();
        bool initial_downsample_rdy = false;
        while (STATE.COLLECTING && STATE.raw_data_idx < STATE.MAX_SAMPLES) {
            value = qtr_decoder.read_sensor_raw();
            if (!initial_downsample_rdy && value < 100) {
                minimal_downsample_step = STATE.raw_data_idx / 7;
                initial_downsample_rdy = true;
            }
            median_window.append(value);
            median = median_window.median();
            if (median != last_median) {
                same_value_lengths.append(value_cnt);
                same_value_median = same_value_lengths.median();
                value_cnt = 0;
                last_median = median;
                last_median_change = time_us_32();
            } else {
                value_cnt += 1;
            }
            if (downsample_cnt == 0) {
                if (time_us_32() - last_median_change > 500'000) {
                    break;
                }
                STATE.raw_data_buffer[STATE.raw_data_idx] = value;
                STATE.raw_data_idx++;
                downsample_val = same_value_median;
                if (same_value_median < value_cnt) {
                    downsample_val = value_cnt;
                }
                
                downsample_step = MIN(MAX(minimal_downsample_step, downsample_val/3), 30); // value from 1 to 30
            }
            downsample_cnt = (downsample_cnt + 1) % downsample_step;
            sleep_us(SAMPLES_SLEEP_US);
        }
        STATE.RAW_GATHERER_RUNNING = false;
    }

    struct SignalData {
        bool try_again;
        uint16_t size;
        uint16_t *array_ptr;
    };

    class BarcodeDataGatherer {
        private:
            uint8_t expected_bits = 10;
            int baseline = -1;

            bool baseline_undefined() {
                return baseline == -1;
            }

            bool determine_baseline() {
                uint32_t start = time_us_32();
                uint16_t min = UINT16_T_MAX;
                // qtr_decoder._led_on();
                while (min > 500 || min < 200) {
                    min = UINT16_T_MAX;
                    for (int i = 0; i < 5; i++) {
                        uint16_t value = qtr_decoder.read_sensor_raw();
                        if (value >= 50 && value < min) {
                            min = value;
                        }
                        sleep_ms(10);
                    }
                    if (time_us_32() - start > FUNC_TIMEOUT_US) {
                        return true;
                    }
                    sleep_ms(100);
                }
                
                // qtr_decoder._led_off();
                baseline = min;
                LOG_DEBUG("%u", baseline);
                LOG_INFO("baseline determined %d", baseline);
                return false;
            }

            bool wait_for_card_not_present() {
                //qtr_decoder.reset_led();
                LOG_DEBUG("waiting for card not present");
                uint8_t missing_reached_ctr = 0;
                uint16_t value = 0;
                uint32_t start = time_us_32();
                while (true) {
                    //qtr_decoder.led_on();
                    value = qtr_decoder.read_sensor_raw();
                    if (value >= baseline*0.9) {
                        missing_reached_ctr += 1;
                        if (missing_reached_ctr >= 3) {
                            return false;
                        }
                    } else {
                        missing_reached_ctr = 0;
                        //qtr_decoder.led_off();
                    }
                    if (time_us_32() - start > FUNC_TIMEOUT_US) {
                        return true;
                    } 
                    sleep_ms(30);
                }
            }

            bool wait_for_card_present(uint16_t *break_value) {
                //qtr_decoder.reset_led();
                LOG_DEBUG("waiting for card present");
                uint16_t value = 0;
                uint32_t start = time_us_32();
                while (true) {
                    //qtr_decoder.led_on();
                    value = qtr_decoder.read_sensor_raw();
                    if (value < baseline * 0.8) {
                        *break_value = value;
                        return false;
                    }
                    //qtr_decoder.led_off();
                    if (time_us_32() - start > FUNC_TIMEOUT_US) {
                        return true;
                    }
                    sleep_us(SAMPLES_SLEEP_US);
                }
            }

            bool wait_for_sample_ready(uint16_t raw_data_processed_idx, uint32_t last_sample_ts) {
                while (STATE.raw_data_idx <= raw_data_processed_idx) {
                    if (time_us_32() - last_sample_ts > 100'000) {
                        return true;
                    }
                    sleep_us(5);
                }
                return false;
            }

            uint16_t detect_signal_end(uint16_t raw_data_processed_idx, uint16_t starting_idx, uint16_t last_max_extremum) {
                uint16_t end_idx = raw_data_processed_idx - 1;
                uint16_t signal_min = data_min(starting_idx, last_max_extremum).value;
                for (uint16_t idx = last_max_extremum; idx < end_idx; idx++) {
                    if (STATE.raw_data_buffer[idx] <= signal_min) {
                        end_idx = idx;
                        break;
                    }
                }
                return end_idx;
            }

            uint16_t detect_signal_start(uint16_t starting_idx, uint16_t end_idx) {
                uint16_t signal_max = data_max(starting_idx, end_idx).value;
                for (uint16_t idx = 0; idx <= starting_idx; idx++) {
                    int value = STATE.raw_data_buffer[idx];
                    if (value <= signal_max) {
                        return idx;
                    }
                }
                return starting_idx;
            }

            SignalData try_again() {
                STATE.COLLECTING = false;
                //qtr_decoder.led_off();
                return SignalData { true, 0, NULL };
            }

        public:
             bool initialize_baseline() {
                //qtr_decoder.led_on();
                if (baseline_undefined()) {
                    return determine_baseline();
                }
                return false;
                //qtr_decoder.reset_led();
            }

            bool wait_for_card_missing() {
                return wait_for_card_not_present();
            }

            bool wait_for_card() {
                uint16_t break_value = 0;
                if(wait_for_card_present(&break_value)) {
                    return true;
                }
                STATE.raw_data_idx = 0;
                STATE.COLLECTING = true;
                STATE.FIRST_MINIMUM_REACHED = false;
                multicore_fifo_push_blocking(START_COLLECTING_DATA_MSG);
                LOG_INFO("gathering signal %d", break_value);
                return false;
            }
            
            SignalData gather_raw_barcode_data() {
                // qtr_decoder._led_on();
                uint16_t raw_data_processed_idx = 0;
                uint16_t last_max_extremum = UINT16_T_MAX;
                uint16_t delayed_min_value = baseline;
                uint16_t value = 0;
                
                WindowExtremumDetector window_extremum_detector(GATHER, STATE.raw_data_buffer);
                WindowExtremumDetector short_window_extremum_detector(GATHER, STATE.raw_data_buffer, 25);
                uint8_t maxima_counter = 0;
                Window median_window = Window(5);
                IndexedExtremum first_minumum = IndexedExtremum { NONE, 0 };
                std::vector<IndexedExtremum> extrema_vec;
                uint32_t last_sample_ts = time_us_32();
                uint8_t timeout_cnt = 0;
                uint16_t current_delay = 25;
                while (raw_data_processed_idx < STATE.MAX_SAMPLES) {
                    bool timeouted = wait_for_sample_ready(raw_data_processed_idx, last_sample_ts);
                    if (!timeouted) {
                        current_delay = MAX(25, raw_data_processed_idx/12);
                        timeout_cnt = 0;
                        last_sample_ts = time_us_32();
                        value = STATE.raw_data_buffer[raw_data_processed_idx];
                        if (raw_data_processed_idx == 0) {
                            LOG_INFO("first core 1 value %d", value);
                        }
                        IndexedExtremum window_extremum = window_extremum_detector.add_point(value, raw_data_processed_idx);
                        if (extrema_vec.empty()) {
                            IndexedExtremum short_window_extremum = short_window_extremum_detector.add_point(value, raw_data_processed_idx);
                            if (short_window_extremum.extremum == MAXIMUM) {
                                LOG_DEBUG("AD short maxima detected %i", short_window_extremum.index);
                            } else if (short_window_extremum.extremum == MINIMUM) {
                                LOG_DEBUG("AD short minimum detected %i", short_window_extremum.index);
                            }
                            if (window_extremum.extremum == NONE && short_window_extremum.extremum != NONE && short_window_extremum.index > 0) {
                                extrema_vec.push_back(short_window_extremum);
                            }
                        }
                        if (window_extremum.extremum != NONE) {
                            if (window_extremum.extremum == MAXIMUM) {
                                last_max_extremum = window_extremum.index;
                                LOG_DEBUG("AD maxima detected %i", window_extremum.index);
                            } else {
                                LOG_DEBUG("AD minimum detected %i", window_extremum.index);
                            }
                            if (extrema_vec.empty()) {
                                if (window_extremum.index > 0) {
                                    extrema_vec.push_back(window_extremum);
                                }
                            } else {
                                IndexedExtremum last_extremum = extrema_vec.back();
                                if (last_extremum.extremum == window_extremum.extremum) {
                                    if (window_extremum.index - last_extremum.index >= 20) {
                                        IndexedExtremum missed_extremum = window_extremum_detector.find_missed_extremum(window_extremum, last_extremum.index + 1);
                                        if (missed_extremum.extremum != NONE) {
                                            extrema_vec.push_back(missed_extremum);
                                            extrema_vec.push_back(window_extremum);
                                        }
                                    }
                                    last_extremum = extrema_vec.back();
                                    if (last_extremum.index != window_extremum.index) {
                                        if (window_extremum.extremum == MAXIMUM && STATE.raw_data_buffer[window_extremum.index] > STATE.raw_data_buffer[last_extremum.index]) {
                                            extrema_vec.back() = window_extremum;
                                        } else if (window_extremum.extremum == MINIMUM && STATE.raw_data_buffer[window_extremum.index] < STATE.raw_data_buffer[last_extremum.index]) {
                                            extrema_vec.back() = window_extremum;
                                        }
                                    }
                                } else {
                                    extrema_vec.push_back(window_extremum);
                                }

                            }
                            if (first_minumum.extremum == NONE && window_extremum.extremum == MINIMUM) {
                                first_minumum = window_extremum;
                                STATE.FIRST_MINIMUM_REACHED = true;
                            }
                            uint8_t cnt = 0;
                            for (int i = 0; i < extrema_vec.size(); i++) {
                                if (extrema_vec[i].extremum == MAXIMUM) {
                                    cnt += 1;
                                }
                            }
                            maxima_counter = cnt;
                            LOG_DEBUG("%u", maxima_counter);
                        }
                        if (raw_data_processed_idx > 0 && value > baseline) {
                            log_data("Unexpected missing card", STATE.raw_data_buffer, raw_data_processed_idx);
                            return try_again();
                        }
                        
                        if (!timeouted && raw_data_processed_idx - current_delay > 0) {
                            delayed_min_value = MIN(delayed_min_value, STATE.raw_data_buffer[raw_data_processed_idx - current_delay]);
                        }
                    }

                    if (maxima_counter >= 10) {
                        if (raw_data_processed_idx - last_max_extremum > current_delay && value <= delayed_min_value) {
                            LOG_DEBUG("FOUND 10 MAXIMAS");
                            break;
                        }
                    } else if (maxima_counter == 9) {
                        if (raw_data_processed_idx - last_max_extremum > current_delay * 2 && value <= delayed_min_value) {
                            LOG_DEBUG("FOUND 9 MAXIMAS and 2 * delay reached");
                            break;
                        }
                    } else if (maxima_counter == 8) {
                        if (raw_data_processed_idx - last_max_extremum > current_delay * 4 && value <= delayed_min_value) {
                            LOG_DEBUG("FOUND 8 MAXIMAS and 4 * delay reached");
                            break;
                        }
                    }

                    if (!timeouted) {
                        raw_data_processed_idx += 1;
                        if (raw_data_processed_idx >= STATE.MAX_SAMPLES) {
                            log_data("Max samples reached", STATE.raw_data_buffer, STATE.MAX_SAMPLES);
                            return try_again();
                        }
                    } else if (time_us_32() - last_sample_ts > 2'000'000) {
                        log_data("Timeout reached", STATE.raw_data_buffer, raw_data_processed_idx);
                        return try_again();
                    }

                    if (!STATE.RAW_GATHERER_RUNNING) {
                        log_data("Core1 timeout reached", STATE.raw_data_buffer, raw_data_processed_idx);
                        return try_again();
                    }
                }
                STATE.COLLECTING = false;
                //qtr_decoder.led_off();
                LOG_INFO("signal gathered");
                LOG_DEBUG("%u", maxima_counter);

                LOG_DEBUG("EXTREMORUM");
                for (int i = 0; i < extrema_vec.size(); i++) {
                    IndexedExtremum extremum = extrema_vec.at(i);
                    if (extremum.extremum == MAXIMUM) {
                        LOG_DEBUG("AD maxima detected %i", extremum.index);
                    } else {
                        LOG_DEBUG("AD minimum detected %i", extremum.index);
                    }
                }

                if (raw_data_processed_idx < expected_bits * 10) {
                    log_data("Not enough samples 1 reached", STATE.raw_data_buffer, raw_data_processed_idx);
                    return try_again();
                }

                uint16_t end_idx = detect_signal_end(raw_data_processed_idx, first_minumum.index, last_max_extremum);
                uint16_t starting_idx = detect_signal_start(first_minumum.index, end_idx);

                if (logger::get_level() > logger::LOG_LEVEL_INFO) {
                    for (int i = 0; i < starting_idx; i++) {
                        LOG_INFO("%u, ", STATE.raw_data_buffer[i]);
                    }
                    LOG_INFO("\n");
                    LOG_INFO("BUKAAAAAAA 1111\n");

                    for (int i = starting_idx; i < end_idx; i++) {
                        LOG_INFO("%u, ", STATE.raw_data_buffer[i]);
                    }
                    LOG_INFO("\n");
                    LOG_INFO("BUKAAAAAAA 2222\n");
                }


                uint16_t signal_data_size = end_idx + 1 - starting_idx;
                uint16_t (*signal_data) = &STATE.raw_data_buffer[starting_idx];
                if (signal_data_size < expected_bits * 10) {
                    log_data("Not enough samples 1 reached", STATE.raw_data_buffer, end_idx);
                    return try_again();
                }
                return SignalData { false, signal_data_size, signal_data };
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

            uint16_t normalize_data_inplace(uint16_t data[], uint16_t N) {
                uint16_t min_val = slice_min(data, 0, N).value;
                LOG_TRACE("%u", min_val);
                uint16_t max_val = slice_max(data, 0, N).value;
                LOG_TRACE("%u", max_val);
                uint16_t denom = max_val - min_val;
                for (uint16_t idx = 0; idx < N; idx++) {
                    data[idx] =  SCALE *(data[idx] - min_val) / denom;
                }
                return SCALE / denom;
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
            struct SmoothenResult {
                uint16_t N;
                uint16_t minimal_step;
            };

            SmoothenResult smoothen_inplace(uint16_t data[], uint16_t N) {
                uint16_t minimal_step = normalize_data_inplace(data, N);
                LOG_DEBUG("normalization completed");
                median_filter_inplace(data, N, 7);
                LOG_DEBUG("median filter completed");
                // uint16_t filtered_N = filter_plateaus_inplace(data, N, int(SCALE*0.02));
                // log_message(LOG_LEVEL_DEBUG, "filter plateaus completed");
                return SmoothenResult { N, minimal_step };
            } 
    };

    class BarcodeDecoder {
        private:
            uint16_t expand_left(uint16_t data[], uint16_t N, uint16_t left, uint16_t window_size, uint16_t window_size_threshold, bool shrink_edge_plateaus) {
                uint16_t expanded_left = left;
                uint16_t last_changed = MAX(0, left - 1);
                for (int idx = MAX(0, left - 1); idx >= 0; idx--) {
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
                    for (int idx = expanded_left; idx < left; idx++) {
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
                    for (int idx = expanded_right; idx > right; idx--) {
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
                        LOG_DEBUG("%u", idx);
                        continue;
                    }
                    if ((data[idx] >= threshold && data[idx-1] < threshold) ||
                            (data[idx] <= threshold && data[idx-1] > threshold)) {
                        intersections.push_back(idx);
                    }
                }
                LOG_DEBUG("%u", intersections.size());
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

            static bool indexed_extremum_comp(IndexedExtremum a, IndexedExtremum b) {
                return a.index < b.index;
            }

        public:
            std::vector<Range> find_bits(uint16_t data[], DataPreprocessor::SmoothenResult smoothen_result) {
                uint16_t window_size = MIN(smoothen_result.N / 20, 51);
                WindowExtremumDetector extremum_finder = WindowExtremumDetector(DETECT, data, window_size);
                LOG_INFO("extremum window size %i", window_size);
                std::vector<IndexedExtremum> signal_extremums;
                std::vector<Range> bits_positions;
                uint16_t max_minimum_value = 0;
                for (uint16_t idx = 0; idx < smoothen_result.N; idx++) {
                    IndexedExtremum local_extremum = extremum_finder.add_point(data[idx], idx);
                    if (local_extremum.extremum != NONE) {
                        if (signal_extremums.empty()) {
                            if (local_extremum.index > 1 && data[local_extremum.index] < data[0]) {
                                IndexedExtremum missed_extremum = extremum_finder.find_missed_extremum(local_extremum, 0, false);
                                if (missed_extremum.extremum != NONE && missed_extremum.index > 1) {
                                    LOG_DEBUG("missed extremum detected %i", missed_extremum.index);
                                    signal_extremums.push_back(missed_extremum);
                                }
                                signal_extremums.push_back(local_extremum);
                            }
                        } else {
                            if (local_extremum.extremum == MINIMUM && max_minimum_value < data[local_extremum.index]) {
                                max_minimum_value = data[local_extremum.index];
                            }
                            IndexedExtremum last_extremum = signal_extremums.back();
                            if (last_extremum.extremum == local_extremum.extremum) {
                                if (local_extremum.index - last_extremum.index >= 20) {
                                    IndexedExtremum missed_extremum = extremum_finder.find_missed_extremum(local_extremum, last_extremum.index + 1);
                                    if (missed_extremum.extremum != NONE) {
                                        signal_extremums.push_back(missed_extremum);
                                        signal_extremums.push_back(local_extremum);
                                    }
                                }
                                last_extremum = signal_extremums.back();
                                if (last_extremum.index != local_extremum.index) {
                                    if (local_extremum.extremum == MAXIMUM && data[local_extremum.index] > STATE.raw_data_buffer[last_extremum.index]) {
                                        signal_extremums.back() = local_extremum;
                                    } else if (local_extremum.extremum == MINIMUM && data[local_extremum.index] < STATE.raw_data_buffer[last_extremum.index]) {
                                        signal_extremums.back() = local_extremum;
                                    }
                                }
                            } else {
                                signal_extremums.push_back(local_extremum);
                            }

                        }
                    }
                }

                if (!signal_extremums.empty()) {
                    LOG_INFO("Max minimum value %d", max_minimum_value);
                    std::vector<Range> minimas_positions;
                    int minimum_start = -1;
                    for (uint16_t idx = 0; idx < smoothen_result.N; idx++) {
                        if (data[idx] <= max_minimum_value) {
                            if (minimum_start == -1) {
                                minimum_start = idx;
                            }
                        } else {
                            if (minimum_start != -1) {
                                minimas_positions.push_back(Range {(uint16_t) minimum_start, idx} );
                                minimum_start = -1;
                            }
                        }
                    }
                    if (minimum_start != -1) {
                        minimas_positions.push_back(Range {(uint16_t) minimum_start, smoothen_result.N } );
                    }
                    LOG_INFO("Found %d minimas", minimas_positions.size());
                    if (minimas_positions.size() > 1) {
                        uint16_t last_minimum_end = minimas_positions.at(0).end;
                        for(int idx = 1; idx < minimas_positions.size(); idx++) {
                            Range current_min = minimas_positions.at(idx);
                            bool found_max_between = false;
                            bool found_current_min_in_extremums = false;
                            for (int idx = 0; idx < signal_extremums.size(); idx++) {
                                IndexedExtremum extremum = signal_extremums.at(idx);
                                if (!found_max_between && extremum.extremum == MAXIMUM) {
                                    if (extremum.index > last_minimum_end && extremum.index < current_min.start) {
                                        found_max_between = true;
                                    }
                                } else if (!found_current_min_in_extremums && extremum.extremum == MINIMUM) {
                                    if (extremum.index >= current_min.start && extremum.index < current_min.end) {
                                        found_current_min_in_extremums = true;
                                    }
                                }
                            }
                            if (!found_max_between) {
                                IndexedValue max = slice_max(data, last_minimum_end, current_min.start);
                                if (max.value > max_minimum_value * 2) {
                                    LOG_DEBUG("AD#2 maxima detected %i", max.index);
                                    signal_extremums.push_back( IndexedExtremum { MAXIMUM, max.index } );
                                }
                            }
                            if (!found_current_min_in_extremums) {
                                IndexedValue min = slice_min(data, current_min.start, current_min.end);
                                if (min.value <= max_minimum_value) {
                                    LOG_DEBUG("AD#2 minima detected %i", min.index);
                                    signal_extremums.push_back( IndexedExtremum { MINIMUM, min.index } );
                                }
                            }
                            last_minimum_end = current_min.end;
                        }
                    }
                    std::sort(signal_extremums.begin(), signal_extremums.end(), indexed_extremum_comp);
                    for (int idx = signal_extremums.size() - 1; idx >= 0; idx--) {
                        IndexedExtremum extremum = signal_extremums.at(idx);
                        if (extremum.extremum == MAXIMUM) {
                            LOG_DEBUG("AD maxima detected %i", extremum.index);
                        } else {
                            LOG_DEBUG("AD minimum detected %i", extremum.index);
                        }
                    }
                    for (int idx = signal_extremums.size() - 1; idx >= 0; idx--) {
                        IndexedExtremum extremum = signal_extremums.at(idx);
                        if (extremum.extremum == MAXIMUM) {
                            if (idx < signal_extremums.size() - 1 && idx > 0) {
                                IndexedExtremum prev_extremum = signal_extremums.at(idx + 1);
                                IndexedExtremum next_extremum = signal_extremums.at(idx - 1);
                                if (prev_extremum.extremum == MINIMUM && next_extremum.extremum == MINIMUM) {
                                    bits_positions.push_back(extend_bit_range(data, smoothen_result.N, MAX(0, extremum.index - 5), MIN(extremum.index + 5, smoothen_result.N - 1), 3, true));
                                }
                            } else if (idx == signal_extremums.size() - 1 && idx > 0) {
                                IndexedExtremum next_extremum = signal_extremums.at(idx - 1);
                                if (next_extremum.extremum == MINIMUM) {
                                    bits_positions.push_back(extend_bit_range(data, smoothen_result.N, MAX(0, extremum.index - 5), MIN(extremum.index + 5, smoothen_result.N - 1), 3, true));
                                }
                            } else if (idx == 0 && idx < signal_extremums.size() - 1) {
                                IndexedExtremum prev_extremum = signal_extremums.at(idx + 1);
                                if (prev_extremum.extremum == MINIMUM) {
                                    bits_positions.push_back(extend_bit_range(data, smoothen_result.N, MAX(0, extremum.index - 5), MIN(extremum.index + 5, smoothen_result.N - 1), 3, true));
                                }
                            }
                            
                        }
                    }
                }
                adjust_boundaries(data, smoothen_result.N, bits_positions);
                if (bits_positions.size() == 9) {
                    IndexedExtremum extremum = signal_extremums.at(0);
                    if (extremum.extremum == MINIMUM) {
                        // handle malformed first bit?
                    }
                    return bits_positions;
                }
                adjust_boundaries(data, smoothen_result.N, bits_positions);

                return bits_positions;
            }

            int decode_barcode(uint16_t data[], DataPreprocessor::SmoothenResult smoothen_result) {
                std::vector<Range> all_bits = find_bits(data, smoothen_result);
                LOG_INFO("found %d bits", all_bits.size());
                if (logger::get_level() >= logger::LOG_LEVEL_DEBUG) {
                    char str[300] = "";
                    for(int i = 0; i < all_bits.size(); i++) {
                        LOG_DEBUG("(%d, %d), ", all_bits[i].start, all_bits[i].end);  // Assuming all_bits[i] is an integer
                    }
                    LOG_DEBUG("\n");
                }
                if (all_bits.size() != 10) {
                    return -1;
                }
                uint16_t number = 0;
                char binary_number[] = "0000000000";
                for (uint16_t idx = 0; idx < all_bits.size(); idx++) {
                    uint16_t bit_height_diff = slice_max(data, all_bits[idx].start, all_bits[idx].end).value - slice_min(data, all_bits[idx].start, all_bits[idx].end + 1).value;
                    uint16_t bit = 0;
                    if (bit_height_diff <= int(SCALE*0.5)) {
                        bit = 1;
                    }
                    if (bit != 0 && (idx == 0 || idx == 1)) {
                        return -1;
                    }
                    if (bit == 1) {
                        binary_number[idx] = '1';
                    }
                    number = number*2 + bit;
                }
                LOG_INFO(binary_number);

                return number;
            }
    };

    static BarcodeDataGatherer gatherer = BarcodeDataGatherer();
    static DataPreprocessor preprocessor = DataPreprocessor();
    static BarcodeDecoder barcode_decoder = BarcodeDecoder();
        
    void initialize_module() {
        qtr_decoder.init();
        qtr_decoder.led_on();
        sleep_ms(50);
    }

    void deinitialize_module() {
        qtr_decoder.deinit();
    }

    bool initialize_baseline() {
        return gatherer.initialize_baseline();
    }

    bool wait_for_missing_card() {
        return gatherer.wait_for_card_missing();
    }

    bool wait_for_card() {
        return gatherer.wait_for_card();
    }

    int read_barcode() {
        uint32_t start = time_us_32();
        SignalData data = gatherer.gather_raw_barcode_data();
        uint32_t gathered = time_us_32();
        if (data.try_again) {
            LOG_DEBUG("please try again");
        } else {
            LOG_DEBUG("signal gathered");
            LOG_DEBUG("%u", data.size);
            DataPreprocessor::SmoothenResult smoothen_result = preprocessor.smoothen_inplace(data.array_ptr, data.size);
            uint32_t smoothen =  time_us_32();
            LOG_DEBUG("signal smoothened");
            if (logger::get_level() > logger::LOG_LEVEL_INFO) {
                for (int i = 0; i < smoothen_result.N; i++) {
                    LOG_DEBUG("%u, ", data.array_ptr[i]);
                }
                LOG_DEBUG("\n");
            }
            int value = barcode_decoder.decode_barcode(data.array_ptr, smoothen_result);
            uint32_t decoded = time_us_32();
            LOG_INFO("%u", (gathered - start) / 1'000);
            LOG_INFO("%u", (smoothen - gathered) / 1'000);
            LOG_INFO("%u", (decoded - smoothen)  / 1'000);
            LOG_DEBUG("barcode decoded");
            LOG_INFO("%u", value);
            //qtr_decoder.led_off();
            if (value == -1) {
                LOG_INFO("Cannot decode signal to barcode");
                log_data("Cannot decode signal to barcode", data.array_ptr, smoothen_result.N);
            }
            return value;
        }
        return -1;
    }

}
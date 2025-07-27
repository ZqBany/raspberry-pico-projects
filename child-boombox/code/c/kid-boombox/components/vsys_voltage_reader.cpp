#include "vsys_voltage_reader.h"
#include "pico/stdlib.h"
#include "hardware/adc.h"

namespace voltage_reader
{
    class VsysVoltageReader {
        private:
            const float conversion_factor = 3 * 3.3f / (1 << 12); // 3.3V reference / 12-bit range,  Pico divides VSYS by 3 for the ADC for protection reasons
        public:
            VsysVoltageReader() {}

            void init() {
                adc_init();
                sleep_ms(5);
                adc_gpio_init(29);
                sleep_ms(5);
                adc_select_input(3); // Select ADC input 3 (GPIO29)
                sleep_ms(5);
            }

            float vsys_voltage() {
                return adc_read() * conversion_factor;
            }
    };

    static VsysVoltageReader vsys_voltage_reader = VsysVoltageReader();

    void initialize_module() {
        vsys_voltage_reader.init();
    }

    float vsys_voltage() {
        return vsys_voltage_reader.vsys_voltage();
    }

    float low_battery() {
        return critically_low_battery() + 0.1f;
    }

    float critically_low_battery() {
        if (battery == SINGLE_USE) {
            return single_use_low_voltage;
        }
        return rechargable_low_voltage;
    }

    bool is_critically_low_battery() {
        return vsys_voltage() < critically_low_battery();
    }
}
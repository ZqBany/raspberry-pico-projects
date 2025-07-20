#ifndef SCANNER_H
#define SCANNER_H

#define LED_BRIGHTNESS_LEVEL 7 // 7 
#define CONTROL_PULSE_DELAY_US 2
#define MEASUREMENT_CHARGE_TIME_US 10
#define UINT16_T_MAX UINT16_MAX
#define SAMPLES_SLEEP_US 50
#define START_COLLECTING_DATA_MSG 42
#define SCALE 1'000

namespace barcode
{
    void initialize_module();
    void deinitialize_module();
    void initialize_baseline();
    bool wait_for_missing_card();
    bool wait_for_card();
    int read_barcode();
    void gather_barcode_raw_data();
}

#endif // SCANNER_H
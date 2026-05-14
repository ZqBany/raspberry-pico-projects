#ifndef VSYS_VOLTAGE_READER_H
#define VSYS_VOLTAGE_READER_H

namespace voltage_reader
{
    enum BatteryType {
        SINGLE_USE,
        RECHARGABLE
    };

    const BatteryType battery = SINGLE_USE;
    const float single_use_low_voltage = 1.23f * 3; // testing in progress
    const float rechargable_low_voltage = 1.0f * 3; // not tested

    void initialize_module();
    float vsys_voltage();
    float low_battery();
    float critically_low_battery(); // voltage that will cause device to not wake up but immediately shutdown without wake up handling
    bool is_critically_low_battery(); 
}

#endif // VSYS_VOLTAGE_READER_H
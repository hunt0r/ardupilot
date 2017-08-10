/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

#include <AP_HAL/utility/OwnPtr.h>
#include <AP_HAL/I2CDevice.h>
#include <utility>

class HGM_ThreePresSnsr
{
public:
    // constructor
    HGM_ThreePresSnsr();

    // set initial values, open hal i2c device, bind _timer to timed scheduler
    bool init(void);

    // return the current pressure in Pa
    float get_pressure_RM(void);
    float get_pressure_UD(void);
    float get_pressure_LR(void);

    // return the current temperature in degC
    float get_temperature_RM(void);
    float get_temperature_UD(void);
    float get_temperature_LR(void);

    // return true if ThreePresSnsr is enabled
    bool    enabled(void) const {
        return _enable;
    }

    // // force disable the sensor
    // void    disable(void) {
    //     _enable.set(0);
    // }

    // return health status of sensor
    bool healthy(void) const { return _healthy && _enable; }

    AP_Int8         _enable; // HGM: Not sure why this needs to be public... but compile error if private

    static const struct AP_Param::GroupInfo var_info[];

private:
    void _measure();
    void _collect();
    void _timer();

    float           _pressure_RM;
    float           _pressure_UD;
    float           _pressure_LR;
    float           _temperature_RM;
    float           _temperature_UD;
    float           _temperature_LR;
    bool		    _healthy:1;
    uint32_t _last_sample_time_ms;
    uint32_t _measurement_started_ms;
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev_snsrRM;
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev_snsrUD;
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev_snsrLR;
};

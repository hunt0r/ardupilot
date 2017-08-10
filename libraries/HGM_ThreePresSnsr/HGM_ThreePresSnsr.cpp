/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
 *   HGM_ThreePresSnsr.cpp - early five-hole-probe driver, basically
 *    just reads and logs 3 pressure sensors via I2C, with no
 *    concept of their functionality as a five-hole probe.
 *   2017-07-20
 */
#include "HGM_ThreePresSnsr.h"

#include <AP_ADC/AP_ADC.h>
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include <utility>

#include <stdio.h>

extern const AP_HAL::HAL &hal;

#define RM_I2C_ADDR 0x46
#define UD_I2C_ADDR 0x47
#define LR_I2C_ADDR 0x48

// use HAL_AIRSPEED_MS4515DO_I2C_BUS if defined, else use bus 1
#ifdef HAL_AIRSPEED_MS4515DO_I2C_BUS
#define MS4525D0_I2C_BUS HAL_AIRSPEED_MS4515DO_I2C_BUS
#else
#define MS4525D0_I2C_BUS 1
#endif

// the virtual pin for digital airspeed sensors
#define AP_AIRSPEED_I2C_PIN 65

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
 #define ARSPD_DEFAULT_PIN 1
#elif CONFIG_HAL_BOARD == HAL_BOARD_PX4  || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
 #include <sys/stat.h>
 #include <sys/types.h>
 #include <fcntl.h>
 #include <unistd.h>
 #include <systemlib/airspeed.h>
 #include <drivers/drv_airspeed.h>
 #include <uORB/topics/differential_pressure.h>
#if defined(CONFIG_ARCH_BOARD_VRBRAIN_V45)
 #define ARSPD_DEFAULT_PIN 0
#elif defined(CONFIG_ARCH_BOARD_VRBRAIN_V51)
 #define ARSPD_DEFAULT_PIN 0
#elif defined(CONFIG_ARCH_BOARD_VRBRAIN_V52)
 #define ARSPD_DEFAULT_PIN 0
#elif defined(CONFIG_ARCH_BOARD_VRUBRAIN_V51)
 #define ARSPD_DEFAULT_PIN 0
#elif defined(CONFIG_ARCH_BOARD_VRUBRAIN_V52)
 #define ARSPD_DEFAULT_PIN 0
#elif defined(CONFIG_ARCH_BOARD_VRCORE_V10)
 #define ARSPD_DEFAULT_PIN 0
#elif defined(CONFIG_ARCH_BOARD_VRBRAIN_V54)
 #define ARSPD_DEFAULT_PIN 0
#elif defined(CONFIG_ARCH_BOARD_PX4FMU_V1)
 #define ARSPD_DEFAULT_PIN 11
#else
 #define ARSPD_DEFAULT_PIN 15
#endif
#elif CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    #if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO2 || CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO
         #define ARSPD_DEFAULT_PIN 5
    #else
         #define ARSPD_DEFAULT_PIN AP_AIRSPEED_I2C_PIN
    #endif
#else
 #define ARSPD_DEFAULT_PIN 0
#endif

// table of user settable parameters
const AP_Param::GroupInfo HGM_ThreePresSnsr::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: ThreePresSnsr enable
    // @Description: enable three-pressure-sensor (fhp)
    // @Values: 0:Disable,1:Enable
    AP_GROUPINFO_FLAGS("ENABLE", 0, HGM_ThreePresSnsr, _enable, 1, AP_PARAM_FLAG_ENABLE),

    AP_GROUPEND
};


HGM_ThreePresSnsr::HGM_ThreePresSnsr()
{
    AP_Param::setup_object_defaults(this, var_info);
}

bool HGM_ThreePresSnsr::init()
{

    _dev_snsrRM = hal.i2c_mgr->get_device(MS4525D0_I2C_BUS, RM_I2C_ADDR);
    _dev_snsrUD = hal.i2c_mgr->get_device(MS4525D0_I2C_BUS, UD_I2C_ADDR);
    _dev_snsrLR = hal.i2c_mgr->get_device(MS4525D0_I2C_BUS, LR_I2C_ADDR);

    // take i2c bus sempahore
    if ((!_dev_snsrRM || !_dev_snsrRM->get_semaphore()->take(200)) ||
        (!_dev_snsrUD || !_dev_snsrUD->get_semaphore()->take(200)) ||
        (!_dev_snsrLR || !_dev_snsrLR->get_semaphore()->take(200))   ) {
        return false;
    }

    _measure();
    hal.scheduler->delay(10);
    _collect();
    _dev_snsrRM->get_semaphore()->give();
    _dev_snsrUD->get_semaphore()->give();
    _dev_snsrLR->get_semaphore()->give();    

    if (_last_sample_time_ms != 0) {
        hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&HGM_ThreePresSnsr::_timer, void));
        return true;
    }
    return false;
}


// start a measurement
void HGM_ThreePresSnsr::_measure()
{
    _measurement_started_ms = 0;
    uint8_t cmd = 0;
    if (_dev_snsrRM->transfer(&cmd, 1, nullptr, 0) &&
        _dev_snsrUD->transfer(&cmd, 1, nullptr, 0) &&
        _dev_snsrLR->transfer(&cmd, 1, nullptr, 0)   ) {
        _measurement_started_ms = AP_HAL::millis();
    }
}

// read the values from the sensor
void HGM_ThreePresSnsr::_collect()
{
    uint8_t data_RM[4];
    uint8_t data_UD[4];
    uint8_t data_LR[4];    

    _measurement_started_ms = 0;

    if (!_dev_snsrRM->transfer(nullptr, 0, data_RM, sizeof(data_RM)) ||
        !_dev_snsrUD->transfer(nullptr, 0, data_UD, sizeof(data_UD)) ||
        !_dev_snsrLR->transfer(nullptr, 0, data_LR, sizeof(data_LR))   ) {
        return;
    }

    uint8_t status_RM = data_RM[0] & 0xC0;
    if (status_RM == 1 || status_RM == 2) { // 1=RESERVED, 2=OLD_DATA
        return;
    }
    uint8_t status_UD = data_UD[0] & 0xC0;
    if (status_UD == 1 || status_UD == 2) { // 1=RESERVED, 2=OLD_DATA
        return;
    }
    uint8_t status_LR = data_LR[0] & 0xC0;
    if (status_LR == 1 || status_LR == 2) { // 1=RESERVED, 2=OLD_DATA
        return;
    }

    int16_t p_RM, T_RM;
    p_RM = (data_RM[0] << 8) + data_RM[1];
    p_RM = 0x3FFF & p_RM;
    T_RM = (data_RM[2] << 8) + data_RM[3];
    T_RM = (0xFFE0 & T_RM) >> 5;

    int16_t p_UD, T_UD;
    p_UD = (data_UD[0] << 8) + data_UD[1];
    p_UD = 0x3FFF & p_UD;
    T_UD = (data_UD[2] << 8) + data_UD[3];
    T_UD = (0xFFE0 & T_UD) >> 5;

    int16_t p_LR, T_LR;
    p_LR = (data_LR[0] << 8) + data_LR[1];
    p_LR = 0x3FFF & p_LR;
    T_LR = (data_LR[2] << 8) + data_LR[3];
    T_LR = (0xFFE0 & T_LR) >> 5;

    const float temp_bits_max = 16383.0f;
    //const float A_min = 0.1*temp_bits_max;
    //const float A_max = 0.9*temp_bits_max;
    const float B_min = 0.05*temp_bits_max;
    const float B_max = 0.95*temp_bits_max;
    const float inh2o_to_pa = 249.088908333;

    _pressure_RM = (((float)p_RM)-B_min)*(5*inh2o_to_pa)/(B_max-B_min) + (0);
    _pressure_UD = (((float)p_UD)-B_min)*(4*inh2o_to_pa)/(B_max-B_min) + (-2*inh2o_to_pa);
    _pressure_LR = (((float)p_LR)-B_min)*(4*inh2o_to_pa)/(B_max-B_min) + (-2*inh2o_to_pa);
    
    _temperature_RM = ((200.0f * T_RM) / 2047) - 50;
    _temperature_UD = ((200.0f * T_UD) / 2047) - 50;
    _temperature_LR = ((200.0f * T_LR) / 2047) - 50;

    _last_sample_time_ms = AP_HAL::millis();
}

// 1kHz timer
void HGM_ThreePresSnsr::_timer()
{
    if (!_dev_snsrRM->get_semaphore()->take_nonblocking() ||
        !_dev_snsrUD->get_semaphore()->take_nonblocking() ||
        !_dev_snsrLR->get_semaphore()->take_nonblocking()  ) {
        return;
    }

    if (_measurement_started_ms == 0) {
        _measure();
        _dev_snsrRM->get_semaphore()->give();
        _dev_snsrUD->get_semaphore()->give();
        _dev_snsrLR->get_semaphore()->give();
        return;
    }
    if ((AP_HAL::millis() - _measurement_started_ms) > 10) {
        _collect();
        // start a new measurement
        _measure();
    }
    _dev_snsrRM->get_semaphore()->give();
    _dev_snsrUD->get_semaphore()->give();
    _dev_snsrLR->get_semaphore()->give();
}

float HGM_ThreePresSnsr::get_pressure_RM(void)
{
    if (!_enable) {
        return 0;
    }
    float pressure_RM = 0;

    if ((AP_HAL::millis() - _last_sample_time_ms) > 100) {
        _healthy = false;
    }
    else {
        pressure_RM = _pressure_RM;
        _healthy = true;
    }

    return pressure_RM;
}
float HGM_ThreePresSnsr::get_pressure_UD(void)
{
    if (!_enable) {
        return 0;
    }
    float pressure_UD = 0;

    if ((AP_HAL::millis() - _last_sample_time_ms) > 100) {
        _healthy = false;
    }
    else {
        pressure_UD = _pressure_UD;
        _healthy = true;
    }

    return pressure_UD;
}
float HGM_ThreePresSnsr::get_pressure_LR(void)
{
    if (!_enable) {
        return 0;
    }
    float pressure_LR = 0;

    if ((AP_HAL::millis() - _last_sample_time_ms) > 100) {
        _healthy = false;
    }
    else {
        pressure_LR = _pressure_LR;
        _healthy = true;
    }

    return pressure_LR;
}
float HGM_ThreePresSnsr::get_temperature_RM(void)
{
    if (!_enable) {
        return 0;
    }
    return _temperature_RM;
}
float HGM_ThreePresSnsr::get_temperature_UD(void)
{
    if (!_enable) {
        return 0;
    }
    return _temperature_UD;
}
float HGM_ThreePresSnsr::get_temperature_LR(void)
{
    if (!_enable) {
        return 0;
    }
    return _temperature_LR;
}

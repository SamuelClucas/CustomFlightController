#include "telemetry.h"
#include "radio_receiver.h"
#include "sense.h"
#include "act.h"
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include <cstdio>
#include <cstring>
#include <cstdarg>
#include <cmath>
#include <unordered_map>
#include <string>

// --- Main Loop ---
int main() {
    Telemetry telemetry;    
    sleep_ms(1000);  // let things settle
    telemetry.send_telemetry("BOOT OK\n");

    uint64_t last_flush_time = 0;

    Receiver receiver;
    Act act;
    Sense sense(telemetry, act);

    act.init_pca9685(telemetry);
    sleep_ms(500);

    for (uint8_t addr = 1; addr < 127; ++addr) {
        uint8_t dummy;
        int ret = i2c_read_blocking(i2c0, addr, &dummy, 1, false);
        if (ret >= 0) {
            telemetry.send_telemetry("Found I2C device at 0x%02X\n", addr);
        }
    }
        
    float dt = 0.0f;
    telemetry.send_telemetry("ENTERING LOOP\n"); 
    sense.calibrate_gyro(act, telemetry, dt);
    uint64_t last = time_us_64();
    bool ground_set = false;
    while (true) {
        uint64_t now = time_us_64();
        dt = (now - last) / 1e6f;

        receiver.read_ibus();
        receiver.fail_safe(telemetry, act); 
        
        sense.read_bmp(dt, telemetry); // always read before resetting ground altitude
        if (receiver.check_altitude_switch()){ // set ground altitude, SWB switch (whenever switch position changes)
            sense.set_ground_altitude(telemetry); 
            ground_set = true;
        }
        if (act.safety_check(sense, receiver, telemetry) && ground_set) { // cannot fly without setting ground altitude once
            sense.update_voltage(sense, telemetry, act, receiver, dt);
            sense.read_mpu6050(dt, telemetry);
            sense.update_magnetometer(telemetry);
            receiver.filter_channels(telemetry);
            receiver.fail_safe(telemetry, act);
            sense.PID_calculations(receiver, act, telemetry, dt);
            act.recovery_reset();
            } 
        else if (receiver.check_switch_armed(act, telemetry) && sense.get_current_altitude() > (sense.get_ground_altitude() + 0.15f) && ground_set) { // assuming altitude is in meters
                act.recovery(sense, telemetry, receiver, dt); // attempt drone recovery
                sense.reset_integrals(); // reset error variables
            }

        if (now - last_flush_time > 1000000) {  // 1Hz
            telemetry.flush_telemetry();
            last_flush_time = now;
        }
        last = now;
    }
}


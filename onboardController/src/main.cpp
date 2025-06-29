#include "telemetryRelay.h"
#include "radioReceiver.h"
#include "sensorController.h"
#include "motorController.h"
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
    TelemetryRelay telemetry;    
    sleep_ms(1000);  // let things settle
    telemetry.send_telemetry("BOOT OK\n");

    uint64_t last_flush_time = 0;

    RadioReceiver receiver;
    MotorController mController;
    SensorController sController(telemetry, mController);

    mController.init_pca9685(telemetry);
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
    sController.calibrate_gyro(mController, telemetry, dt);
    uint64_t last = time_us_64();
    bool ground_set = false;
    while (true) {
        uint64_t now = time_us_64();
        dt = (now - last) / 1e6f;

        receiver.read_ibus();
        receiver.fail_safe(telemetry, mController); 
        
        sController.read_bmp(dt, telemetry); // always read before resetting ground altitude
        if (receiver.check_altitude_switch()){ // set ground altitude, SWB switch (whenever switch position changes)
            sController.set_ground_altitude(telemetry); 
            ground_set = true;
        }
        if (mController.safety_check(sController, receiver, telemetry) && ground_set) { // cannot fly without setting ground altitude once
            sController.update_voltage(sController, telemetry, mController, receiver, dt);
            sController.read_mpu6050(dt, telemetry);
            sController.update_magnetometer(telemetry);
            receiver.filter_channels(telemetry);
            receiver.fail_safe(telemetry, mController);
            sController.PID_calculations(receiver, mController, telemetry, dt);
            mController.recovery_reset();
            } 
        else if (receiver.check_switch_armed(mController, telemetry) && sController.get_current_altitude() > (sController.get_ground_altitude() + 0.15f) && ground_set) { // assuming altitude is in meters
                mController.recovery(sController, telemetry, receiver, dt); // attempt drone recovery
                sController.reset_integrals(); // reset error variables
            }

        if (now - last_flush_time > 1000000) {  // 1Hz
            telemetry.flush_telemetry();
            last_flush_time = now;
        }
        last = now;
    }
}


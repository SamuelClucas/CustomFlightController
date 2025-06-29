#pragma once

#include "pico/stdlib.h"

class SensorController; // forward declarations
class RadioReceiver;
class TelemetryRelay;

class MotorController {
private:
    bool allow_arm;
    int m0, m1, m2, m3;
    bool pca_ok;
    static int constexpr PCA9685_ADDR = 0x40;
    static int constexpr PCA9685_FREQ = 50;
    float roll_integral = 0.0f; 
    float pitch_integral = 0.0f; 
    float yaw_integral = 0.0f; 
    float accel_integral = 0.0f;

    static constexpr float KP = 0.8f;
    static constexpr float KI = 0.02f; // very low integral initially
    static constexpr float KD = 0.01f;
    static constexpr float YAW_KP = 0.3f;
    static constexpr float YAW_KI = 0.002f;
    static constexpr float YAW_KD = 0.005f;

    static constexpr int MIN_THROTTLE = 1000; 
    static constexpr int MAX_THROTTLE = 2000;

    int throttle = 1400; // lifts at 1400 throttle

public:
    MotorController();
    ~MotorController();

    void init_pca9685(TelemetryRelay& telemetry);
    void pca_read_check(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t val);
    bool pca_write_check(uint8_t addr, uint8_t reg, uint8_t val);

    void kill_motors(TelemetryRelay& telemetry);
    bool set_pwm_us(uint8_t ch, uint16_t us, TelemetryRelay& telemetry);

    void disarm(TelemetryRelay& telemetry);
    void arm(RadioReceiver& receiver, TelemetryRelay& telemetry);
    bool armed();

    bool safety_check(SensorController& sense, RadioReceiver& receiver, TelemetryRelay& telemetry);

    void set_m0(int m0_);
    void set_m1(int m1_);
    void set_m2(int m2_);
    void set_m3(int m3_);

    static int clamp(int val, int min_val, int max_val);
    bool get_pca_ok() { return pca_ok; }
    void set_allow_arm(bool state) { allow_arm = state; }

    void m_test(TelemetryRelay& telemetry); 

    void recovery(SensorController& sense, TelemetryRelay& telemetry, RadioReceiver& receiver, float dt);
    
    float clampf(float val, float min_val, float max_val) { return val < min_val ? min_val : (val > max_val ? max_val : val); };

    void recovery_reset() {
        throttle = 1400;
        roll_integral = 0.0f; 
        pitch_integral = 0.0f; 
        yaw_integral = 0.0f; 
        accel_integral = 0.0f;
    };
    
};
#include "sensorController.h"
#include "radioReceiver.h"
#include "telemetryRelay.h"
#include "motorController.h"
#include "pico/stdlib.h"
#include <cmath>

MotorController::MotorController()
{
    pca_ok = 0;
    allow_arm = false;
    m0 = 1000; m1 = 1000; m2 = 1000; m3 = 1000;
}

MotorController::~MotorController() {}

int Act::clamp(int val, int min_val, int max_val) { return val < min_val ? min_val : (val > max_val ? max_val : val); }

void MotorController::init_pca9685(TelemetryRelay& telemetry)
{
    bool s0 = pca_write_check(0x40, 0x00, 0x10);  // Sleep
    bool s1 = pca_write_check(0x40, 0xFE, (uint8_t)(25000000 / (4096 * 50) - 1));  // Prescale
    bool s2 = pca_write_check(0x40, 0x00, 0xA1);  // Restart + Auto-Inc

    pca_ok = s0 && s1 && s2;
    telemetry.update_telemetry("[PCA W0]", "%d", s0);
    telemetry.update_telemetry("[PCA W1]", "%d", s1);
    telemetry.update_telemetry("[PCA W2]", "%d", s2);

    if (!pca_ok) disarm(telemetry);
}
bool MotorContorller::pca_write_check(uint8_t addr, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};  // <-- critical fix: no more temporary array!
    int ret = i2c_write_blocking(i2c0, addr, buf, 2, false);
    if (ret != 2) {
        allow_arm = false;
        return false;
    }
    return true;
}

void MotorController::pca_read_check(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len)
{
    if (i2c_write_blocking(i2c0, addr, &reg, 1, true) != 1) { allow_arm = false; pca_ok = false; }
    if (i2c_read_blocking(i2c0, addr, buf, len, false) != len) { allow_arm = false; pca_ok = false; }
    else {
        pca_ok = true;
    }
}

void MotorController::arm(RadioReceiver& receiver, TelemetryRelay& telemetry)
{
    if (receiver.check_switch_armed(*this, telemetry)){
        allow_arm = true;
    }
    else {
        disarm(telemetry);
    }
}

void MotorController::kill_motors(TelemetryRelay& telemetry)
{
    for (int i = 0; i < 4; i++) {
        set_pwm_us(i, 0, telemetry);
    }
}

bool MotorController::set_pwm_us(uint8_t ch, uint16_t us, TelemetryRelay& telemetry)
{
    us = clamp((int)us, 1000, 2000);  // Clamp to safe servo range
    uint16_t ticks = (uint16_t)(us * PCA9685_FREQ * 4096 / 1000000.0f + 0.5f);

    uint8_t data[] = {
        (uint8_t)(0x06 + 4 * ch),  // LEDx_ON_L
        0,                         // ON_L
        0,                         // ON_H
        (uint8_t)(ticks & 0xFF),   // OFF_L
        (uint8_t)(ticks >> 8)      // OFF_H
    };

    bool success = i2c_write_blocking(i2c0, PCA9685_ADDR, data, 5, false) == 5;
    if (!success) {
        telemetry.send_telemetry("PCA write failed on ch %d\n", ch);
    }
    return success;
}

void MotorController::disarm(TelemetryRelay& telemetry) {
    allow_arm = false;
    kill_motors(telemetry);
}

bool MotorController::armed()
{
    return allow_arm;
}

bool MotorController::safety_check(SensorController& sense, RadioReceiver& receiver, TelemetryRelay& telemetry)
{
    static bool safety_state = true;
    receiver.fail_safe(telemetry, *this);
    allow_arm = receiver.check_switch_armed(*this, telemetry);
    telemetry.update_telemetry("[Switch]", "%d", allow_arm);
    static bool bmp_Warn = false;
    static bool mpu_Warn = false;
    static bool qmc_Warn = false;
    static bool pitch_Warn = false;
    static bool volt_Warn = false;
    static bool pca_Warn = false;

    if(allow_arm) { // add warning reset
        if (!sense.get_bmp_ok()) {
            if(!bmp_Warn){
                telemetry.send_telemetry("BMP FAIL\n");
                safety_state = false;
                bmp_Warn = true;
            }

        }
        else if (!sense.get_mpu_ok()) {
            if(!mpu_Warn){
                telemetry.send_telemetry("MPU FAIL\n");
                safety_state = false;
                mpu_Warn = true;
            }
        }
        else if (!sense.get_qmc_ok()) {
            if(!qmc_Warn){
                telemetry.send_telemetry("QMC FAIL\n");
                safety_state = false;
                qmc_Warn = true;
            }
        }
        else if (!pca_ok) {
            if(!pca_Warn){
                telemetry.send_telemetry("PCA FAIL\n");
                safety_state = false;
                pca_Warn=true;
            }
        }
        else if (!(fabsf(sense.get_pitch_angle()) < 25.0f && fabsf(sense.get_roll_angle()) < 25.0f)) {
            if(!pitch_Warn){
                telemetry.send_telemetry("G_OVERTILT\n");
                safety_state = false;
                pitch_Warn = true;
            }
        }
        else if (sense.get_critically_warned()){
            if (!volt_Warn){
                telemetry.send_telemetry("VOLTAGE CRITICAL\n");
                safety_state = false;
                volt_Warn = true;
            }
        }
        else {
            safety_state = true; // passed safety check allows safety state reset
            bmp_Warn = false;
            mpu_Warn = false;
            qmc_Warn = false;
            pitch_Warn = false;
            volt_Warn = false;
        }
    }
    if (safety_state == true && allow_arm) {return 1;}
    else {
        return 0;
    }
}

void MotorController::recovery(SensorController& sense, TelemetryRelay& telemetry, RadioReceiver& receiver, float dt)
{
    if (dt <= 0.0f) return;
    // RECOVERY PID LOOP
    
    float last_roll_angle = sense.get_roll_angle(); // move to 0
    float last_pitch_angle = sense.get_pitch_angle(); // move to 0
    float last_yaw_hold = sense.get_yaw_angle(); // maintain this
    float last_accel_z = sense.get_az_f();
    
    sense.read_mpu6050(dt, telemetry); // fused roll and pitch
    sense.update_magnetometer(telemetry); // fused yaw
    sense.read_bmp(dt, telemetry); // measure altitude

    float inv_dt = 1 / dt;
    
    float accel_derivative = clamp(((sense.get_az_f() - last_accel_z) * inv_dt), -10.0f, 10.0f);
    float roll_derivative = clamp(((sense.get_roll_angle() - last_roll_angle) * inv_dt), -10.0f, 10.0f);
    float pitch_derivative = clamp(((sense.get_pitch_angle() - last_pitch_angle) * inv_dt), -10.0f, 10.0f);
    float yaw_derivative = clamp(((sense.get_yaw_angle() - last_yaw_hold) * inv_dt), -10.0f, 10.0f);
    telemetry.update_telemetry("[RollD]", "%.2f", roll_derivative);
    telemetry.update_telemetry("[PitchD]", "%.2f", pitch_derivative);
    telemetry.update_telemetry("[YawD]", "%.2f", yaw_derivative);
    telemetry.update_telemetry("[AccelD]", "%.2f", accel_derivative);

    float accel_error = -0.1f - sense.get_az_f();

    yaw_integral = clampf(yaw_integral + sense.get_yaw_angle() * dt, -10.0f, 10.0f);
    pitch_integral = clampf(pitch_integral + sense.get_pitch_angle() * dt, -10.0f, 10.0f);
    roll_integral = clampf(roll_integral + sense.get_roll_angle() * dt, -10.0f, 10.0f);
    accel_integral = clampf(accel_integral + accel_error * dt, -10.0f, 10.0f);
    telemetry.update_telemetry("[YawIntegral]", "%.2f", yaw_integral);
    telemetry.update_telemetry("[PitchIntegral]", "%.2f", pitch_integral);
    telemetry.update_telemetry("[RollIntegral]", "%.2f", roll_integral);
    telemetry.update_telemetry("[AccelIntegral]", "%.2f", accel_integral);

    float pitch_pid = sense.get_pitch_angle() * KP + pitch_derivative * KD + pitch_integral * KI;
    float roll_pid = sense.get_roll_angle() * KP + roll_derivative * KD + roll_integral * KI;
    float yaw_pid = sense.get_yaw_angle() * YAW_KP + yaw_derivative * YAW_KD + yaw_integral * YAW_KI;
    float accel_pid = clamp((-0.1f - sense.get_az_f()) * KP + accel_derivative * KD + accel_integral * KI, -5.0f, +5.0f);
    
    throttle = clamp(throttle + accel_pid, MIN_THROTTLE + 100, MAX_THROTTLE - 200);
    int m0 = clamp((throttle - roll_pid - pitch_pid - yaw_pid), MIN_THROTTLE, MAX_THROTTLE); // Front Right CCW
    int m1 = clamp((throttle + roll_pid + pitch_pid - yaw_pid), MIN_THROTTLE, MAX_THROTTLE); // Back Left CCW
    int m2 = clamp((throttle + roll_pid - pitch_pid + yaw_pid), MIN_THROTTLE, MAX_THROTTLE); // Front Left CW
    int m3 = clamp((throttle - roll_pid + pitch_pid + yaw_pid), MIN_THROTTLE, MAX_THROTTLE); // Back Right CW
    telemetry.update_telemetry("[M0]recov", "%d", m0);
    telemetry.update_telemetry("[M1]recov", "%d", m1);
    telemetry.update_telemetry("[M2]recov", "%d", m2);
    telemetry.update_telemetry("[M3]recov", "%d", m3);

    set_pwm_us(0, m0, telemetry);
    set_pwm_us(1, m1, telemetry);
    set_pwm_us(2, m2, telemetry);
    set_pwm_us(3, m3, telemetry);
}

void MotorController::m_test(TelemetryRelay& telemetry) {
    set_pwm_us(0, 1100, telemetry);
    telemetry.send_telemetry("M0 now!");
    sleep_ms(2000);
    set_pwm_us(1, 1100, telemetry); // Only M1 spins
    telemetry.send_telemetry("M1 now!");
    sleep_ms(2000);
    set_pwm_us(2, 1100, telemetry); // Only M1 spins
    telemetry.send_telemetry("M2 now!");
    sleep_ms(2000);
    set_pwm_us(3, 1100, telemetry); // Only M1 spins
    telemetry.send_telemetry("M3 now!");
    sleep_ms(2000);
}

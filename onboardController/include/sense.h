#include "telemetry.h"
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "radio_receiver.h"
#pragma once
class Act;

struct BMP280_Calib {
    uint16_t dig_T1;
    int16_t dig_T2, dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2, dig_P3, dig_P4, dig_P5;
    int16_t dig_P6, dig_P7, dig_P8, dig_P9;
};

class Sense {
    private:
        BMP280_Calib calib;
        int32_t t_fine;
        static int constexpr MPU6050_ADDR = 0x68;
        static int constexpr BMP280_ADDR = 0x76;
        static int constexpr QMC5883_ADDR = 0x0D;
        
        static int constexpr SDA_PIN  = 0;
        static int constexpr SCL_PIN  = 1;

        static int constexpr GYRO_CALIB_SAMPLES = 500;
        static constexpr float GYRO_SENSITIVITY = 131.0f;
        static constexpr float INPUT_SCALE_ROLL = 0.06f;
        static constexpr float YAW_RATE_SCALE = 0.1f;

        const float ACCEL_SCALE = 16384.0f;
        
        bool mpu_ok;
        bool qmc_ok;
        bool bmp_ok;

        float voltage;
        float filtered_voltage;

        float const MAX_ANGLE_DEG = 60.0f;
        
        float mx, my, mz;
        float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
        float ax_g, ay_g, az_g;
        float gx_offset, gy_offset, gz_offset;
        float ax_f = 0.0f, ay_f = 0.0f, az_f = 1.0f; // filtered accel
        
        float roll_angle, pitch_angle, yaw_angle;
        float yaw_offset = 0.0f;
        bool yaw_zeroed = false;
        float yaw_setpoint;
        float mag_yaw;
        float accel_roll;
        float accel_pitch;
        bool angles_valid = 1;
        int accel_rejects = 0;
        int accel_uses = 0;


        static float constexpr SAFE_LANDING_DURATION_S = 5.0f;

        static int constexpr ADC_VOLTAGE_CH = 1;
        static constexpr float VOLTAGE_SCALE = 2.0f * (16.8f / 3.3f);
        static constexpr float VOLTAGE_WARN_LEVEL = 13.5f;
        static constexpr float VOLTAGE_CRITICAL_LEVEL = 13.0f;

        float emergency_throttle_reduction = 1.0f;
        float throttle_ratio; 
        
        // --- PID Gains ---
        static constexpr float KP = 0.8f;
        static constexpr float KI = 0.04f; // very low integral initially
        static constexpr float KD = 0.02f;
        static constexpr float YAW_KP = 0.3f;
        static constexpr float YAW_KI = 0.002f;
        static constexpr float YAW_KD = 0.005f;


        static constexpr float INTEGRAL_LIMIT = 20.0f; // Clamps buildup to sane corrections


        static int constexpr MIN_THROTTLE = 1020;
        static int constexpr MAX_THROTTLE = 2000;

        // Error variables
        float roll_error, pitch_error, yaw_error;
        float last_pitch_error;
        float last_roll_error;
        float last_yaw_error;

        // Derivative variables
        float roll_derivative;
        float pitch_derivative;
        float yaw_derivative;

        // Integral variables
        float roll_integral;
        float pitch_integral;
        float yaw_integral;

        float compensate_temperature(int32_t adc_T);
        float compensate_pressure(int32_t adc_P);
        float current_altitude = 0.0f;
        float last_altitude = 0.0f;
        float ground_altitude = 0.0f;
        bool critically_warned = false;
        
    public:
        Sense(Telemetry& telemetry, Act& act);
        ~Sense();

        void init_mpu6050(Telemetry& telemetry, Act& act);
        void init_qmc5883();
        void init_bmp280();
        bool i2c_write_check(uint8_t addr, uint8_t reg, uint8_t val);
        bool i2c_read_check(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t len);
        void update_voltage(Sense& sense, Telemetry& telemetry, Act& act, Receiver& receiver, float dt);
        
        void calibrate_gyro(Act& act, Telemetry& telemetry, float dt);
        void read_mpu6050(float dt, Telemetry& telemetry);
        void read_bmp(float dt, Telemetry& telemetry);
        void update_magnetometer(Telemetry& telemetry);

        void PID_calculations(Receiver& receiver, Act& act, Telemetry& telemetry, float dt);

        void reset_integrals();

        static inline int clamp(int val, int min_val, int max_val);
        static inline float clampf(float val, float min_val, float max_val);

        float get_roll_angle() { return roll_angle; }
        float get_pitch_angle() { return pitch_angle; }
        bool get_mpu_ok() { return mpu_ok; }
        bool get_qmc_ok() { return qmc_ok; }
        bool get_bmp_ok() { return bmp_ok; }

        float get_current_altitude();
        void set_ground_altitude(Telemetry& telemetry);
        float get_ground_altitude(){return ground_altitude;};
        bool get_critically_warned();
        float get_yaw_angle(){return yaw_angle;};
        float get_az_f(){return az_f;}; 
        
};
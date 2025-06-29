#include "sensorController.h"
#include "telemetryRelay.h"
#include "motorController.h"
#include "radioReceiver.h"
#include <cmath>
#include "pico/stdlib.h"

SensorController::SensorController(TelemetryRelay& telemetry, MotorController& mController)
{
    i2c_init(i2c0, 100000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(4);
    gpio_pull_up(5);
    adc_init(); adc_gpio_init(27);
    mpu_ok = 0;
    qmc_ok = 0;
    bmp_ok = 0;
    
    sleep_ms(300);

    init_mpu6050(telemetry, mController);
    init_qmc5883();
    init_bmp280();
    mag_yaw = 0.0f;

    ax=0, ay=0, az=16384, gx=0, gy=0, gz=0;
    accel_roll = atan2f(ay, az) * 180.0f / M_PI;
    accel_pitch = atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f / M_PI;
    gx_offset = 0.0f, gy_offset = 0.0f, gz_offset = 0.0f;
    roll_angle = 0.0f, pitch_angle = 0.0f, yaw_angle = 0.0f;
}

SensorController::~SensorController() {}

int SensorController::clamp(int val, int min_val, int max_val) { return val < min_val ? min_val : (val > max_val ? max_val : val); }
float SensorController::clampf(float val, float min_val, float max_val) { return val < min_val ? min_val : (val > max_val ? max_val : val); }

bool SensorController::i2c_write_check(uint8_t addr, uint8_t reg, uint8_t val)
{
    int ret = i2c_write_blocking(i2c0, addr, (uint8_t[]){reg, val}, 2, false);
    if (ret != 2) { return false; }
    
    return true;
}

bool SensorController::i2c_read_check(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len)
{
    if (i2c_write_blocking(i2c0, addr, &reg, 1, true) != 1) { return false; }
    if (i2c_read_blocking(i2c0, addr, buf, len, false) != len) { return false; }
    return true;
}

void SensorController::init_qmc5883() {
    qmc_ok = i2c_write_check(QMC5883_ADDR, 0x0B, 0x01); // Soft reset
    sleep_ms(10);
    qmc_ok &= i2c_write_check(QMC5883_ADDR, 0x09, 0x1D); // Continuous, 200Hz, 2G, 512 OSR
    qmc_ok &= i2c_write_check(QMC5883_ADDR, 0x0A, 0x01); // Set/Reset period
}


void SensorController::init_mpu6050(TelemetryRelay& telemetry, MotorController& mController) {
    mpu_ok = i2c_write_check(MPU6050_ADDR, 0x6B, 0x00); // Wake up
    mpu_ok &= i2c_write_check(MPU6050_ADDR, 0x1C, 0x00); // Set accel ±2g
    mpu_ok &= i2c_write_check(MPU6050_ADDR, 0x1B, 0x00); // Set gyro ±250dps

    static bool first_mpu = true;
    if (first_mpu) {
        if (!mpu_ok) {
            telemetry.send_telemetry("MPUinit FAIL\n");
            act.disarm(telemetry);
        }
        first_mpu = false;
    }
    

}

void SensorController::init_bmp280() {
    uint8_t id = 0;
    if (!i2c_read_check(BMP280_ADDR, 0xD0, &id, 1) || id != 0x58) {
        bmp_ok = false;
        return;
    }

    // CONFIG and CTRL_MEAS setup
    i2c_write_check(BMP280_ADDR, 0xF5, 0x20);
    bmp_ok = i2c_write_check(BMP280_ADDR, 0xF4, 0x37);

    //  Calibration data read
    uint8_t buf[24];
    if (!i2c_read_check(BMP280_ADDR, 0x88, buf, 24)) {
        bmp_ok = false;
        return;
    }

    calib.dig_T1 = buf[1] << 8 | buf[0];
    calib.dig_T2 = buf[3] << 8 | buf[2];
    calib.dig_T3 = buf[5] << 8 | buf[4];

    calib.dig_P1 = buf[7] << 8 | buf[6];
    calib.dig_P2 = buf[9] << 8 | buf[8];
    calib.dig_P3 = buf[11] << 8 | buf[10];
    calib.dig_P4 = buf[13] << 8 | buf[12];
    calib.dig_P5 = buf[15] << 8 | buf[14];
    calib.dig_P6 = buf[17] << 8 | buf[16];
    calib.dig_P7 = buf[19] << 8 | buf[18];
    calib.dig_P8 = buf[21] << 8 | buf[20];
    calib.dig_P9 = buf[23] << 8 | buf[22];
}


void SensorController::read_bmp(float dt, TelemetryRelay& telemetry) {
    if (!bmp_ok) return;

    // Read raw pressure and temperature (6 bytes from 0xF7)
    uint8_t data[6];
    if (!i2c_read_check(BMP280_ADDR, 0xF7, data, 6)) {
        bmp_ok = false;
        return;
    }

    int32_t adc_P = ((int32_t)data[0] << 12) | ((int32_t)data[1] << 4) | (data[2] >> 4);
    int32_t adc_T = ((int32_t)data[3] << 12) | ((int32_t)data[4] << 4) | (data[5] >> 4);

    // Must call compensate_temperature() first to set t_fine
    float temperature = compensate_temperature(adc_T);  // °C
    float pressure = compensate_pressure(adc_P);        // Pa

    bmp_ok = true;
    telemetry.update_telemetry("[Temp]", "%.2f", temperature);
    telemetry.update_telemetry("[Press]", "%.2f", pressure);

    // Optionally convert to altitude
    float sea_level_pa = 101325.0f; // Adjust this if flying at altitude
    current_altitude = 44330.0f * (1.0f - powf(pressure / sea_level_pa, 0.1903f));
    if (last_altitude==0.0f){
        last_altitude = current_altitude;
    }
    else {
        last_altitude = 0.98f*last_altitude + 0.02f * current_altitude;
        current_altitude = last_altitude;
    }
    telemetry.update_telemetry("[Alt]", "%.2f", current_altitude);
}

float SensorController::compensate_temperature(int32_t adc_T) {
    int32_t var1 = ((((adc_T >> 3) - ((int32_t)calib.dig_T1 << 1))) * ((int32_t)calib.dig_T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - ((int32_t)calib.dig_T1)) * ((adc_T >> 4) - ((int32_t)calib.dig_T1))) >> 12) *
                    ((int32_t)calib.dig_T3)) >> 14;
    t_fine = var1 + var2;
    return ((t_fine * 5 + 128) >> 8) / 100.0f;  // Celsius
}

float SensorController::compensate_pressure(int32_t adc_P) {
    int64_t var1 = (int64_t)t_fine - 128000;
    int64_t var2 = var1 * var1 * (int64_t)calib.dig_P6;
    var2 += ((var1 * (int64_t)calib.dig_P5) << 17);
    var2 += ((int64_t)calib.dig_P4 << 35);
    var1 = ((var1 * var1 * (int64_t)calib.dig_P3) >> 8) + ((var1 * (int64_t)calib.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * (int64_t)calib.dig_P1 >> 33;

    if (var1 == 0) return 0;  // avoid division by zero

    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = ((int64_t)calib.dig_P9 * (p >> 13) * (p >> 13)) >> 25;
    var2 = ((int64_t)calib.dig_P8 * p) >> 19;

    p = ((p + var1 + var2) >> 8) + ((int64_t)calib.dig_P7 << 4);
    return (float)p / 256.0f;  // Pascals
}

float SensorController::get_current_altitude(){
    return current_altitude;
}

void SensorController::set_ground_altitude(TelemetryRelay& telemetry){
    ground_altitude = current_altitude;
    telemetry.send_telemetry("Ground altitude set: %.2f", ground_altitude);
}

void SensorController::read_mpu6050(TelemetryRelay &telemetry, float dt)
{
    uint8_t buf[14];
    if (!i2c_read_check(MPU6050_ADDR, 0x3B, buf, 14)) mpu_ok = false;
    else {
        ax = (int16_t)(buf[0] << 8) | buf[1]; ay = (int16_t)(buf[2] << 8) | buf[3]; az = (int16_t)(buf[4] << 8) | buf[5];
        gx = (int16_t)(buf[8] << 8) | buf[9]; gy = (int16_t)(buf[10] << 8) | buf[11]; gz = (int16_t)(buf[12] << 8) | buf[13];
        mpu_ok = true;
    }
    if (!mpu_ok) {
        angles_valid = 0;
        accel_rejects++;
        return;
    }
    ax_g = ax / ACCEL_SCALE;
    ay_g = ay / ACCEL_SCALE;
    az_g = az / ACCEL_SCALE;
    gx = (gx - gx_offset) / GYRO_SENSITIVITY;
    gy = (gy - gy_offset) / GYRO_SENSITIVITY;
    gz = (gz - gz_offset) / GYRO_SENSITIVITY;

    // 1. Always integrate gyro
    roll_angle  += gx * dt;
    pitch_angle += gy * dt;
    yaw_angle   += gz * dt;

    // 2. Compute accel angles (with optional filtering)
    accel_roll  = atan2f(ay_g, az_g) * 180.0f / M_PI;
    accel_pitch = atan2f(-ax_g, sqrtf(ay_g * ay_g + az_g * az_g)) * 180.0f / M_PI;

    // 3. Compute accel magnitude
    float a_mag = sqrtf(ax_g * ax_g + ay_g * ay_g + az_g * az_g);

    // 4. Blend ONLY if accel looks sane
    if (fabsf(a_mag - 1.0f) < 0.5f) {
        roll_angle  = 0.98f * roll_angle  + 0.02f * accel_roll;
        pitch_angle = 0.98f * pitch_angle + 0.02f * accel_pitch;
        angles_valid = 1;
        accel_uses++;
    } else {
        accel_rejects++;
    }

    telemetry.update_telemetry("[RejectA]", "%d", accel_rejects);
    telemetry.update_telemetry("[AcceptA]", "%d", accel_uses);
}

void SensorController::calibrate_gyro(Act& act, TelemetryRelay& telemetry, float dt) {
    int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0;
    for (int i = 0; i < GYRO_CALIB_SAMPLES; i++) {
        uint8_t buf[14];
        if (!i2c_read_check(MPU6050_ADDR, 0x3B, buf, 14)) continue;
        int16_t gx_raw = (int16_t)(buf[8] << 8) | buf[9];
        int16_t gy_raw = (int16_t)(buf[10] << 8) | buf[11];
        int16_t gz_raw = (int16_t)(buf[12] << 8) | buf[13];

    gx_sum += gx_raw;
    gy_sum += gy_raw;
    gz_sum += gz_raw;
    sleep_ms(2);
    }

    gx_offset = gx_sum / (float)GYRO_CALIB_SAMPLES;
    gy_offset = gy_sum / (float)GYRO_CALIB_SAMPLES;
    gz_offset = gz_sum / (float)GYRO_CALIB_SAMPLES;

    read_mpu6050(dt, telemetry);  // this already computes ax_g, ay_g, az_g
    roll_angle = accel_roll;
    pitch_angle = accel_pitch;

    telemetry.send_telemetry("[GyroCalib] Roll %.2f Pitch %.2f\n", roll_angle, pitch_angle);
}

void SensorController::update_magnetometer(TelemetryRelay& telemetry) {
    uint8_t buf[6];
    if (!i2c_read_check(QMC5883_ADDR, 0x00, buf, 6)) {
        qmc_ok = false;
        return;
    }
    
    int16_t mx_raw = (int16_t)(buf[1] << 8) | buf[0];
    int16_t my_raw = (int16_t)(buf[3] << 8) | buf[2];
    int16_t mz_raw = (int16_t)(buf[5] << 8) | buf[4];
    
    mx = (float)mx_raw;
    my = (float)my_raw;
    mz = (float)mz_raw;
    
    mag_yaw = atan2f(my, mx) * 180.0f / M_PI;
    
    // Auto-zero yaw on first valid reading
    if (!yaw_zeroed) {
        yaw_offset = mag_yaw;
        yaw_zeroed = true;
        telemetry.update_telemetry("[YawOffset]", "%f\n", yaw_offset);
    }
    
    // Apply correction
    mag_yaw -= yaw_offset;
    if (mag_yaw > 180.0f) mag_yaw -= 360.0f;
    if (mag_yaw < -180.0f) mag_yaw += 360.0f;
    
    // Fuse with gyro-integrated yaw
    yaw_angle = 0.98f * yaw_angle + 0.02f * mag_yaw;
}

void SensorController::PID_calculations(Receiver &receiver, MotorController& mController, TelemetryRelay &telemetry, float dt)
{
    mController.set_allow_arm(receiver.has_recent_data() && act.get_pca_ok() && bmp_ok && fabsf(roll_angle) <= 30.0f && fabsf(pitch_angle) <= 30.0f);
    if (dt <= 0.0f) return;
    
    throttle_ratio = (receiver.get_throttle() - MIN_THROTTLE) / (float)(MAX_THROTTLE - MIN_THROTTLE);
    throttle_ratio = clampf(throttle_ratio, 0.0f, 1.0f);

    // Optional: apply a minimum cutoff to avoid too-early scaling
    if (throttle_ratio < 0.35f) throttle_ratio = 0.0f;

    roll_error = -roll_angle + receiver.get_roll() * INPUT_SCALE_ROLL;
    pitch_error = -pitch_angle + receiver.get_pitch() * INPUT_SCALE_ROLL;
    yaw_setpoint = receiver.get_yaw() * YAW_RATE_SCALE;
    yaw_error = yaw_setpoint - gz;

    roll_integral = clampf(roll_integral + roll_error * dt, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
    pitch_integral = clampf(pitch_integral + pitch_error * dt, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
    yaw_integral = clampf(yaw_integral + yaw_error * dt, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

    float inv_dt = 1.0f / dt;
    roll_derivative = (roll_error - last_roll_error) * inv_dt;
    pitch_derivative = (pitch_error - last_pitch_error) * inv_dt;
    yaw_derivative = (yaw_error - last_yaw_error) * inv_dt;
    last_roll_error = roll_error; last_pitch_error = pitch_error; last_yaw_error = yaw_error;

    int roll_pid = throttle_ratio * (KP * roll_error + KI * roll_integral + KD * roll_derivative);
    int pitch_pid = throttle_ratio * (KP * pitch_error + KI * pitch_integral + KD * pitch_derivative);
    int yaw_pid = throttle_ratio * (YAW_KP * yaw_error + YAW_KI * yaw_integral + YAW_KD * yaw_derivative);

    int throttle = (int)receiver.get_throttle();

    int m0 = clamp((throttle - roll_pid - pitch_pid - yaw_pid), MIN_THROTTLE, MAX_THROTTLE); // Front Right CCW
    int m1 = clamp((throttle + roll_pid + pitch_pid - yaw_pid), MIN_THROTTLE, MAX_THROTTLE); // Back Left CCW
    int m2 = clamp((throttle + roll_pid - pitch_pid + yaw_pid), MIN_THROTTLE, MAX_THROTTLE); // Front Left CW
    int m3 = clamp((throttle - roll_pid + pitch_pid + yaw_pid), MIN_THROTTLE, MAX_THROTTLE); // Back Right CW

    if(mController.armed()){
        mController.set_pwm_us(0, m0, telemetry);
        mController.set_pwm_us(1, m1, telemetry);
        mController.set_pwm_us(2, m2, telemetry);
        mController.set_pwm_us(3, m3, telemetry);
        telemetry.send_telemetry("[PWM sent] M0:%d M1:%d M2:%d M3:%d", m0, m1, m2, m3); 
    }

    telemetry.update_telemetry("[Gz]", "%.2f", gz);
    telemetry.update_telemetry("[YawError]", "%.2f", yaw_error);
    telemetry.update_telemetry("[Allow Arm]", "%d", act.armed());
    telemetry.update_telemetry("[Roll]", "%.2f", roll_angle);
    telemetry.update_telemetry("[Pitch]", "%.2f", pitch_angle);
    telemetry.update_telemetry("[Yaw]", "%.2f", yaw_angle);
    telemetry.update_telemetry("[M0]", "%d", m0);
    telemetry.update_telemetry("[M1]", "%d", m1);
    telemetry.update_telemetry("[M2]", "%d", m2);
    telemetry.update_telemetry("[M3]", "%d", m3);
    telemetry.update_telemetry("[Gx]", "%.2f", gx);
    telemetry.update_telemetry("[Gy]", "%.2f", gy);
    telemetry.update_telemetry("[Gz]", "%.2f", gz);
    telemetry.update_telemetry("[Ax_g]", "%.2f", ax_g);
    telemetry.update_telemetry("[Ay_g]", "%.2f", ay_g);
    telemetry.update_telemetry("[Az_g]", "%.2f", az_g);
    telemetry.update_telemetry("[MagYaw]", "%.2f", mag_yaw);
}

void SensorController::reset_integrals() {
    roll_integral = pitch_integral = yaw_integral = 0.0f;
    last_roll_error = last_pitch_error = last_yaw_error = 0.0f;
}

void SensorController::update_voltage(TelemetryRelay& telemetry)
{
    adc_select_input(ADC_VOLTAGE_CH);
    voltage = (adc_read() / 4095.0f * 3.3f) * VOLTAGE_SCALE;
    static bool first_read = 0;
    static float last_voltage;
    if(!first_read){
        last_voltage = voltage;
        first_read=1;
    }
    filtered_voltage = 0.9f * last_voltage + 0.1f * voltage; // averaging out bogus reads
    telemetry.update_telemetry("[Voltage]", "%.2f", filtered_voltage);
    last_voltage = filtered_voltage;

    static bool warned = false;
    

    if (!warned && filtered_voltage < VOLTAGE_WARN_LEVEL && filtered_voltage > VOLTAGE_CRITICAL_LEVEL) {
        telemetry.send_telemetry("[Voltage_warn] %.2f\n", filtered_voltage);
        warned = true;
    }

    if (!critically_warned && filtered_voltage < VOLTAGE_CRITICAL_LEVEL) {
        telemetry.send_telemetry("[Voltage_critical] %.2f\n", filtered_voltage);
        critically_warned = true;
    }
}

bool SensorController::get_critically_warned(){
    return critically_warned;
}
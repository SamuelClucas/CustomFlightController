#include "telemetry.h"

#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include <cstdint>

#pragma once
class Act;

class Receiver {
private:
    static constexpr int IBUS_FRAME_SIZE = 32;
    static constexpr int timeout_us = 750000;
    uint8_t ibus_buffer[IBUS_FRAME_SIZE];
    int ibus_index;
    
    uint64_t boot_time;

    uint16_t channels[14];
    float channels_filtered[14];

    uint64_t last_radio_update;

    int throttle;
    int roll;
    int pitch;
    int yaw;
    uint64_t last_byte_time = 0;
    bool syncing = true;
    bool last_switch_state = true;

public:
    Receiver();
    ~Receiver();
    bool has_recent_data() const;

    void read_ibus();
    void filter_channels(Telemetry& telemetry);
    void fail_safe(Telemetry& telemetry, Act& act);

    bool check_switch_armed(Act& act, Telemetry& telemetry);
    bool check_altitude_switch();

    int get_throttle();
    int get_roll();
    int get_pitch();
    int get_yaw();

    int get_channel(int ch);

};
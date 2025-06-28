#include "telemetry.h"
#include "radio_receiver.h"
#include "act.h"
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include <cstdint>

Receiver::Receiver() {
    boot_time = time_us_64();

    uart_init(uart0, 115200);
    gpio_set_function(17, GPIO_FUNC_UART);  // UART0 RX
    gpio_pull_up(17);  // just in case the signal line is weak

    ibus_index = 0;
    last_radio_update = 0;  

    for (int i = 0; i < 14; ++i) {
        channels[i] = 1500;
        channels_filtered[i] = 1500.0f;
    }   
    
}

Receiver::~Receiver() {
    uart_deinit(uart0);
}

void Receiver::filter_channels(Telemetry& telemetry) {
    for (int i = 0; i < 14; i++) {
        channels_filtered[i] = 0.9f * channels_filtered[i] + 0.1f * channels[i];
    }
    throttle = (int)(channels_filtered[2]);
    roll = (int)(channels_filtered[0] - 1500);
    pitch = (int)(channels_filtered[1] - 1500);
    yaw = (int)(channels_filtered[3] - 1500);
    telemetry.update_telemetry("[Throttle]", "%d", throttle);
}

void Receiver::read_ibus() {
    uint64_t now = time_us_64();

    while (uart_is_readable(uart0)) {
        uint8_t b = uart_getc(uart0);
        last_byte_time = now;

        if (syncing) {
            if (b == 0x20) {
                ibus_index = 0;
                ibus_buffer[ibus_index++] = b;
                syncing = false;
            }
            continue;
        }

        ibus_buffer[ibus_index++] = b;

        if (ibus_index == IBUS_FRAME_SIZE) {
            uint16_t chk = 0xFFFF;
            for (int i = 0; i < 30; i++) chk -= ibus_buffer[i];
            uint16_t recv = ibus_buffer[30] | (ibus_buffer[31] << 8);

            if (chk == recv) {
                for (int i = 0; i < 14; ++i) {
                    channels[i] = ibus_buffer[2 + i * 2] | (ibus_buffer[3 + i * 2] << 8);
                }
                last_radio_update = now;
            } 
            ibus_index = 0;
            syncing = true;
        }
    }

    // 🛡️ Resync if stalled mid-frame
    static int failed_count = 0;
    if (++failed_count >= 10) {
        failed_count = 0;
    }
}

void Receiver::fail_safe(Telemetry& telemetry, Act& act) {
    uint64_t now = time_us_64();
    // Ignore failsafe for 5 seconds after boot
    if (now - boot_time < 5000000) return;

    // If we’ve never received a valid iBus frame, don’t trigger failsafe
    if (last_radio_update == 0) return;
    
    now = time_us_64(); // just to be safe, a little gratuitous
    uint64_t time_since_last_update = now - last_radio_update;

    if (time_since_last_update > timeout_us) {
        char buffer[512];
        // Write timestamp
        snprintf(buffer, sizeof(buffer), "[SIGNAL_DEAD] TIME %llu\n [FOR] %llu\n [SINCE] %llu\n", now / 1000, time_since_last_update, last_radio_update);
        telemetry.send_telemetry(buffer);
        act.disarm(telemetry);
    }
}

bool Receiver::check_switch_armed(Act& act, Telemetry& telemetry) {
    if (channels[4] < 1300) {return true;}
    else {
        act.disarm(telemetry);
        return false;
    }
}

bool Receiver::check_altitude_switch() {
    // Prevent false trigger before first valid read
    static bool switch_tracking_started = false;
    static bool last_switch_state = false;

    if (last_radio_update == 0) return false;

    bool current_state = (channels[5] < 1300);

    if (!switch_tracking_started) {
        last_switch_state = current_state;  // prime with first observed value
        switch_tracking_started = true;
        return false;  // don’t trigger on first read
    }

    bool triggered = (!last_switch_state && current_state);  // rising edge only
    last_switch_state = current_state;
    return triggered;
}


int Receiver::get_throttle() {
    return throttle;
}
int Receiver::get_roll() {
    return roll;
}
int Receiver::get_pitch() {
    return pitch;
}
int Receiver::get_yaw() {
    return yaw;
}
int Receiver::get_channel(int ch) {
    if (ch < 0 || ch > 13) return -1;
    return channels_filtered[ch];
}

bool Receiver::has_recent_data() const {
    return (time_us_64() - last_radio_update) < timeout_us;
}

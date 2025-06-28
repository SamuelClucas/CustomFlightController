#include "telemetry.h"
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include <cstdint>
#include <cstdarg>  // for va_list, va_start, va_end
#include <cstdio>   // for vsnprintf 
#include <unordered_map>
#include <string>

Telemetry::Telemetry() {
    uart_init(uart1, 57600);
    gpio_set_function(8, GPIO_FUNC_UART);   // UART1 TX

}

Telemetry::~Telemetry() {uart_deinit(uart1);}

void Telemetry::update_telemetry(const char* key, const char* fmt, ...) {
    if (!key || !fmt) return;

    char value[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(value, sizeof(value), fmt, args);
    va_end(args);

    latest[std::string(key)] = std::string(value);  // overwrites cleanly
}




void Telemetry::flush_telemetry() {
    for (const auto& [key, val] : latest) {
        if (!key.empty() && !val.empty()) {
            uart_puts(uart1, key.c_str());
            uart_puts(uart1, " ");
            uart_puts(uart1, val.c_str());
            uart_puts(uart1, "\r\n");
        }
    }
    latest.clear();  // wipe map after flush
}





void Telemetry::send_telemetry(const char* fmt, ...) {
    char buffer[256];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    buffer[len++] = '\r';
    buffer[len++] = '\n';
    buffer[len] = '\0';

    for (int i = 0; i < len; ++i) {
        while (!uart_is_writable(uart1));
        uart_putc_raw(uart1, buffer[i]);
    }
}






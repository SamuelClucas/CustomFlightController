// telemetry.h
#include <cstdint>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include <cstdarg>  // for va_list, va_start, va_end
#include <cstdio>   // for vsnprintf 
#include <unordered_map>
#include <string>
#include <cstring>

#pragma once

class Telemetry {
private:
    // Add this in telemetry.h
    private:
    std::unordered_map<std::string, std::string> latest;


public:
    Telemetry();
    ~Telemetry();
    
    void flush_telemetry();
    void update_telemetry(const char* key, const char* fmt, ...);
    void send_telemetry(const char* fmt, ...);
};

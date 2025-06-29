#include <iostream>
#include <unordered_map>
#include <string>
#include <chrono>
#include <thread>
#include <fstream>
#include <termios.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#pragma once

class Receiver {
    private:
    enum class State {
        ONLINE,
        PARSING,
    };

    State currentState;
    static Receiver* instance;  

    public:
    int fd;
    char buf[512];
    int n; // stores read 
    struct termios tty{}; // termios struct used to reconfigure character device attributes


    Receiver();
    ~Receiver();
    static void handleInterrupt(int signal);
    void parsing();
    void shutdown();
    std::string to_string(State s);
    State announce();
};
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

class Amygdala {
    private:
    enum class State {
        AWAKE,
        READING,
    };

    State currentState;
    static Amygdala* instance;  

    public:
    int fd;
    char buf[512];
    int n; // stores read 
    struct termios tty{}; // termios struct used to reconfigure character device attributes


    Amygdala();
    ~Amygdala();
    static void handleInterrupt(int signal);
    void reading();
    void rest();
    std::string to_string(State s);
    State announce();
};
#include <iostream>
#include <unordered_map>
#include <string>
#include <chrono>
#include <thread>
#include "amygdala.h"
#include <fstream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <csignal>
#include <atomic>

extern std::atomic<bool> conscious;  // shared with main
Amygdala* Amygdala::instance = nullptr;

void Amygdala::handleInterrupt(int signal) {
    conscious = false;
}

Amygdala::Amygdala() {
    instance = this;
    currentState = State::AWAKE;
    std::signal(SIGINT, handleInterrupt);
}

Amygdala::~Amygdala() {
}

void Amygdala::rest(){
    std::cout << "\033[1;31mThe amygdala now rests...\033[0m" << std::endl;
}

void Amygdala::reading() {
    currentState = State::READING;

    fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        std::cerr << "\033[0;31mFailed to open file descriptor...\033[0m" << std::endl;
        return;
    }

    std::cout << "\033[0;32mSuccessfully opened file descriptor!\033[0m" << std::endl;

    if (tcgetattr(fd, &tty) != 0) {
        std::cout << "tcgetattr failed" << std::endl;
        close(fd);
        fd = -1;
        return;
    }

    // Set baud rate and 8N1 mode
    cfsetispeed(&tty, B57600);
    cfsetospeed(&tty, B57600);
    tty.c_cflag |= (CLOCAL | CREAD);  // enable receiver
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;               // 8-bit characters
    tty.c_cflag &= ~PARENB;           // no parity
    tty.c_cflag &= ~CSTOPB;           // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;          // no hardware flow control

    //tcflush(fd, TCIFLUSH);
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cout << "tcsetattr failed" << std::endl;
        close(fd);
        fd = -1;
        return;
    }

    fcntl(fd, F_SETFL, 0);
    std::cout << "Now reading..." << std::endl;
    while (conscious) {
        n = read(fd, buf, sizeof(buf) - 1);
        if (n > 0) {
            buf[n] = '\0';
            std::cout << "\033[1;36m" << buf << "\033[0m" << std::flush;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // avoid tight loop
    }

    std::cout << "\n\033[1;33mExiting reading loop.\033[0m\n";
    close(fd);
    fd = -1;
}


std::string Amygdala::to_string(Amygdala::State s) {
    switch(s) {
        case State::AWAKE: return "AWAKE";
        case State::READING: return "READING";
        default: return "UNKNOWN";
    }
}

Amygdala::State Amygdala::announce(){
    std::cout << "\033[1;32mAmygdala is " << to_string(currentState) << "\033[0m" << std::endl;
    return currentState;
}


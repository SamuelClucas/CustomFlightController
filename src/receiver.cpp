#include <iostream>
#include <unordered_map>
#include <string>
#include <chrono>
#include <thread>
#include "receiver.h"
#include <fstream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <csignal>
#include <atomic>

extern std::atomic<bool> online;  // shared with main
Receiver* Receiver::instance = nullptr;

void Receiver::handleInterrupt(int signal) {
    online = false;
}

Receiver::Receiver() {
    instance = this;
    currentState = State::ONLINE;
    std::signal(SIGINT, handleInterrupt);
}

Receiver::~Receiver() {
}

void Receiver::shutdown(){
    std::cout << "\033[1;31mReceiver shutting down...\033[0m" << std::endl;
}

void Receiver::parsing() {
    currentState = State::PARSING;

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
    std::cout << "Parsing incoming signal..." << std::endl;
    while (online) {
        n = read(fd, buf, sizeof(buf) - 1);
        if (n > 0) {
            buf[n] = '\0';
            std::cout << "\033[1;36m" << buf << "\033[0m" << std::flush;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // avoid tight loop
    }

    std::cout << "\n\033[1;33mExiting parsing loop.\033[0m\n";
    close(fd);
    fd = -1;
}


std::string Receiver::to_string(Receiver::State s) {
    switch(s) {
        case State::ONLINE: return "ONLINE";
        case State::PARSING: return "PARSING";
        default: return "UNKNOWN";
    }
}

Receiver::State Receiver::announce(){
    std::cout << "\033[1;32mReceiver state: " << to_string(currentState) << "\033[0m" << std::endl;
    return currentState;
}


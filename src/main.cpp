#include <iostream>
#include <unordered_map>
#include <string>
#include <csignal> // defines SIGINT constant
#include <chrono>
#include <thread>
#include "receiver.h"
#include <atomic>
#include "shared_state.h"

std::atomic<bool> online(true);  // actual definition

void handleInterrupt(int signal){
    online = false;
}

class GroundControl {
    public:
    Receiver receiver = Receiver();

    GroundControl() {
        std::cout << "\033[1;32mGround control online...\033[0m" << std::endl;
    }
    ~GroundControl(){
        receiver.shutdown();
        std::cout << "\033[1;31mGround control offline...\033[0m" << std::endl;
    }
};

int main(){
    std::signal(SIGINT, handleInterrupt); // passes interrupt function to std::signal when SIGINT received for process
    GroundControl groundControl = GroundControl();
    groundControl.receiver.announce();
    groundControl.receiver.parsing();
    while (online){
        std::cout << "Main loop" << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    return 0;
}
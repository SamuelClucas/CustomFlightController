#include <iostream>
#include <unordered_map>
#include <string>
#include <csignal> // defines SIGINT constant
#include <chrono>
#include <thread>
#include "amygdala.h"
#include <atomic>
#include "shared_state.h"

std::atomic<bool> conscious(true);  // actual definition

void handleInterrupt(int signal){
    conscious = false;
}

class Observer {
    public:
    Amygdala amygdala = Amygdala();

    Observer() {
        std::cout << "\033[1;32mThe Observer now wakes...\033[0m" << std::endl;
    }
    ~Observer(){
        amygdala.rest();
        std::cout << "\033[1;31mThe Observer now rests...\033[0m" << std::endl;
    }
};

int main(){
    std::signal(SIGINT, handleInterrupt); // passes interrupt function to std::signal when SIGINT received for process
    Observer observer = Observer();
    observer.amygdala.announce();
    observer.amygdala.reading();
    while (conscious){
        std::cout << "Main loop" << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    return 0;
}
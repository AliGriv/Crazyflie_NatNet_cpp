//
// Created by AliGriv on 2021-02-09.
//

#include <iostream>

#include <inttypes.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <thread>
#include <mutex>
#include "NatNetUtility.h"
#include "NatNetClient.h"

std::mutex my_mutex;
int run_result;

void run_NatNet_thread() {
    run_result = run_NatNet(1, std::vector<char*>(1,NULL));
}


void print_data_thread() {
    while(IsRunning) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::lock_guard<std::mutex> guard(my_mutex);
        for (int i {0}; i < get_data().size(); i++) {
            std::cout << "x: " << get_data().at(i).pos.at(0)
                      << "y: " << get_data().at(i).pos.at(1)
                      << "z: " << get_data().at(i).pos.at(2) << std::endl;
        }
    }
}

int main() {
//    NatNetClient* g_pClient2;
//    g_pClient2 = create_NatNetClient();

//    client_NatNet g_pClient2;
//    g_pClient2.initialization_flag = 0;
//    g_pClient2.run();
//      int i = run_NatNet(1, std::vector<char*>(1,NULL));
//      std::cout << "data_vec size is " << get_data_size() << std::endl;


//    delete g_pClient2;


    std::thread run_thread (run_NatNet_thread);
//    std::thread print_thread (print_data_thread);


    run_thread.join();
//    print_thread.join();
    std::cout << "threads joined! cleaning.... " <<std::endl;
    clean_data();
    return 0;
}
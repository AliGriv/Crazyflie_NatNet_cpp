//
// Created by AliGriv on 2021-02-09.
//

#ifndef NATNET_EXAMPLE_NATNETUTILITY_H
#define NATNET_EXAMPLE_NATNETUTILITY_H

#include <iostream>
#include <vector>
#include "NatNetTypes.h"
#include "NatNetCAPI.h"
#include "NatNetClient.h"

class data_NatNet;

int run_NatNet( int argc, std::vector<char*> argv );
std::vector<data_NatNet> get_data();
int get_data_size();

std::vector <std::vector <data_NatNet>>* data_vec = new std::vector <std::vector <data_NatNet>>;
bool IsRunning = false;



class data_NatNet {
public:
    std::vector<double> pos{std::vector<double>(3,0.0)};
    std::vector<double> orient{std::vector<double>(4,0.0)};
    bool bTrackingValid;
    int ID;
};
data_NatNet* test_data = new data_NatNet;

void clean_data(){
    delete data_vec;
    delete test_data;
}


class test_class {
public:
    void do_something();
};





















#endif //NATNET_EXAMPLE_NATNETUTILITY_H

//
// Created by AliGriv on 2021-02-23.
//

/* This program tries to connect to a crazyflie with given uri and ramp-up and ramp-down the motors */
#include <iostream>
#include <chrono>
#include <thread>

#include <crazyflie_cpp/Crazyflie.h>
#include "Velocity_Filter.h"
#include "Sensor.h"
#include <Eigen/Dense>
#include "CSVWriter.h"
#include "Recorder.h"
#include <cmath>
#include "signal.h"
#include "PB_Control.h"

bool stop = false;

std::vector <double> find_highLC(const Eigen::VectorXd &errors, const Eigen::VectorXd &gain) {
    double ux = gain(0) * errors(0) + gain(3) * errors(3);
    double uy = gain(1) * errors(1) + gain(4) * errors(4);
    double uz = gain(2) * errors(2) + gain(5) * errors(5);
    return (std::vector <double> {ux, uy, uz});
}

// Define the function to be called when ctrl-c (SIGINT) is sent to process
void signal_callback_handler(int signum) {
    std::cout << "Caught signal " << signum << std::endl;
    // Terminate program
//    exit(signum);
    stop = true;
}
int sign(double variable) {
    return ((0.0 <= variable) - (variable < 0.0));
}

int main() {


    double remainder = fmod(0.05, M_PI);
    std::cout << "remainder is " << remainder << std::endl;
    std::cout << "atan(1) is " << atan(1.0) << std::endl;
    std::cout << "fabs(-5.6) is " << fabs(-5.6) << std::endl;
    std::cout << "sign(0.0) is " << sign(0.0) << std::endl;
    PB_Control ctrl(1);
    std::cout << ctrl.saturate(1.5,1.0) << std::endl;
//    CSVWriter csv;
//    auto t0 = std::chrono::high_resolution_clock::now();
//    signal(SIGINT, signal_callback_handler);
//    while(!stop){
//        std::cout << "Program processing..." << std::endl;
//        auto t = std::chrono::high_resolution_clock::now();
//        auto duration = std::chrono::duration_cast<std::chrono::duration<double>>(t-t0);
//        double dd = duration.count();
//        std::cout << dd << std::endl;
//        sleep(1);
//    }
//    return EXIT_SUCCESS;



//    std::string uri {"radio://0/80/2M/E7E7E7E7E6"};
//    Crazyflie cf(uri);
//    cf.logReset();
//    std::cout << "reboot" <<std::endl;
//    cf.reboot();
//    std::this_thread::sleep_for(std::chrono::seconds(3));
//    cf.sendSetpoint(0.0,0.0,0.0,0);
//    std::cout << "let's ramp up:" << std::endl;
//    for (uint16_t i {5000}; i <= 10000; i++) {
////        cf.sendSetpoint(0.0,0.0,0.0,2*i);
//
//        std::cout << "thrust: " << 2*i << std::endl;
//        if (cf.IsConnected()){
//            cf.sendSetpoint(0.0,0.0,0.0,2*i);
////            std::cout << "Connected" << std::endl;
//        }
//        else {
//            std::cout << "Connection Lost" << std::endl;
//        }
//        std::this_thread::sleep_for(std::chrono::milliseconds(10));
//    }

//    Eigen::MatrixXd temp(3,4);
//    temp << 1, 2, 3, 4,
//            4, 5, 6, 7,
//            7, 8, 9, 0;
//    std::cout << temp << std::endl;
//    Velocity_Filter v_fil;
//    Eigen::Vector3d t;
//    t = Eigen::Vector3d::Zero();
//    std::cout << t << std::endl;
//    std::vector <bool> t_bool;
//    t_bool.push_back(false);
//    t_bool.push_back(true);
//    t_bool.push_back(false);
//    std::cout << "Size of t_bool is: " << t_bool.size() << std::endl;
//
//    std::cout << "t(1) is " << t(1) << std::endl;
//    std::cout << "t_bool.at(1) is " << t_bool.at(1) << std::endl;
//
//    Sensor sensor(3);
//    std::vector <Sensor> sensor_vec;
//    sensor_vec = std::vector <Sensor> (3, Sensor(3));
//    Eigen::VectorXd kPos_i (6);
//    kPos_i << 0.35,0.35,0.9,0.15,0.15,0.2;
//    Eigen::VectorXd error (6);
//    error << 0.35,0.35,0.9,0.15,0.15,0.2;
//    std::cout << kPos_i << std::endl;
////    std::vector <double> highLC {1.0,2.0,3.0};
////    std::cout << "highLC size is " << highLC.size() << std::endl;
////    std::cout << "highLC[1] is " << highLC[1] << std::endl;
//    std::vector <double> highLC;
//    highLC = find_highLC(error, kPos_i);
//    std::cout << highLC[0] << " " << highLC[1] << " " << highLC[2] << std::endl;
//
//    std::cout << "int(-24.2) is " << int(-24.2) << std::endl;
//
//    std::vector <std::vector <double>> setPointCoordinates {{0.0,0.0,0.0,0.0} ,{0.0,0.0,1.5,5.0}, {0.0,0.0,1.5,10.0}, {0.0,0.0,0.0,5.0}};
//    Eigen::Vector3d vec3d (1,2,3);
//    std::cout << vec3d << std::endl;
//
//    int a = 'y' - 'x';
//    std::cout << "a is " << a << std::endl;
//    std::string aa = "aa"; aa += 'x';
//    std::cout << aa << " " << aa + 'x' << std::endl;
//    std::cout << aa + aa << std::endl;
//
//    Recorder rec(3);

//    for (int n=0; n < 3; n++){
//        for (int i = 0; i < 1000; i++) {
//            double t = i * 0.01;
//            rec.appendTime(t, n);
//            Eigen::Vector3d p;
//            p << sin(M_PI / 4 * t) + n, cos(M_PI / 3 * t) + n, t+n;
//            rec.appendDesiredPosition(p, n);
//        }
//    }
//    rec.generatePlots();

//    rec.saveDataToFile();
//    rec.printVariableNames();
    return 0;
}

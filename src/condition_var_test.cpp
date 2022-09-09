//
// Created by AliGriv on 2021-03-21.
//

#include <iostream>
#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <Eigen/Dense>
#include <cmath>
#include <chrono>
std::vector <Eigen::Vector3d> positions;
std::vector <std::vector <Eigen::Vector3d>> command_storage;
int num_copters;
std::mutex m_event;
std::condition_variable cv_event;
bool ready_event = false;
std::mutex m;
std::mutex m_in_main;
bool finish_it = false;

void receiveRigidBodyFrame() {
    double step = 0.0;
    int num_iter = 0;
    while (num_iter < 1000){
        {
            std::lock_guard<std::mutex> lk(m_event);
            std::lock_guard<std::mutex> lg(m);
            for (int i = 0; i < num_copters; ++i) {
                positions.at(i)(0) = 5.0 * sin(M_PI * step);
                positions.at(i)(1) = 5.0 * sin(M_PI * step);
                positions.at(i)(2) = 5.0 * sin(M_PI * step);
            }
            ready_event = true;
            cv_event.notify_one();
        }
        step += 0.01;
        num_iter++;
        std::this_thread::sleep_for(std::chrono::milliseconds(8));
    }
    finish_it = true;
    ready_event = true;
    cv_event.notify_one();
    return;
}

void mainThread_run(){
    int loop_counter = 0;
    double kx = 1.0;
    double ky = 2.0;
    double kz = 3.0;
    std::vector <Eigen::Vector3d> position_cache;
    std::vector <Eigen::Vector3d> temp (num_copters, Eigen::Vector3d::Zero());
    std::chrono::high_resolution_clock::time_point expInitTime;
    std::chrono::high_resolution_clock::time_point time_temp;
    expInitTime = std::chrono::high_resolution_clock::now();
    while (!finish_it) {
        std::unique_lock<std::mutex> lk(m_event);
        cv_event.wait(lk, []{return ready_event;});
        position_cache = positions;

        for (int i=0; i< num_copters; i++) {
            temp.at(i)(0) = kx*position_cache.at(i)(0);
            temp.at(i)(1) = ky*position_cache.at(i)(1);
            temp.at(i)(2) = kz*position_cache.at(i)(2);
        }
        m_in_main.lock();
        command_storage.push_back(temp);
        m_in_main.unlock();
        loop_counter++;
        if (loop_counter % 100 == 0) {
            time_temp = std::chrono::high_resolution_clock::now();
            auto time_span = std::chrono::duration_cast<std::chrono::duration<double>>(time_temp - expInitTime);

//                        std::cout << "time_span.count()" << time_span.count() << std::endl;
            std::cout << "Average loop rate (main thread) is " << loop_counter / (time_span.count()) << "Hz"
                      << std::endl;
        }
        ready_event = false;
        lk.unlock();
    }
    return;
}


int main () {
    num_copters = 3;
    for (int i=0;i < num_copters; i++) {
        positions.push_back(Eigen::Vector3d::Zero());
    }

    std::thread th1(receiveRigidBodyFrame);
    std::thread th2(mainThread_run);


    th1.join();
    th2.join();

    std::cout << "the command_storage.size() is " << command_storage.size() << std::endl;

    std::this_thread::sleep_for(std::chrono::seconds (3));

    std::cout << "command_storage.at(i)(0) for i < 10 are" << std::endl;
    for (int i = 0; i < 10; i++) {
        std::cout << command_storage.at(i).at(0).transpose() << std::endl;
    }
    return 0;
}


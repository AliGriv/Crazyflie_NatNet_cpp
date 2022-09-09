//
// Created by AliGriv on 2021-03-06.
//

#ifndef CRAZYFLIE_NATNET_SENSOR_H
#define CRAZYFLIE_NATNET_SENSOR_H

#include "Velocity_Filter.h"
#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <math.h>
class Sensor {
public:
    // Failsafe Related
    std::vector <int> trackLostCounter;
    int FailSafeLostCounts;
    bool FAILSAFE_FLAG; //INTERFACE ATTRIBUTE
    // FeedBack Related
    bool initFlag;
    Eigen::Vector3d initLeaderPosition;
    std::vector <Eigen::Vector3d> Position; //INTERFACE ATTRIBUTE. A vector of eigen::vec; each of sublist contains the position of a copter in the world frame. Initializes based on camera measurements.
    std::vector <Eigen::Vector3d> Velocity; //INTERFACE ATTRIBUTE. Contains all velocity vectors of all copters. vector of eigen_vec. Initializes at zeros.
    std::vector <Eigen::Vector3d> tempVel;
    // Yaw Related
    std::vector <double> initYaw;
    std::vector <double> yaw;
    std::vector <int> yawCounter;
    std::vector <double> yawFiltered; //INTERFACE ATTRIBUTE. Initializes based on camera measuremnts.
    std::vector <Velocity_Filter> firFilter;
    double nonlinFilterThreshold = 0.1; // m/s


    /* some useful variables */
    double q0, q1, q2, q3;
    double r11, r31;
    double yaww;
    Eigen::VectorXd tempVell;


    Sensor(int numCopters) {
        FailSafeLostCounts = 10;
        FAILSAFE_FLAG = false;
        initFlag = false;
        initLeaderPosition = Eigen::Vector3d::Zero();
        Position = std::vector <Eigen::Vector3d> (numCopters, Eigen::Vector3d::Zero());
        Velocity = std::vector <Eigen::Vector3d> (numCopters, Eigen::Vector3d::Zero());
        tempVel = std::vector <Eigen::Vector3d> (numCopters, Eigen::Vector3d::Zero());
        for (int i = 0; i < numCopters; i++) {
            yaw.push_back(0.0);
            yawCounter.push_back(0); //Yaw conters initialized at 0s.
            yawFiltered.push_back(0.0); //Yaw will be initialized with the actual measurements later.
            trackLostCounter.push_back(0);
            firFilter.push_back(Velocity_Filter(5, 0.05, 3));
        }
    }

    void failsafe(const std::vector <bool> trackingFlag) {
        for (int i = 0; i < trackingFlag.size(); i++) {
            if (!trackingFlag.at(i)) {
                trackLostCounter.at(i) += 1;
                if (trackLostCounter.at(i) > FailSafeLostCounts) {
                    FAILSAFE_FLAG = true;
                }
            }
            else {
                trackLostCounter.at(i) = 0;
            }
        }
    }

    void find_yaw(const std::vector <Eigen::Vector4d> &orientations, const std::vector <bool> &trackingFlag) {
        // The last element of orientation is the scalar one.
        // "Extraction of the copter's yaw wrt the world frame from the camera measured quaternions"
        for (int i = 0; i < orientations.size(); i++) {
            if (trackingFlag.at(i)) {
                q0 = orientations.at(i)(3);
                q1 = orientations.at(i)(0);
                q2 = orientations.at(i)(1);
                q3 = orientations.at(i)(2);
                r11 = 1 - 2 * (pow(q2,2) + pow(q3,2));
                r31 = 2 * (q1 * q3 - q0 * q2);
                yaww = atan2(-r31,r11);
                // initializing yawFiltered
                if (!initFlag) {
                    yawFiltered.at(i) = yaww;
                }
                // Compensating for yaw overflows
                if (yaww - yaw.at(i) < -5.25) { //approx -300 degrees
                    yawCounter.at(i)++;
                }
                else if (yaww - yaw.at(i) > 5.25){
                    yawCounter.at(i)--;
                }
                yaw.at(i) = yaww;
                yaww = yaww + 2 * M_PI * yawCounter.at(i);
                //Filtering possible spikes
                if(fabs(yaww - yawFiltered.at(i) < 0.05)) {
                    yawFiltered.at(i) = yaww;
                }
            }
        }
    }

    void setPosition(const std::vector <Eigen::Vector3d> &Positions, const std::vector <bool> &trackingFlag) {
        // Sets all positions by accounting for the initial position offset. Leader is placed in the origin of the world frame!
        for (int i = 0; i < Positions.size(); i++) {
            if (trackingFlag.at(i)) {
                Position.at(i) = Positions.at(i) - initLeaderPosition;
            }
        }
    }


    void estimateVel() {
        for (int i = 0; i < Position.size(); i++) {
            tempVell = firFilter.at(i).numDiff_FIR(Position.at(i));
            for (int j = 0; j < tempVell.size(); j++) {
                if (fabs(tempVel.at(i)(j) - tempVell(j)) < nonlinFilterThreshold) {
                    Velocity.at(i)(j) = tempVell(j);
                }
                tempVel.at(i)(j) = tempVell(j);
            }
        }
    }


    void process(const std::vector <Eigen::Vector3d> &Positions,
                 const std::vector <Eigen::Vector4d> &orientations,
                 const std::vector <bool> &trackingFlag) {
        //Initializing measurements
        if (!initFlag) {
            bool allTracked = true;
            for (int i = 0; i < trackingFlag.size(); ++i) {
                allTracked = allTracked && trackingFlag.at(i);
            }
            if (allTracked) {
                initLeaderPosition = Positions.at(0); //Reading the leader initial position
                setPosition(Positions, trackingFlag); //Initializing positions
                find_yaw(orientations, trackingFlag); //Initializing yaws
                initFlag = true;
                std::cout << "Copter position/orientation coordinates initialized" << std::endl;
            }
        }
        find_yaw(orientations, trackingFlag); //yawFiltered gets updated only for the copters that have been tracked
        setPosition(Positions, trackingFlag); //self.Position gets updated only for the copters that have been tracked
        estimateVel();                       //self.Velocity gets updated
        failsafe(trackingFlag); //If any of the copters is lost for more than 10 consecutive samples, the system goes into a failsafe mode.
    }

};




#endif //CRAZYFLIE_NATNET_SENSOR_H

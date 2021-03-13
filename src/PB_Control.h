//
// Created by AliGriv on 2021-03-07.
//

#ifndef CRAZYFLIE_NATNET_PB_CONTROL_H
#define CRAZYFLIE_NATNET_PB_CONTROL_H

#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <math.h>


struct Command {
    double roll;
    double pitch;
    int throttle;
    double yawRate;
    Command() {
        roll = 0.0;
        pitch = 0.0;
        throttle = 0;
        yawRate = 0.0;
    }
};


class PB_Control {
    //"This is an implementation of a high-level position controller"
public:

    std::vector <Eigen::VectorXd> kPos; // [[kpx, kpy, kpz, kdx, ...], [], ...]. This list includes the PD gains of all copters as well as the nominal gravity compensating thrust. Each copter gains are given in a sublist.
                                        // Kp, Kd, Throttle_bias. Exp: 0.3 is mapped to around 1400 in RC commands. [[0.6,0.4,0.6,0.4,0.35,0.13], [0.6,0.4,0.6,0.4,0.5,0.2], [0.6,0.4,0.6,0.4,0.5,0.2]]
    /* Define gains manually in the setGains() */

    double Throttle_bias; //(0 to 1) percentage throttle
    double yawKp; // 1/sec
    double maxYawRate; // rad/sec
    double maxRoll;
    double maxPitch;
    double maxThrottle;
    /* To change the values check the constructor */

    std::vector <double> throttle; //in range (0,1). 1 being max throttle available and 0 being no throttle.
    std::vector <double> roll; //rad
    std::vector <double> pitch; //rad
    std::vector <double> yawRate; //TBD
    std::vector <Eigen::Vector3d> fXYZ;

    // Commands for the crazyflie
    const int MAX_THROTTLE_CF = 65000;
    const int MIN_THROTTLE_CF = 20000;
    std::vector <int> throttleCF;
    std::vector <double> rollCF;
    std::vector <double> pitchCF;
    std::vector <double> yawRateCF;

    std::vector <Command> mappedCommands;

    std::vector <double> desiredYaw;
    bool initYawFlag;


    double initTime;
    int phase;

    int num_copters;



    double epsilon = 0.000001;
    double delta = M_PI/720;

    PB_Control(int numCopters):
    num_copters(numCopters) {
        Throttle_bias = 0.45;
        yawKp = 0.1;
        maxYawRate = 1;
        maxRoll = M_PI/4;
        maxPitch = M_PI/4;
        maxThrottle = 0.99;
        initYawFlag = false;
        throttle = std::vector <double> (numCopters, 0.0);
        roll = std::vector <double> (numCopters, 0.0);
        pitch = std::vector <double> (numCopters, 0.0);;
        yawRate = std::vector <double> (numCopters, 0.0);
        throttleCF = std::vector <int> (numCopters, 0);
        rollCF = std::vector <double> (numCopters, 0.0);
        pitchCF = std::vector <double> (numCopters, 0.0);
        yawRateCF = std::vector <double> (numCopters, 0.0);
        mappedCommands = std::vector <Command> (numCopters, Command());
        desiredYaw = std::vector <double> (numCopters, 0.0);
        fXYZ = std::vector <Eigen::Vector3d> (numCopters, Eigen::Vector3d::Zero());
        initTime = 0.0;
        phase = 0;


        setGains();
        if (kPos.size() != numCopters) {
            std::cerr << "Define gains manually. The number of gains is not equal to the number of copters" << std::endl;
        }
    }





    /* Please define your gains here :D */
    void setGains(){ //Define and push_back as many as you need
        Eigen::VectorXd kPos_i (6);
        kPos_i << 0.35,0.35,0.9,0.15,0.15,0.2;
        kPos_i = 40 * kPos_i;
        kPos.push_back(kPos_i);
    }


    void ramp_up( double lb, double ub, double duration, double timer) {
        // ramps up from lower bound (lb) to upper bound (ub) in a given time duration
        timer = timer - initTime;
        double throttleTemp = (ub - lb) * timer / duration + lb;
        for (int i = 0; i < num_copters; i++) {
            throttle.at(i) = throttleTemp;
            roll.at(i) = 0.0;
            pitch.at(i) = 0.0;
        }
    }

    void ramp_down ( double lb, double ub, double duration, double timer) {
        timer = timer - initTime;
        double throttleTemp = (lb - ub) * timer / duration + ub;
        for (int i = 0; i < num_copters; i++) {
            throttle.at(i) = throttleTemp;
            roll.at(i) = 0.0;
            pitch.at(i) = 0.0;
        }
    }

    Eigen::Vector3d find_highLC(const Eigen::VectorXd &errors, const Eigen::VectorXd &gain) {
        Eigen::Vector3d u;
        u(0) = gain(0) * errors(0) + gain(3) * errors(3);
        u(1) = gain(1) * errors(1) + gain(4) * errors(4);
        u(2) = gain(2) * errors(2) + gain(5) * errors(5);
        return u;
    }

    std::vector <double> find_lowLC(const Eigen::Vector3d &highLC, const double &yaw_val) {
        // Finding throttle
        double throttle_temp = highLC.norm();
        // Finding roll and pitch
        double roll_temp;
        double pitch_temp;
        double n_roll;
        double d_roll;
        double n_pitch, d_pitch;

        double remainder = fmod(yaw_val, 2*M_PI);
        if ((fabs(remainder - M_PI) <= delta) || (fabs(remainder + M_PI) <= delta)) {
            roll_temp = asin(highLC(1)/throttle_temp);
            pitch_temp = -atan(highLC(0)/highLC(2));
        }
        else {
            // n_roll = (math.tan(yaw/2)**2 * throttle - (ux**2 * math.tan(yaw/2)**4 - 2 * ux**2 * math.tan(yaw/2)**2 + 4 * uy**2 * math.tan(yaw/2)**2 + 2 * uz**2 * math.tan(yaw/2)**2 + uz**2 * math.tan(yaw/2)**4 + ux**2 + uz**2 + 4 * ux * uy * math.tan(yaw/2) - 4 * ux * uy * math.tan(yaw/2)**3)**(1/2) + throttle)
            n_roll = (pow(tan(yaw_val/2),2)*throttle_temp - sqrt(pow(highLC(0),2) + pow(tan(yaw_val/2),4) - 2*pow(highLC(0),2)*pow(tan(yaw_val/2),2) + 4 * pow(highLC(1),2) * pow(tan(yaw_val/2),2) + 2 * pow(highLC(2),2) * pow(tan(yaw_val/2),2) + pow(highLC(2),2) * pow(tan(yaw_val/2),4) + pow(highLC(0),2) + pow(highLC(2),2) + 4 * highLC(0) * highLC(1) * tan(yaw_val/2) - 4 * highLC(0) * highLC(1) * pow(tan(yaw_val/2),3))* + throttle_temp);
            d_roll = pow(highLC(1)*tan(yaw_val/2),2) - highLC(1) + 2*highLC(0)*tan(yaw_val/2);
            if (fabs(d_roll) < epsilon) {
                roll_temp = 0.0;
            }
            else {
                roll_temp = 2*atan(n_roll/d_roll);
            }
            // Finding Pitch
            n_pitch = (highLC(2) - sqrt(pow(pow(highLC(0),2),4) - 2 * pow(highLC(0),2) * pow(tan(yaw_val/2),2) + 4 * pow(highLC(1),2) * pow(tan(yaw_val/2),2) + 2 * pow(highLC(2),2) * pow(tan(yaw_val/2),2) + pow(highLC(0),2) + pow(highLC(2),2) + 4 * highLC(0) * highLC(1) * tan(yaw_val/2) - 4 * highLC(0) * highLC(1) * pow(tan(yaw_val/2),3) + highLC(2)) * pow(tan(yaw_val/2),2) );
            d_pitch = (highLC(0) - highLC(0) * pow(tan(yaw_val/2),2) + 2 * highLC(1) * tan(yaw_val/2));
            if (fabs(d_pitch) < epsilon) {
                pitch_temp = 0;
            }
            else{
                pitch_temp = -2 * atan(n_pitch/d_pitch);
            }
        }
        return std::vector <double> {throttle_temp, roll_temp, pitch_temp};
    }

    int sign(double variable) {
        return ((0.0 <= variable) - (variable < 0.0));
    }

    double saturate(double variable, double maxVal) {
        if (variable >  fabs(maxVal) || -variable > fabs(maxVal)) {
            variable = fabs(maxVal) * sign(variable);
        }
        return variable;
    }

    void map_commands() {
        // " This maps the commands to what crazyflie is expecting to receive"
        for (int i=0; i < num_copters; i++) {
            throttleCF.at(i) = int(throttle.at(i) * (MAX_THROTTLE_CF - MIN_THROTTLE_CF) + MIN_THROTTLE_CF);
            rollCF.at(i) = (roll.at(i) * (180 / M_PI));
            pitchCF.at(i) = (pitch.at(i) * (180 / M_PI));
            yawRateCF.at(i) = (-yawRate.at(i) * (180 / M_PI));
            mappedCommands.at(i).roll = rollCF.at(i);
            mappedCommands.at(i).pitch = pitchCF.at(i);
            mappedCommands.at(i).throttle = throttleCF.at(i);
            mappedCommands.at(i).yawRate = yawRateCF.at(i);
        }
    }


    void control_allocation(const double &timer, const std::vector <double> &yaw_val,
                            const std::vector <Eigen::VectorXd> &errors,
                            int phase_val, const double &rampUpDuration, const double &rampDownDuration) {
        if (!initYawFlag) {
            desiredYaw = yaw_val;
            initYawFlag = true;
        }
        //Resetting the initTime if the phase has changed
        if (phase != phase_val) {
            initTime = timer;
        }
        // update controller phase
        phase = phase_val;
        //Remaining on the floor
        if (phase == 0) {
            for (int i = 0; i < num_copters; i++) {
                throttle.at(i) = 0.0;
                roll.at(i) = 0.0;
                pitch.at(i) = 0.0;
            }
        }
        else if (phase == 1) {
            ramp_up(0.0, Throttle_bias, rampUpDuration, timer);
        }
        else if (phase == 2) {
            // Controller is in the loop now!
            for (int i = 0; i < num_copters; i++) {
                Eigen::Vector3d highLC = find_highLC(errors.at(i), kPos.at(i));
                double sat_ux = saturate(highLC[0], Throttle_bias);
                double sat_uy = saturate(highLC[1], Throttle_bias);
                double sat_uz = saturate(highLC[2], 2 * Throttle_bias) + Throttle_bias; //TO DO for adaptive gravity compensation
                Eigen::Vector3d sat_highLC;
                sat_highLC << sat_ux, sat_uy, sat_uz;
                fXYZ.at(i) = sat_highLC;
                //Find Low-level commands
                std::vector <double> lowLC = find_lowLC(sat_highLC, yaw_val.at(i));
                throttle.at(i) = saturate(lowLC.at(0), maxThrottle);
                roll.at(i) = saturate(lowLC.at(i), maxRoll); // This is in radians.
                pitch.at(i) = saturate(lowLC.at(2), maxPitch); // This is in radians.
                yawRate[i] = saturate(yawKp * (0 - yaw_val.at(i)), maxYawRate); //Radian per second
            }
        }
        else if (phase == 3) {
            ramp_down(0.0, Throttle_bias, rampDownDuration, timer);
        }
        map_commands();
    }


};














#endif //CRAZYFLIE_NATNET_PB_CONTROL_H

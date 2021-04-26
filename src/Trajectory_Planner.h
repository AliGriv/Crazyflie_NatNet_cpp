//
// Created by AliGriv on 2021-03-07.
//

#ifndef CRAZYFLIE_NATNET_TRAJECTORY_PLANNER_H
#define CRAZYFLIE_NATNET_TRAJECTORY_PLANNER_H

#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <math.h>


struct PointWithTime {
    double x,y,z,t;
    PointWithTime() {
        x = 0.0;
        y = 0.0;
        z = 0.0;
        t = 0.0;
    }
    PointWithTime(double x_val, double y_val, double z_val, double t_Val){
        x = x_val;
        y = y_val;
        z = z_val;
        t = t_Val;
    }

};

class Trajectory_Planner {

public:
    double deadTime = 0;   // During deadtime operation phase remains 0 which means no ramp up no closed-loop control
    double rampUpDuration = 2; //During this period operation phase is 1 and props ramp up still no closed-loop control
    double trajStartDelay = 2; //During this period operation phase is 2 (closed-loop control) but trajectories have not started yet. (Desired position is set to [0, 0, 0])
    double rampDownDuration = 0; //During this period operation phase is 3 and props ramp down no closed-loop control.
    double expTimeTotal = 0; //The totall time from expTime = 0 (absolute zero) to the end of trajectories (landing).
    int trajType = 2; //set to 1 for set points and to 2 for smooth polynomials.
    int phase = 0;

    // setPointCoordinates is a vector of vectos in shape {x,y,z,t}
    // The sequence of coordinates for the leader copter to get to at a certain time, [x,y,z, Time] all in the world frame!
    //NOTE: the initial coordinate of the leader is 0 0 0.
//
    /* Determine this */
//    std::vector <std::vector <double>> setPointCoordinates {{0.0,0.0,0.0,0.0} ,{0.0,0.0,1.5,5.0}, {-1.5,-1.5,1.5,4.0}, {0.0,0.0,1.5,4.0}, {-1.5,-1.5,1.5,4.0}, {0.0,0.0,1.5,4.0}, {0.0,0.0,0.0,5.0}};

    std::vector <std::vector <double>> setPointCoordinates {{0.0,0.0,0.0,0.0} ,{0.0,0.0,1.5,5.0}, {0.0,0.0,1.5,35.0}, {0.0,0.0,0.0,5.0}};

    /*{-1.5,-1.5,1.5,4.0}, {0.0,0.0,1.5,4.0}, {-1.5,-1.5,1.5,4.0}, {0.0,0.0,1.5,4.0},*/
//    setPointCoordinates.push_back(std::vector<double>{-1.5,-1.5,1.5,4.0});
//    setPointCoordinates.push_back({0.0,0.0,1.5,4.0});
//    setPointCoordinates.push_back({-1.5,-1.5,1.5,4.0});
//    setPointCoordinates.push_back({0.0,0.0,1.5,4.0});
//    setPointCoordinates.push_back({0.0,0.0,0.0,5.0});

    std::vector <double> xOffsets {0.0}; // X coordinates of all copters w.r.t the leader e.g. {0, 1.2, 0.5}
    std::vector <double> yOffsets {0.0}; // Y coordinates of all copters w.r.t the leader e.g. {0, 0, -0.8}
    /* End of Determine this */

    std::vector <Eigen::Vector3d> setPointOnlyXYZ;

    std::vector <Eigen::VectorXd> coffsX;
    std::vector <Eigen::VectorXd> coffsY;
    std::vector <Eigen::VectorXd> coffsZ;

    std::vector <Eigen::VectorXd> errors; //This is a vector of eigen-vectors containing the position/velocity errors of each copter.

    /* Look at the constructor for initial values */
    Eigen::Vector3d desiredPose;  // Desired position of the leader copter
    Eigen::Vector3d desiredVel;   // [0, 0, 0]  Desired position of the leader copter
    Eigen::Vector3d desiredAccel; // [0, 0, 0]This list contains the desired acceleration of the copters. Since copter trajectories are the same except for just an offset, this is one list but not a list of lists.
    /* Look at the constructor for initial values */
    bool ARM_FLAG = true;
    bool FAILSAFE_FLAG = false;
    double maxPositionErrorTolerance = 0.5*3; //in meter

    int index = 1; //Indicates the current number of piece-wise trajectories past sofar
    double expTimeElapsed = deadTime + rampUpDuration + trajStartDelay;  //Experiment time elapsed at the begining of trajectory generation




    Trajectory_Planner(){
        desiredPose = Eigen::Vector3d::Zero();
        desiredVel = Eigen::Vector3d::Zero();
        desiredAccel = Eigen::Vector3d::Zero();
        double timeFinal = deadTime + rampUpDuration + trajStartDelay;  // trajStartDelay is becuase of having the controller in the loop before the traj starts
        for (int i = 0; i <(setPointCoordinates.size()-1); i++) {
            double timeInitial = timeFinal;
            timeFinal +=  setPointCoordinates.at(i+1).at(3);
            coffsX.push_back(traj_coeffs(timeInitial, timeFinal, setPointCoordinates.at(i).at(0), setPointCoordinates.at(i+1).at(0)));
            coffsY.push_back(traj_coeffs(timeInitial, timeFinal, setPointCoordinates.at(i).at(1), setPointCoordinates.at(i+1).at(1)));
            coffsZ.push_back(traj_coeffs(timeInitial, timeFinal, setPointCoordinates.at(i).at(2), setPointCoordinates.at(i+1).at(2)));
        }
        for (int i = 0; i <(setPointCoordinates.size()); i++) {
            Eigen::Vector3d temp_vec;
            temp_vec << setPointCoordinates.at(i).at(0),setPointCoordinates.at(i).at(1),setPointCoordinates.at(i).at(2);
            setPointOnlyXYZ.push_back(temp_vec);
        }
        expTimeTotal = timeFinal; // The totall time from expTime = 0 (absolute zero) to the end of trajectories (landing).
        std::cout << "Total experiment time is " << expTimeTotal << "seconds" << std::endl;
        for (int i = 0; i < xOffsets.size(); i++) {
            errors.push_back(Eigen::VectorXd::Zero(6));  //Position/velocity errors [e_x, e_y, e_z, e_xd, e_yd, e_zd]
        }
    }

    Eigen::VectorXd traj_coeffs(double Ti, double Tf ,double qi, double qf) {
        /* Derives the trajectory coefficients for a 3rd order polynomial */
        // Ti and Tf are absolute times w.r.t the begining of the test.

        Eigen::MatrixXd A_Matrix (4,4);
        A_Matrix << 1.0, Ti, pow(Ti,2), pow(Ti,3),
                    0.0, 1.0, 2.0*Ti, 3.0*pow(Ti,2),
                    1.0, Tf, pow(Tf,2), pow(Tf,3),
                    0.0, 1.0, 2.0*Tf, 3.0*pow(Tf,2);
        Eigen::VectorXd B_Vector (4);
        B_Vector << qi, 0.0, qf, 0.0;
        Eigen::VectorXd coeffsTemp = A_Matrix.colPivHouseholderQr().solve(B_Vector);
        return coeffsTemp;
    }

    Eigen::VectorXd T_vector3(double t) {
        Eigen::Vector4d temp;
        temp << 1, t, pow(t,2), pow(t,3);
        return temp;
    }
    Eigen::VectorXd T_vector2(double t) {
        Eigen::Vector3d temp;
        temp << 1, 2*t, 3*pow(t,2);
        return temp;
    }
    Eigen::VectorXd T_vector1(double t) {
        Eigen::Vector2d temp;
        temp << 2, 6*t;
        return temp;
    }

    void generate(double expTime, std::vector <Eigen::Vector3d> position, std::vector <Eigen::Vector3d> velocity) {
        /* Generates desired trajectory, sets the phase, and returns desired positions, velocities and acceleration */
        // Determining the operation phase
        if (expTime <= deadTime) {
        }
        else if (expTime <= deadTime + rampUpDuration) {
            //ramp up
            phase = 1;
        }
        else if (expTime <=  expTimeTotal + trajStartDelay){ //
            // Feedback control is in the loop. # 2 seconds before/after the trajectories start/end.
            phase = 2;
        }
        else if (expTime <=  expTimeTotal + trajStartDelay + rampDownDuration){
            // Ramp down
            phase = 3;
        }
        else{
            phase = 0;
            ARM_FLAG = false;
        }
        /* Planning the trajectories */
        Eigen::Vector3d DX;
        Eigen::Vector3d DXd;
        Eigen::Vector3d DXdd;
        if (trajType==1) {
            if (expTime < deadTime + rampUpDuration + trajStartDelay) {
                // Stay at the very first commanded coordinates
                // DX = self.setPointCoordinates[0][0] , DY = self.setPointCoordinates[0][1], DZ = self.setPointCoordinates[0][2]
                DX = setPointOnlyXYZ.at(0);
                DXd = Eigen::Vector3d::Zero();
                DXdd = Eigen::Vector3d::Zero();
                // DXd = 0; DYd = 0; DZd = 0; DXdd = 0; DYdd = 0; DZdd = 0;
            }
            else if (expTime < expTimeTotal){
                if (setPointCoordinates[index][3] > expTime - expTimeElapsed) {
                    DX = setPointOnlyXYZ.at(index);
                    DXd = Eigen::Vector3d::Zero();
                    DXdd = Eigen::Vector3d::Zero();
                }
                else {
                    //Have to define desired trajectories in the else case
                    DX = setPointOnlyXYZ.at(index);
                    DXd = Eigen::Vector3d::Zero();
                    DXdd = Eigen::Vector3d::Zero();
                    expTimeElapsed += setPointCoordinates[index][3];
                    index += 1;
                    expTimeElapsed += setPointCoordinates[index][3];
                    index += 1; //Note that this equates len(setPointCoordinates) at the end.
                }
            }
            else {
                //Stay at the very last commanded coordinates
                DX = setPointOnlyXYZ.back();
                DXd = Eigen::Vector3d::Zero();
                DXdd = Eigen::Vector3d::Zero();
            }
        }
        else {
            /* Polynomials */
            if (expTime < deadTime + rampUpDuration + trajStartDelay) {
                DX = setPointOnlyXYZ.at(0);
                DXd = Eigen::Vector3d::Zero();
                DXdd = Eigen::Vector3d::Zero();
            }
            else if (expTime < expTimeTotal) {
                if (setPointCoordinates[index][3] > expTime - expTimeElapsed) {
                    int derivativeEnable = 1;
                    DX(0) = coffsX.at(index-1).transpose() * T_vector3(expTime);
                    DX(1) = coffsY.at(index-1).transpose() * T_vector3(expTime);
                    DX(2) = coffsZ.at(index-1).transpose() * T_vector3(expTime);
                    DXd(0) = derivativeEnable*coffsX.at(index-1).segment(1,3).transpose() * T_vector2(expTime);
                    DXd(1) = derivativeEnable*coffsY.at(index-1).segment(1,3).transpose() * T_vector2(expTime);
                    DXd(2) = derivativeEnable*coffsZ.at(index-1).segment(1,3).transpose() * T_vector2(expTime);
                    DXdd(0) = derivativeEnable*coffsX.at(index-1).segment(2,2).transpose() * T_vector1(expTime);
                    DXdd(1) = derivativeEnable*coffsY.at(index-1).segment(2,2).transpose() * T_vector1(expTime);
                    DXdd(2) = derivativeEnable*coffsZ.at(index-1).segment(2,2).transpose() * T_vector1(expTime);
                }
                else {
                    // Have to define desired trajectories in the else case
                    DX = setPointOnlyXYZ.at(index);
                    DXd = Eigen::Vector3d::Zero();
                    DXdd = Eigen::Vector3d::Zero();
                    expTimeElapsed += setPointCoordinates.at(index).at(3);
                    index += 1;
                }
            }
            else {
                //Stay at the very last commanded coordinates
                DX = setPointOnlyXYZ.back();
                DXd = Eigen::Vector3d::Zero();
                DXdd = Eigen::Vector3d::Zero();
            }
        }
        // Position/velocity errors [e_x, e_y, e_z, e_xd, e_yd, e_zd]
        for (int i = 0; i < xOffsets.size(); i++) {
            errors.at(i) <<  DX(0) + xOffsets[i] - position.at(i)(0), DX(1) + yOffsets[i] - position.at(i)(1), DX(2) - position.at(i)(2), DXd - velocity.at(i); //Leader first

            // Failsafe Check
            for (int j = 0; j < 3; j++) {
                if (fabs(errors.at(i)(j)) > maxPositionErrorTolerance){
                    FAILSAFE_FLAG = true;
                }
            }
        }
        // Desired position, velocity, and acceleration of the head copter
        desiredPose = DX;
        desiredVel = DXd;
        desiredAccel = DXdd;
    }



};





#endif //CRAZYFLIE_NATNET_TRAJECTORY_PLANNER_H

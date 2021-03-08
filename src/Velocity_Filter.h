//
// Created by AliGriv on 2021-03-04.
//

#ifndef CRAZYFLIE_NATNET_VELOCITY_FILTER_H
#define CRAZYFLIE_NATNET_VELOCITY_FILTER_H

#include "fir_filter.h"
#include <vector>
#include <Eigen/Dense>
class Velocity_Filter {
public:
    Velocity_Filter(int order = 3, float cutoff = 0.1, int channelNum = 3) {
        velDataRaw_m = Eigen::MatrixXd::Zero(channelNum, order); //Holds raw data (has a memory equal to order of filter)
        fir_filter = FIR_filter(order, cutoff, 0.0, fir_type, fir_window);
        FIR_coeffs = fir_filter.getCoefficients();
        FIR_coeffs_vec = Eigen::VectorXd::Zero(FIR_coeffs.size()); //Get coefficients for FIR lowpass and put in vector form
        for (int i = 0; i < FIR_coeffs.size(); i++) {
            FIR_coeffs_vec(i) = FIR_coeffs.at(i);
        }
        posOld_m = Eigen::MatrixXd::Zero(channelNum,1);
        velCur_m = Eigen::MatrixXd::Zero(channelNum,1);
    }
    Eigen::VectorXd numDiff_FIR(Eigen::VectorXd curChannelData) {
        for (int i = 0; i < curChannelData.size(); i++) {
            velCur_m(i,0) = (curChannelData(i) - posOld_m(i,0))/0.008; //Numerical differntiation to get current velocity
            posOld_m(i,0) = curChannelData(i); //update old position
        }
        velDataRaw_m.block(0,1,velDataRaw_m.rows(),velDataRaw_m.cols()-1) = velDataRaw_m.block(0,0,velDataRaw_m.rows(),velDataRaw_m.cols()-1); //Shift all elements to the right by 1
        velDataRaw_m.block(0,0,velDataRaw_m.rows(),1) = velCur_m;
        return velDataRaw_m * FIR_coeffs_vec.transpose();
    }

private:
    int channelSize;
    std::vector <double> FIR_coeffs;
    Eigen::VectorXd FIR_coeffs_vec;
    Eigen::MatrixXd velDataRaw_m;
    FIR_filter fir_filter;
    const char* fir_type = "lp";
    const char* fir_window = "hamming";

    Eigen::MatrixXd posOld_m;
    Eigen::MatrixXd velCur_m;

};










#endif //CRAZYFLIE_NATNET_VELOCITY_FILTER_H

//
// Created by AliGriv on 2021-03-18.
//
#include <iostream>
#include "gnuplot_i.hpp"
#include "fir_filter.h"
#include "Velocity_Filter.h"
#include <vector>
#include <Eigen/Dense>
#include "cmath"

void wait_for_key ()
{
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__TOS_WIN__)  // every keypress registered, also arrow keys
    cout << endl << "Press any key to continue..." << endl;

    FlushConsoleInputBuffer(GetStdHandle(STD_INPUT_HANDLE));
    _getch();
#elif defined(unix) || defined(__unix) || defined(__unix__) || defined(__APPLE__)
    std::cout << std::endl << "Press ENTER to continue..." << std::endl;

    std::cin.clear();
    std::cin.ignore(std::cin.rdbuf()->in_avail());
    std::cin.get();
#endif
    return;
}


int main () {
    std::vector <double> x_vec (200);
    for (int i = 1; i < x_vec.size(); i++) {
        x_vec.at(i) = x_vec.at(i-1) + 0.01;
    }
    std::cout << "x_vec is ";
    for (auto &x: x_vec) {
        std::cout << x << " ";
    }
    std::cout << std::endl;

    std::vector <double> y_vec (200);
    for (int i=0; i < y_vec.size(); i++) {
        y_vec.at(i) = 10.0*sin(M_PI*x_vec.at(i));
    }

    std::vector <double> y_noisy_vec (y_vec.size());
    for (int i=0; i < y_noisy_vec.size(); i++) {
        double r = ((double) rand() / (RAND_MAX) - 0.5)/10.0;
        y_noisy_vec.at(i) = y_vec.at(i) + r;
    }


    Gnuplot Position("points");
    Position.reset_plot();
    std::string title = "Position Signal";
//    Position.savetops(title);
    Position.set_smooth().set_style("points").plot_xy(x_vec,y_vec,"Ideal Position Signal").plot_xy(x_vec,y_noisy_vec, "Noisy Position Signal");
    Position.set_title(title);
    Position.set_legend("outside right top");
    Position.set_grid();


    Velocity_Filter vel_fil(3,0.1,1);
    std::vector <double> yd_filtered (y_noisy_vec.size());
    std::vector <double> yd_ideal (y_noisy_vec.size());
    Eigen::VectorXd y_temp (1);
    Eigen::VectorXd yd_temp;
    for (int i=0; i < y_noisy_vec.size(); i++){
        y_temp(0) = y_noisy_vec.at(i);
        yd_temp = vel_fil.numDiff_FIR(y_temp,0.01);
        yd_filtered.at(i) = yd_temp(0);
        yd_ideal.at(i) = 10.0*M_PI*cos(M_PI*x_vec.at(i));
    }

    Gnuplot Velocity("points");
    Velocity.reset_plot();
    title = "Velocity Signal";
    Velocity.set_smooth().set_style("points").plot_xy(x_vec,yd_ideal,"Ideal Velocity Signal").plot_xy(x_vec,yd_filtered,"Filtered Velocity Signal");
    Velocity.set_title(title);
    Velocity.set_legend("outside right top");
    Velocity.set_grid();

    wait_for_key();

    return 0;
}
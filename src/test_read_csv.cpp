//
// Created by AliGriv on 2021-05-02.
//

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include "Trajectory_Planner.h"
#include <Eigen/Dense>
using namespace std;

int main()
{
    ifstream in("trajectories.csv");
    vector<vector<double>> fields;

    if (in) {
        string line;

        while (getline(in, line)) {
            stringstream sep(line);
            string field;

            fields.push_back(vector<double>());

            while (getline(sep, field, ',')) {
                fields.back().push_back(stod(field));
            }
        }
    }

//    for (auto row : fields) {
//        for (auto field : row) {
//            cout << field << ' ';
//        }
//
//        cout << '\n';
//    }

    std::cout << "Number of rows is " << fields.size() << std::endl;
    std::cout << "Number of columns is " << fields.at(0).size() << std::endl;


    Trajectory_Planner_CSV traj(3, "trajectories.csv");
    int i = 0;
    while (i < traj.desiredPose_vec.at(1).size()) {
        std::cout << "desired pos is " << traj.desiredPose_vec.at(1).at(i).transpose() << std::endl;
        i += 100;
    }
}
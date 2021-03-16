//
// Created by AliGriv on 2021-03-11.
//

#ifndef CRAZYFLIE_NATNET_RECORDER_H
#define CRAZYFLIE_NATNET_RECORDER_H


#define GET_VARIABLE_NAME(Variable) (#Variable)

#include <iostream>
#include <vector>
#include <string>
#include <stdexcept>
#include <Eigen/Dense>
#include "CSVWriter.h"
#include "PB_Control.h"
//gnuplot related
#include "gnuplot_i.hpp" //Gnuplot class handles POSIX-Pipe-communikation with Gnuplot

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__TOS_WIN__)
#include <conio.h>   //for getch(), needed in wait_for_key()
 #include <windows.h> //for Sleep()
 void sleep(int i) { Sleep(i*1000); }
#endif

void wait_for_key ();


class Recorder {
public:
    Recorder(int num_copters, const char* logFileName = "data.csv"){
        fileName = logFileName;
        numCopters = num_copters;
        for (int i=0; i < num_copters; i++) {
            // Any new variables must be initialized here
            DesiredPositions.push_back({});
            Positions.push_back({});
            PositionErrors.push_back({});
            Velocities.push_back({});
            Orientations.push_back({});
            Commands.push_back({});
            HighLevelCommands.push_back({});
            TrackingFlags.push_back({});
            TimeCounter.push_back({});
        }
        // Any new variable must be added to function createVariableNames (and also printVariableNames)
        createVariableNames(num_copters);
        printVariableNames();
    }
    void appendDesiredPosition(const Eigen::Vector3d &p, const int &copter_index);
    void appendPosition(const Eigen::Vector3d &p, const int &copter_index);
    void appendPositionError(const Eigen::Vector3d &p, const int &copter_index);
    void appendVelocity(const Eigen::Vector3d &v, const int &copter_index);
    void appendOrientation(const Eigen::Vector4d &o, const int &copter_index);
    void appendCommand(const Command &c, const int &copter_index);
    void appendHighLevelCommand(const Eigen::Vector3d &c, const int &copter_index);
    void appendTrackingFlag(const bool t, const int &copter_index);
    void appendTime(const double t, const int &copter_index);
    void saveDataToFile();
    void createVariableNames(const int num_copters);
    void addVariablesToCSV(CSVWriter &csv);
    void printVariableNames();
    void addVectorToCSV(CSVWriter &csv, const Eigen::Vector3d &v);
    void addVectorToCSV(CSVWriter &csv, const Eigen::Vector4d &v);
    void generatePlots();
private:
    const char* fileName;
    int numCopters;
    std::vector <std::string> VariableNames;
    std::vector <std::vector <Eigen::Vector3d>> DesiredPositions;
    std::vector <std::vector <Eigen::Vector3d>> Positions;
    std::vector <std::vector <Eigen::Vector3d>> PositionErrors;
    std::vector <std::vector <Eigen::Vector3d>> Velocities;
    std::vector <std::vector <Eigen::Vector4d>> Orientations;
    std::vector <std::vector <Command>> Commands;
    std::vector <std::vector <Eigen::Vector3d>> HighLevelCommands;
    std::vector <std::vector <bool>> TrackingFlags;
    std::vector <std::vector<double>> TimeCounter;

};



void Recorder::appendDesiredPosition(const Eigen::Vector3d &p, const int &copter_index) {
    if (copter_index >= 0 && copter_index < numCopters) {
        DesiredPositions.at(copter_index).push_back(p);
    }
    else {
        std::cerr << "copter_index in appendDesiredPosition is out of range" <<std::endl;
    }
}

void Recorder::appendPosition(const Eigen::Vector3d &p, const int &copter_index) {
    if (copter_index >= 0 && copter_index < numCopters) {
        Positions.at(copter_index).push_back(p);
    }
    else {
        std::cerr << "copter_index in appendPosition is out of range" <<std::endl;
    }
}

void Recorder::appendPositionError(const Eigen::Vector3d &p, const int &copter_index) {
    if (copter_index >= 0 && copter_index < numCopters) {
        PositionErrors.at(copter_index).push_back(p);
    }
    else {
        std::cerr << "copter_index in appendPositionError is out of range" <<std::endl;
    }
}

void Recorder::appendVelocity(const Eigen::Vector3d &v, const int &copter_index) {
    if (copter_index >= 0 && copter_index < numCopters) {
        Velocities.at(copter_index).push_back(v);
    }
    else {
        std::cerr << "copter_index in appendVelocity is out of range" <<std::endl;
    }
}


void Recorder::appendOrientation(const Eigen::Vector4d &o, const int &copter_index){
    if (copter_index >= 0 && copter_index < numCopters) {
        Orientations.at(copter_index).push_back(o);
    }
    else {
        std::cerr << "copter_index in appendOrientation is out of range" <<std::endl;
    }
}
void Recorder::appendCommand(const Command &c, const int &copter_index){
    if (copter_index >= 0 && copter_index < numCopters) {
        Commands.at(copter_index).push_back(c);
    }
    else {
        std::cerr << "copter_index in appendCommand is out of range" <<std::endl;
    }
}
void Recorder::appendHighLevelCommand(const Eigen::Vector3d &c, const int &copter_index){
    if (copter_index >= 0 && copter_index < numCopters) {
        HighLevelCommands.at(copter_index).push_back(c);
    }
    else {
        std::cerr << "copter_index in appendHighLevelCommand is out of range" <<std::endl;
    }
}
void Recorder::appendTrackingFlag(const bool t, const int &copter_index) {
    if (copter_index >= 0 && copter_index < numCopters) {
        TrackingFlags.at(copter_index).push_back(t);
    }
    else {
        std::cerr << "copter_index in appendTrackingFlag is out of range" <<std::endl;
    }
}
void Recorder::appendTime(const double t, const int &copter_index) {
    if (copter_index >= 0 && copter_index < numCopters) {
        TimeCounter.at(copter_index).push_back(t);
    }
    else {
        std::cerr << "copter_index in appendTime is out of range" <<std::endl;
    }
}



void Recorder::createVariableNames(const int num_copters) {
    for (int i=0; i < num_copters; i++) {
        std::string name = "DesredPosition";
        std::string id = std::to_string(i);
        name += '[' + id + ']';
        for (auto c: {'x','y','z'}) {
            VariableNames.push_back(name + c);
        }
    }
    for (int i=0; i < num_copters; i++) {
        std::string name = "Position";
        std::string id = std::to_string(i);
        name += '[' + id + ']';
        for (auto c: {'x','y','z'}) {
            VariableNames.push_back(name + c);
        }
    }
    for (int i=0; i < num_copters; i++) {
        std::string name = "PositionError";
        std::string id = std::to_string(i);
        name += '[' + id + ']';
        for (auto c: {'x','y','z'}) {
            VariableNames.push_back(name + c);
        }
    }
    for (int i=0; i < num_copters; i++) {
        std::string name = "Velocity";
        std::string id = std::to_string(i);
        name += '[' + id + ']';
        for (auto c: {'x','y','z'}) {
            VariableNames.push_back(name + c);
        }
    }
    for (int i=0; i < num_copters; i++) {
        std::string name = "Orientation";
        std::string id = std::to_string(i);
        name += '[' + id + ']';
        for (auto c: {'x','y','z','w'}) {
            VariableNames.push_back(name + c);
        }
    }
    for (int i=0; i < num_copters; i++) {
        std::string name = "Command";
        std::string id = std::to_string(i);
        name += '[' + id + ']';
        for (auto c: {"roll","pitch","throttle","yawrate"}) {
            VariableNames.push_back(name + c);
        }
    }
    for (int i=0; i < num_copters; i++) {
        std::string name = "HighLevelCommand";
        std::string id = std::to_string(i);
        name += '[' + id + ']';
        for (auto c: {'x','y','z'}) {
            VariableNames.push_back(name + c);
        }
    }
    for (int i=0; i< num_copters; i++) {
        std::string name = "TrackingFlag";
        std::string id = std::to_string(i);
        name += '[' + id + ']';
        VariableNames.push_back(name);
    }
    for (int i=0; i< num_copters; i++) {
        std::string name = "TimeCounter";
        std::string id = std::to_string(i);
        name += '[' + id + ']';
        VariableNames.push_back(name);
    }
}

void Recorder::printVariableNames() {
    std::cout << "Here are the variable names" << std::endl;
    for (auto v: VariableNames) {
        std::cout << v << std::endl;
    }
}

void Recorder::saveDataToFile() {
    CSVWriter csv;
    csv.enableAutoNewRow(VariableNames.size());
    //first row
    for (auto name: VariableNames) {
        csv << name;
    }
    addVariablesToCSV(csv);
    csv.writeToFile(fileName);
    std::cout << "CSV file created" << std::endl;
}

void Recorder::addVariablesToCSV(CSVWriter &csv) {
    try {
        for (int i = 0; i < DesiredPositions.at(0).size(); i++) {
            for (int n = 0; n < numCopters; n++) {
                addVectorToCSV(csv, DesiredPositions.at(n).at(i));
            }
            for (int n = 0; n < numCopters; n++) {
                addVectorToCSV(csv, Positions.at(n).at(i));
            }
            for (int n = 0; n < numCopters; n++) {
                addVectorToCSV(csv, PositionErrors.at(n).at(i));
            }
            for (int n = 0; n < numCopters; n++) {
                addVectorToCSV(csv, Velocities.at(n).at(i));
            }
            for (int n = 0; n < numCopters; n++) {
                addVectorToCSV(csv, Orientations.at(n).at(i));
            }
            for (int n = 0; n < numCopters; n++) {
                csv << Commands.at(n).at(i).roll << Commands.at(n).at(i).pitch << Commands.at(n).at(i).throttle << Commands.at(n).at(i).yawRate;
            }
            for (int n = 0; n < numCopters; n++) {
                addVectorToCSV(csv, HighLevelCommands.at(n).at(i));
            }
            for (int n = 0; n < numCopters; n++) {
                csv << TrackingFlags.at(n).at(i);
            }
            for (int n = 0; n < numCopters; n++) {
                csv << TimeCounter.at(n).at(i);
            }
        }
    }
    catch (const std::out_of_range& oor) {
        std::cerr << "Out of Range error: " << oor.what() << '\n';
    }
}

void Recorder::addVectorToCSV(CSVWriter &csv, const Eigen::Vector3d &v) {
    for (int i = 0; i < v.size(); i++) {
        csv << v(i);
    }
}
void Recorder::addVectorToCSV(CSVWriter &csv, const Eigen::Vector4d &v) {
    for (int i = 0; i < v.size(); i++) {
        csv << v(i);
    }
}


void Recorder::generatePlots() {
    try {
        Gnuplot DesiredPositionPlots("points");
        for (int n = 0; n < numCopters; n++) {

            std::string title = "DesiredPosition";
            title += '[' + std::to_string(n) + ']';
            DesiredPositionPlots.savetops(title);
            std::vector <double> x,y,z;
//            std::cout << DesiredPositions.at(n).size() << std::endl;
            for (int i = 0; i < DesiredPositions.at(n).size(); i++) {
                x.push_back(DesiredPositions.at(n).at(i)(0));
                y.push_back(DesiredPositions.at(n).at(i)(1));
                z.push_back(DesiredPositions.at(n).at(i)(2));
            }

            DesiredPositionPlots.set_smooth().set_style("points").plot_xy(TimeCounter.at(n),x, "x").plot_xy(TimeCounter.at(n),y, "y").plot_xy(TimeCounter.at(n),z, "z");


            DesiredPositionPlots.set_title(title);
            DesiredPositionPlots.set_legend("outside right top");
            DesiredPositionPlots.showonscreen();
            DesiredPositionPlots.reset_plot();
        }


        Gnuplot PositionPlots("points");
        for (int n = 0; n < numCopters; n++) {

            std::string title = "Position";
            title += '[' + std::to_string(n) + ']';
            PositionPlots.savetops(title);
            std::vector <double> x,y,z;
//            std::cout << Positions.at(n).size() << std::endl;
            for (int i = 0; i < Positions.at(n).size(); i++) {
                x.push_back(Positions.at(n).at(i)(0));
                y.push_back(Positions.at(n).at(i)(1));
                z.push_back(Positions.at(n).at(i)(2));
            }

            PositionPlots.set_smooth().set_style("points").plot_xy(TimeCounter.at(n),x, "x").plot_xy(TimeCounter.at(n),y, "y").plot_xy(TimeCounter.at(n),z, "z");


            PositionPlots.set_title(title);
            PositionPlots.set_legend("outside right top");
            PositionPlots.showonscreen();
            PositionPlots.reset_plot();
        }

        Gnuplot PositionErrorPlots("points");
        for (int n = 0; n < numCopters; n++) {

            std::string title = "PositionError";
            title += '[' + std::to_string(n) + ']';
            PositionErrorPlots.savetops(title);
            std::vector <double> x,y,z;
//            std::cout << PositionErrors.at(n).size() << std::endl;
            for (int i = 0; i < PositionErrors.at(n).size(); i++) {
                x.push_back(PositionErrors.at(n).at(i)(0));
                y.push_back(PositionErrors.at(n).at(i)(1));
                z.push_back(PositionErrors.at(n).at(i)(2));
            }

            PositionErrorPlots.set_smooth().set_style("points").plot_xy(TimeCounter.at(n),x, "x").plot_xy(TimeCounter.at(n),y, "y").plot_xy(TimeCounter.at(n),z, "z");


            PositionErrorPlots.set_title(title);
            PositionErrorPlots.set_legend("outside right top");
            PositionErrorPlots.showonscreen();
            PositionErrorPlots.reset_plot();
        }

        Gnuplot VelocityPlots("points");
        for (int n = 0; n < numCopters; n++) {

            std::string title = "Velocity";
            title += '[' + std::to_string(n) + ']';
            VelocityPlots.savetops(title);
            std::vector <double> x,y,z;
//            std::cout << Velocities.at(n).size() << std::endl;
            for (int i = 0; i < Velocities.at(n).size(); i++) {
                x.push_back(Velocities.at(n).at(i)(0));
                y.push_back(Velocities.at(n).at(i)(1));
                z.push_back(Velocities.at(n).at(i)(2));
            }

            VelocityPlots.set_smooth().set_style("points").plot_xy(TimeCounter.at(n),x, "x").plot_xy(TimeCounter.at(n),y, "y").plot_xy(TimeCounter.at(n),z, "z");


            VelocityPlots.set_title(title);
            VelocityPlots.set_legend("outside right top");
            VelocityPlots.showonscreen();
            VelocityPlots.reset_plot();
        }

        Gnuplot CommandPlots("points");
        for (int n = 0; n < numCopters; n++) {

            std::string title = "Commands";
            title += '[' + std::to_string(n) + ']';
            CommandPlots.savetops(title);
            std::vector <double> roll,pitch,throttle,yawRate;
//            std::cout << Velocities.at(n).size() << std::endl;
            for (int i = 0; i < Commands.at(n).size(); i++) {
                roll.push_back(Commands.at(n).at(i).roll);
                pitch.push_back(Commands.at(n).at(i).pitch);
                throttle.push_back(Commands.at(n).at(i).throttle);
                yawRate.push_back(Commands.at(n).at(i).yawRate);
            }

            CommandPlots.set_smooth().set_style("points").plot_xy(TimeCounter.at(n),roll, "roll").plot_xy(TimeCounter.at(n),pitch, "pitch").plot_xy(TimeCounter.at(n),yawRate, "yawRate").plot_xy(TimeCounter.at(n),throttle, "throttle");


            CommandPlots.set_title(title);
            CommandPlots.set_legend("outside right top");
            CommandPlots.showonscreen();
            CommandPlots.reset_plot();
        }

        Gnuplot HighLevelCommandPlots("points");
        for (int n = 0; n < numCopters; n++) {

            std::string title = "Velocity";
            title += '[' + std::to_string(n) + ']';
            HighLevelCommandPlots.savetops(title);
            std::vector <double> x,y,z;
//            std::cout << HighLevelCommands.at(n).size() << std::endl;
            for (int i = 0; i < HighLevelCommands.at(n).size(); i++) {
                x.push_back(HighLevelCommands.at(n).at(i)(0));
                y.push_back(HighLevelCommands.at(n).at(i)(1));
                z.push_back(HighLevelCommands.at(n).at(i)(2));
            }

            HighLevelCommandPlots.set_smooth().set_style("points").plot_xy(TimeCounter.at(n),x, "x").plot_xy(TimeCounter.at(n),y, "y").plot_xy(TimeCounter.at(n),z, "z");


            HighLevelCommandPlots.set_title(title);
            HighLevelCommandPlots.set_legend("outside right top");
            HighLevelCommandPlots.showonscreen();
            HighLevelCommandPlots.reset_plot();
        }
    }
    catch (GnuplotException ge)
    {
        std::cout << ge.what() << std::endl;
    }
    std::cout << "Plots were generated" << std::endl;
}

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

#endif //CRAZYFLIE_NATNET_RECORDER_H

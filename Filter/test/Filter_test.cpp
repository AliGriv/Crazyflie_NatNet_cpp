//
// Created by AliGriv on 2021-02-24.
//


#include <iostream>
#include "fir_filter.h"
#include <vector>
#include <string>
#include <fstream>
#include <sstream>

void write_csv(std::string filename, std::vector<std::pair<std::string, std::vector<double>>> dataset){
    // Make a CSV file with one or more columns of integer values
    // Each column of data is represented by the pair <column name, column data>
    //   as std::pair<std::string, std::vector<int>>
    // The dataset is represented as a vector of these columns
    // Note that all columns should be the same size

    // Create an output filestream object
    std::ofstream myFile(filename);

    // Send column names to the stream
    for(int j = 0; j < dataset.size(); ++j)
    {
        myFile << dataset.at(j).first;
        if(j != dataset.size() - 1) myFile << ","; // No comma at end of line
    }
    myFile << "\n";

    // Send data to the stream
    for(int i = 0; i < dataset.at(0).second.size(); ++i)
    {
        for(int j = 0; j < dataset.size(); ++j)
        {
            myFile << dataset.at(j).second.at(i);
            if(j != dataset.size() - 1) myFile << ","; // No comma at end of line
        }
        myFile << "\n";
    }

    // Close the file
    myFile.close();
}
void write_csv(std::string filename, std::vector<double> dataset){
    // Make a CSV file with one or more columns of integer values
    // Each column of data is represented by the pair <column name, column data>
    //   as std::pair<std::string, std::vector<int>>
    // The dataset is represented as a vector of these columns
    // Note that all columns should be the same size

    // Create an output filestream object
    std::ofstream myFile(filename);

    // Send data to the stream
    for(int i = 0; i < dataset.size(); ++i)
    {
        myFile << dataset.at(i);
        myFile << "\n";
    }

    // Close the file
    myFile.close();
}

int main() {
    double fs = 1000.0;
    double f1 = 50.0/fs;
    int m = 3;
    const char* type = "lp";
    const char* window = "hamming";
    std::vector<double> coeffs;
    FIR_filter filter(m,f1,0.0,type,window);
    coeffs = filter.getCoefficients();
    std::cout << "The coefficients are:" << std::endl;
    for (auto c: coeffs) {
        std::cout << c << " ";
    }
    std::cout << std::endl;

    std::string filename = "Client-output.pts";
    std::string line;
    std::vector <float> x,y,z;
    float val;
    std::ifstream myFile(filename);
    if(!myFile.is_open()) throw std::runtime_error("Could not open file");
    if (myFile.good()) {
        while (std::getline(myFile,line)) {
            std::stringstream ss(line);
            int col_idx = 0;
            while (ss >> val) {

                if(ss.peek() == ' ') {
                    ss.ignore();
                }
                col_idx++;

                if (col_idx == 2) {
                    x.push_back(val);
                }
                if (col_idx == 3) {
                    y.push_back(val);
                }
                if (col_idx == 4) {
                    z.push_back(val);
                }
            }
        }
        myFile.close();
    }

    std::cout << "let's print 10 first values of z vector" << std::endl;
    for (int i {0}; i < 10; i++) {
        std::cout << z.at(i) << std::endl;
    }


    // Differentiate z vector
    std::vector<double> z_d;
    for (int i {1}; i < z.size(); i++) {
        z_d.push_back((z.at(i)-z.at(i-1))/0.008);
    }

    std::vector<double> z_d_filtered;
    for (int i {0}; i < z_d.size(); i++) {
        z_d_filtered.push_back(filter.filter(z_d.at(i)));
    }

    write_csv(std::string("z_d_unfiltered.txt"), z_d);
    write_csv(std::string("z_d_filtered.txt"), z_d_filtered);
    return 0;
}

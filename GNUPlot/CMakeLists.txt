cmake_minimum_required(VERSION 3.17)
project(CSVWriter)
set(CMAKE_CXX_STANDARD 11)

add_library(GNU_Plot SHARED gnuplot_i.hpp)
set_target_properties(GNU_Plot PROPERTIES LINKER_LANGUAGE CXX)

add_executable(gnuplot_example example.cpp)
target_link_libraries(gnuplot_example GNU_Plot)
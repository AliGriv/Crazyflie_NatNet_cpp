cmake_minimum_required(VERSION 3.17)
project(Crazyflie_NatNet)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
add_subdirectory(Crazyflie)
add_subdirectory(NatNet)
add_subdirectory(Filter)
add_subdirectory(GNUPlot)
add_subdirectory(CSVWriter)
set(CMAKE_CXX_STANDARD 11)

add_library(Velocity_Filter SHARED src/Velocity_Filter.h)
set_target_properties(Velocity_Filter PROPERTIES LINKER_LANGUAGE CXX)
target_include_directories(Velocity_Filter PUBLIC Filter/include)
target_link_libraries(Velocity_Filter PUBLIC FIR_Filter)
target_link_libraries(Velocity_Filter PUBLIC Eigen3::Eigen)


add_library(Sensor SHARED src/Sensor.h)
set_target_properties(Sensor PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(Sensor PUBLIC Velocity_Filter)
target_link_libraries(Sensor PUBLIC Eigen3::Eigen)


add_library(PB_Control SHARED src/PB_Control.h)
set_target_properties(PB_Control PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(PB_Control PUBLIC Eigen3::Eigen)

add_library(Trajectory_Planner SHARED src/Trajectory_Planner.h)
set_target_properties(Trajectory_Planner PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(Trajectory_Planner PUBLIC Eigen3::Eigen)

add_library(Recorder SHARED src/Recorder.h)
set_target_properties(Recorder PROPERTIES LINKER_LANGUAGE CXX)
target_include_directories(Recorder PUBLIC CSVWriter/include)
target_include_directories(Recorder PUBLIC GNUPlot)
target_link_libraries(Recorder PUBLIC Eigen3::Eigen CSVWriter GNU_Plot)



add_executable(scrap src/scrap.cpp)
target_link_libraries(scrap PRIVATE crazyflie_cpp)
target_include_directories(scrap PRIVATE Crazyflie/crazyflie_cpp/include)
target_link_libraries(scrap PUBLIC Eigen3::Eigen Velocity_Filter PB_Control CSVWriter Recorder)
target_include_directories(scrap PUBLIC CSVWriter/include)
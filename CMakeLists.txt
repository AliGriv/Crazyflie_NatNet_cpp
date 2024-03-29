cmake_minimum_required(VERSION 3.17)
project(Crazyflie_NatNet)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
add_subdirectory(Crazyflie)
add_subdirectory(NatNet)
add_subdirectory(Filter)
add_subdirectory(GNUPlot)
add_subdirectory(CSVWriter)
set(CMAKE_CXX_STANDARD 11)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")
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
target_link_libraries(Recorder PUBLIC Eigen3::Eigen CSVWriter GNU_Plot PB_Control)

#install(FILES src/generatingPlotsFromCSV_old.py DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
configure_file(src/generatingPlotsFromCSV.py ${CMAKE_CURRENT_BINARY_DIR}/generatingPlotsFromCSV.py COPYONLY)
configure_file(src/requirements.txt ${CMAKE_CURRENT_BINARY_DIR}/requirements.txt COPYONLY)
add_executable(scrap src/scrap.cpp)
target_link_libraries(scrap PRIVATE crazyflie_cpp)
target_include_directories(scrap PRIVATE Crazyflie/crazyflie_cpp/include)
target_link_libraries(scrap PUBLIC Eigen3::Eigen Velocity_Filter PB_Control CSVWriter Recorder)
target_include_directories(scrap PUBLIC CSVWriter/include)

add_executable(Main src/Main.cpp)
target_link_libraries(Main PUBLIC crazyflie_cpp Eigen3::Eigen Sensor Velocity_Filter PB_Control Recorder Trajectory_Planner)
target_include_directories(Main PUBLIC Crazyflie/crazyflie_cpp/include)
target_link_directories(Main PUBLIC NatNet)
target_link_libraries(Main PUBLIC NatNetUtility NatNetLibs -lNatNetLibShared)


add_executable(Main_delayed_uxy src/Main_delayed_uxy.cpp)
target_link_libraries(Main_delayed_uxy PUBLIC crazyflie_cpp Eigen3::Eigen Sensor Velocity_Filter PB_Control Recorder Trajectory_Planner)
target_include_directories(Main_delayed_uxy PUBLIC Crazyflie/crazyflie_cpp/include)
target_link_directories(Main_delayed_uxy PUBLIC NatNet)
target_link_libraries(Main_delayed_uxy PUBLIC NatNetUtility NatNetLibs -lNatNetLibShared)

add_executable(Main_multi src/Main_multi.cpp)
target_link_libraries(Main_multi PUBLIC crazyflie_cpp Eigen3::Eigen Sensor Velocity_Filter PB_Control Recorder Trajectory_Planner)
target_include_directories(Main_multi PUBLIC Crazyflie/crazyflie_cpp/include)
target_link_directories(Main_multi PUBLIC NatNet)
target_link_libraries(Main_multi PUBLIC NatNetUtility NatNetLibs -lNatNetLibShared)

add_executable(Main_multi_csv src/Main_multi_csv.cpp)
target_link_libraries(Main_multi_csv PUBLIC crazyflie_cpp Eigen3::Eigen Sensor Velocity_Filter PB_Control Recorder Trajectory_Planner)
target_include_directories(Main_multi_csv PUBLIC Crazyflie/crazyflie_cpp/include)
target_link_directories(Main_multi_csv PUBLIC NatNet)
target_link_libraries(Main_multi_csv PUBLIC NatNetUtility NatNetLibs -lNatNetLibShared)


add_executable(Velocity_filter_test src/velocity_filter_test.cpp)
target_link_libraries(Velocity_filter_test PUBLIC Eigen3::Eigen GNU_Plot Velocity_Filter Sensor)
target_include_directories(Velocity_filter_test PUBLIC GNUPlot )

add_executable(condition_var_test src/condition_var_test.cpp)
target_link_libraries(condition_var_test PUBLIC Eigen3::Eigen)

add_executable(test_csv_read src/test_read_csv.cpp)
configure_file(src/trajectories.csv ${CMAKE_CURRENT_BINARY_DIR}/trajectories.csv COPYONLY)
target_link_libraries(test_csv_read PUBLIC Trajectory_Planner Eigen3::Eigen)
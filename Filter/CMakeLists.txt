cmake_minimum_required(VERSION 3.17)
project(FIR_Filter)

set(CMAKE_CXX_STANDARD 11)

add_library(FIR_Filter SHARED include/fir_filter.h src/fir_filter.cpp)
target_include_directories(FIR_Filter PUBLIC include)

add_executable(Filter_test test/Filter_test.cpp)
target_include_directories(Filter_test PRIVATE include)

install(FILES test/Client-output.pts DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
message(${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(Filter_test FIR_Filter)


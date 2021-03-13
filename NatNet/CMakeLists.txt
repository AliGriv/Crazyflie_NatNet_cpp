cmake_minimum_required(VERSION 3.17)
project(NatNet_Project)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
set(CMAKE_CXX_STANDARD 11)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")


include_directories(include)
link_directories(lib)
#link_directories(/NatNet_SDK_3.1_ubuntu/lib)
set(HEADER_FILES include/NatNetCAPI.h include/NatNetClient.h include/NatNetRepeater.h include/NatNetRequests.h include/NatNetTypes.h)
add_library(NatNetLibs SHARED ${HEADER_FILES})
target_include_directories(NatNetLibs PUBLIC include)
target_link_directories(NatNetLibs PUBLIC lib)
set_target_properties(NatNetLibs PROPERTIES LINKER_LANGUAGE CXX)



#include_directories(${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(NatNetLibs PUBLIC -lNatNetLibShared)

add_executable(SampleClient src/SampleClient.cpp)
target_link_directories(SampleClient PUBLIC lib)
target_include_directories(SampleClient PUBLIC include)
target_link_libraries(SampleClient NatNetLibs -lNatNetLibShared)

add_library(NatNetUtility SHARED include/NatNetUtility.h src/NatNetUtility.cpp)
target_link_directories(NatNetUtility PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(NatNetUtility PRIVATE NatNetLibs -lNatNetLibShared)
set_target_properties(NatNetUtility PROPERTIES LINKER_LANGUAGE CXX)
#
#
add_executable(util_example src/util_example.cpp)
target_link_directories(util_example PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(util_example PUBLIC NatNetUtility NatNetLibs -lNatNetLibShared)

add_executable(NatNet_test src/NatNet_test.cpp)
target_link_directories(NatNet_test PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(NatNet_test PUBLIC NatNetUtility NatNetLibs -lNatNetLibShared)
target_link_libraries(NatNet_test PUBLIC Eigen3::Eigen)


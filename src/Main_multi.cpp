//
// Created by AliGriv on 2021-03-15.
//
#include <iostream>
#include <crazyflie_cpp/Crazyflie.h>
#include "Velocity_Filter.h"
#include "Trajectory_Planner.h"
#include "PB_Control.h"
#include "Sensor.h"
#include "Recorder.h"
#include <cmath>
#include <mutex>
#include <Eigen/Dense>
#include <vector>
#include <chrono>
#include <thread>
#include "NatNetUtility.h"
#include "NatNetTypes.h"
#include "NatNetCAPI.h"
#include "NatNetClient.h"
#include <condition_variable>
#include "signal.h"

/* Global Variables */
std::vector <Eigen::Vector3d> positions;
std::vector <Eigen::Vector4d> orientations;
std::vector <bool> trackingFlags;
std::vector <Command> commandsToGo;
int numRigidBodies;
double expTime;
std::mutex m;
std::mutex m_in_main;
std::vector <Crazyflie> cfs;
bool EmergencyStopExperiment = false;
bool stopExperiment = false;
//NatNet Variables
static const ConnectionType kDefaultConnectionType = ConnectionType_Multicast;
NatNetClient* g_pClient = NULL;
sNatNetClientConnectParams g_connectParams;
sServerDescription g_serverDescription;
int g_analogSamplesPerMocapFrame = 0;

//The following variables are supposed to help with event handling between NatNet callback and mainThread
//bool IsNatNetReceiverInCall = false;
std::mutex m_event;
std::condition_variable cv_event;
bool ready_event = false;

/* End of Global Variables */

/* Functions' Prototypes */
void NATNET_CALLCONV receiveRigidBodyFrame(sFrameOfMocapData* data, void* pUserData);
int ConnectClient();

/* End of Functions' Prototypes */
void signal_callback_handler(int signum) {
    std::cout << "Caught signal " << signum << std::endl;
    std::cout << "This will stop the program" << std::endl;
    // Terminate program
//    exit(signum);
    EmergencyStopExperiment = true;
}
void mainThread_run(Sensor &sensor,
                    Trajectory_Planner &trajPlanner,
                    PB_Control &controller,
                    Recorder &rec){
    int loop_counter = 0;
    std::vector <Eigen::Vector3d> position_cache;
    std::vector <Eigen::Vector4d> orientation_cache;
    std::vector <bool> trackingFlags_cache;
    std::chrono::high_resolution_clock::time_point expInitTime;
    std::chrono::high_resolution_clock::time_point time_temp;

    while (true) {
        if (!EmergencyStopExperiment) { //proceed with natural main code
            if (trajPlanner.ARM_FLAG && !trajPlanner.FAILSAFE_FLAG && !sensor.FAILSAFE_FLAG) {
                std::unique_lock<std::mutex> lk(m_event);
                cv_event.wait(lk, []{return ready_event;});
                //I hope this helps, as a sort of event handling procedure
                position_cache = positions;
                orientation_cache = orientations;
                trackingFlags_cache = trackingFlags;
                if (!sensor.initFlag) {
                    sensor.process(position_cache, orientation_cache, trackingFlags_cache);
                    //precious_positions = position;
                    expInitTime = std::chrono::high_resolution_clock::now();
                }
                else {
                    time_temp = std::chrono::high_resolution_clock::now();
                    auto time_span = std::chrono::duration_cast<std::chrono::duration<double>>(time_temp - expInitTime);
                    expTime = time_span.count();
                    sensor.process(position_cache, orientation_cache, trackingFlags_cache);
                    //positions_previous = positions
                    trajPlanner.generate(expTime, sensor.Position, sensor.Velocity);
//                    std::cout << "at " << expTime << " sec ,trajPlanner.phase is " << trajPlanner.phase << std::endl;
                    controller.control_allocation(expTime, sensor.yawFiltered,
                                                  trajPlanner.errors, trajPlanner.phase,
                                                  trajPlanner.rampUpDuration,
                                                  trajPlanner.rampDownDuration, trajPlanner.desiredAccel);

                    m_in_main.lock();
                    commandsToGo = controller.mappedCommands;
                    m_in_main.unlock();
                    for (int i = 0; i < numRigidBodies; i++) {
                        Eigen::Vector3d Offset(trajPlanner.xOffsets.at(i), trajPlanner.yOffsets.at(i), 0.0);
                        rec.appendDesiredPosition(trajPlanner.desiredPose + Offset, i);
//                        std::cout << trajPlanner.desiredPose(2) << std::endl;
                        rec.appendOrientation(orientation_cache.at(i), i);
                        rec.appendHighLevelCommand(controller.fXYZ.at(i), i);
                        rec.appendPositionError(trajPlanner.errors.at(i), i);
                        rec.appendPosition(sensor.Position.at(i), i);
                        rec.appendVelocity(sensor.Velocity.at(i), i);
                        rec.appendYaw(sensor.yawFiltered.at(i), i);
                        rec.appendCommand(controller.mappedCommands.at(i), i);
                        rec.appendTrackingFlag(trackingFlags_cache.at(i), i);
                        rec.appendTime(expTime, i);
                    }
                    }
                loop_counter++;
//                    std::cout << "loop_counter " << loop_counter << std::endl;
                if (loop_counter % 1000 == 0) {
                    time_temp = std::chrono::high_resolution_clock::now();
                    auto time_span = std::chrono::duration_cast<std::chrono::duration<double>>(time_temp - expInitTime);

//                        std::cout << "time_span.count()" << time_span.count() << std::endl;
                    std::cout << "Average loop rate (main thread) is " << loop_counter / (time_span.count()) << "Hz"
                              << std::endl;
                }
                ready_event = false;
                lk.unlock();
            }
            else {
                // Case1: Experiment is completed
                if (!trajPlanner.ARM_FLAG && !trajPlanner.FAILSAFE_FLAG && !sensor.FAILSAFE_FLAG) {
                    std::cout << "Experiment Completed successfully" << std::endl;
                }
                    // Case2: Failsafe triggered
                else if (sensor.FAILSAFE_FLAG) {
                    std::cout << "Failsafe, root cause: camera system lost track of at least one copter" << std::endl;
                } else {
                    std::cout << "trajPlanner.ARM_FLAG is " << trajPlanner.ARM_FLAG << std::endl;
                    std::cout << "trajPlanner.FAILSAFE_FLAG is " << trajPlanner.FAILSAFE_FLAG << std::endl;
                    std::cout << "sensor.FAILSAFE_FLAG" << sensor.FAILSAFE_FLAG << std::endl;
                    std::cout << "Failsafe, root cause: large deviation from the virtual points" << std::endl;
                }
                stopExperiment = true;
                for (int i = 0; i < numRigidBodies; i++) {
                    commandsToGo.at(i).roll = 0.0;
                    commandsToGo.at(i).pitch = 0.0;
                    commandsToGo.at(i).throttle = 0;
                    commandsToGo.at(i).yawRate = 0.0;
                }
                rec.saveDataToFile();
                rec.printVariableNames();
//                rec.generatePlots();
                break;
            }
//            loop_counter++;
//            if (loop_counter % 1000 == 0) {
//                time_temp = std::chrono::high_resolution_clock::now();
//                auto time_span = std::chrono::duration_cast<std::chrono::duration<double>>(time_temp - expInitTime);
//                std::cout << "loop_counter " << loop_counter << std::endl;
//                std::cout << "time_span.count()" << time_span.count() << std::endl;
//                std::cout << "Average loop rate (main thread) is " << loop_counter / (time_span.count()) << "Hz"
//                          << std::endl;
//            }
        }
        else {
            std::cout << "Main thread is closing due to keyboard interrupt" << std::endl;
            rec.saveDataToFile();
            break;
        }
    }

    return;
}

void comThread_run(){
    int rate = 100;
    int loop_counter = 0;
    std::chrono::high_resolution_clock::time_point start;
    std::chrono::high_resolution_clock::time_point time_temp;
    start = std::chrono::high_resolution_clock::now();
    while (!EmergencyStopExperiment && !stopExperiment) {
        for (int i = 0; i < numRigidBodies; i++) {
            cfs.at(i).sendSetpoint(commandsToGo.at(i).roll, -commandsToGo.at(i).pitch,
                                   commandsToGo.at(i).yawRate, commandsToGo.at(i).throttle);
//            std::cout << "roll: " << commandsToGo.at(i).roll << ", pitch: " << commandsToGo.at(i).pitch << ", yawRate: " << commandsToGo.at(i).yawRate << ", throttle: " << commandsToGo.at(i).throttle << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds (1000/rate-numRigidBodies));
//        if (loop_counter % 1000 == 0) {
//            time_temp = std::chrono::high_resolution_clock::now();
//            auto time_span = std::chrono::duration_cast<std::chrono::duration<double>>(time_temp - start);
//
////                        std::cout << "time_span.count()" << time_span.count() << std::endl;
//            std::cout << "Average loop rate (com thread) is " << loop_counter / (time_span.count()) << "Hz"
//                      << std::endl;
//        }
//        loop_counter++;
    }
    for (int i = 0; i < numRigidBodies; i++) {
        cfs.at(i).sendSetpoint(0.0,0.0,0.0,0);
    }
    std::cout << "comThread is finished" << std::endl;
}



int main() {

    std::vector <std::string> uri_list;
    uri_list.emplace_back("radio://0/80/2M/E7E7E7E7E0");
    uri_list.emplace_back("radio://0/80/2M/E7E7E7E7E3");
    uri_list.emplace_back("radio://0/80/2M/E7E7E7E7E6");
//    uri_list.emplace_back("radio://0/80/2M/E7E7E7E7E6");
//    uri_list.emplace_back("radio://0/80/2M");

    int numCopters = uri_list.size();
    std::cout << "numCopters" << numCopters << std::endl;
    numRigidBodies = numCopters;

    std::cout << "Initializing Crazyflies ---> Rebooting and Sending Zero Commands" << std::endl;
    for (int i = 0; i < numCopters; i++) {
        Crazyflie temp_cf(uri_list.at(i));
        cfs.push_back(temp_cf);
        cfs.at(i).reboot();
        // Let's prepare the for the flight
        cfs.at(i).sendSetpoint(0.0,0.0,0.0,0);
    }

    std::this_thread::sleep_for(std::chrono::seconds(3));


    //initializing global variables
    for (int i = 0; i < numCopters; i++) {
        positions.push_back(Eigen::Vector3d::Zero());
        orientations.push_back(Eigen::Vector4d::Zero());
        trackingFlags.push_back(false);
        commandsToGo.emplace_back(0.0,0.0,0,0.0);
    }
    expTime = 0.0;
    std::cout << "Global Variables Initialized" << std::endl;

    //Objects to be passed to mainThread
    Sensor sensor(numCopters);
    Trajectory_Planner trajPlanner;
    PB_Control controller(numCopters);
    Recorder rec(numCopters);

    std::this_thread::sleep_for(std::chrono::seconds (1));

    std::cout << "Creating NatNet object and connecting to Motive" << std::endl;

    // Connecting to NatNet manually
    // The following values must be read from Motive (LAN Network)
    const char* serverAddress = "192.168.1.2";
    const char* multicastAddress = "239.255.42.99";
    uint16_t commandPort = 1510;
    uint16_t dataPort = 1511;

    // create NatNet client
    g_pClient = new NatNetClient();

    // set the frame callback handler
    g_pClient->SetFrameReceivedCallback( receiveRigidBodyFrame, g_pClient );	// this function will receive data from the server

    g_connectParams.connectionType = kDefaultConnectionType;
    g_connectParams.serverAddress = serverAddress;
    g_connectParams.serverCommandPort = commandPort;
    g_connectParams.serverDataPort = dataPort;
    g_connectParams.multicastAddress = multicastAddress;
    // Let's connect to Motive

    int iResult;
    iResult = ConnectClient();
    if (iResult != ErrorCode_OK)
    {
        printf("Error initializing client.  See log for details.  Exiting");
        return 1;
    }
    else
    {
        printf("Client initialized and ready.\n");
    }

    std::cout << "Listening to Motive has been started" << std::endl;

    std::cout << "Use Ctrl-C to stop the program" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds (1));

    signal(SIGINT, signal_callback_handler); //This will help us to close the program controllably (with ctrl+c)

    std::thread comThread(comThread_run);
    std::thread mainThread(mainThread_run,
                           std::ref(sensor),
                           std::ref(trajPlanner),
                           std::ref(controller),
                           std::ref(rec));



    comThread.join();
    mainThread.join();


    std::cout << "okay let's disconnect" << std::endl;
    if (g_pClient)
    {
        g_pClient->Disconnect();
        delete g_pClient;
        g_pClient = NULL;
    }


    std::cout << "main function successfully finished" << std::endl;
    return 0;
}

void NATNET_CALLCONV receiveRigidBodyFrame(sFrameOfMocapData* data, void* pUserData) {
    std::lock_guard<std::mutex> lk(m_event);
//    IsNatNetReceiverInCall = true;
    NatNetClient* pClient = (NatNetClient*) pUserData;
    // FrameOfMocapData params
    bool bIsRecording = ((data->params & 0x01)!=0);
    bool bTrackedModelsChanged = ((data->params & 0x02)!=0);
    if(bIsRecording)
        printf("RECORDING\n");
    if(bTrackedModelsChanged)
        printf("Models Changed.\n");
    // Rigid Bodies
//    printf("Rigid Bodies [Count=%d]\n", data->nRigidBodies);
    if (numRigidBodies != data->nRigidBodies) {
        std::cout << "numRigidBodies is " << numRigidBodies << std::endl;
        std::cout << "Number of received rigid bodies does not match the expectarion" << std::endl;
        return;
    }
    std::lock_guard<std::mutex> lg(m);
    for(int i=0; i < data->nRigidBodies; i++) {
        trackingFlags.at(i) = data->RigidBodies[i].params & 0x01;
        positions.at(i)(0) = data->RigidBodies[i].x;
        positions.at(i)(1) = -data->RigidBodies[i].z;
        positions.at(i)(2) = data->RigidBodies[i].y;
        orientations.at(i)(0) = data->RigidBodies[i].qx;
        orientations.at(i)(1) = data->RigidBodies[i].qy;
        orientations.at(i)(2) = data->RigidBodies[i].qz;
        orientations.at(i)(3) = data->RigidBodies[i].qw;
    }
//    position_storage.push_back(positions);
//    IsNatNetReceiverInCall = false;
    ready_event = true;
    cv_event.notify_one();
}


// Establish a NatNet Client connection
int ConnectClient()
{
    // Release previous server
    g_pClient->Disconnect();

    // Init Client and connect to NatNet server
    int retCode = g_pClient->Connect( g_connectParams );
    printf("inside connectclient\n");
    if (retCode != ErrorCode_OK)
    {
        printf("Unable to connect to server.  Error code: %d. Exiting", retCode);
        return ErrorCode_Internal;
    }
    else
    {
        // connection succeeded

        void* pResult;
        int nBytes = 0;
        ErrorCode ret = ErrorCode_OK;

        // print server info
        memset( &g_serverDescription, 0, sizeof( g_serverDescription ) );
        ret = g_pClient->GetServerDescription( &g_serverDescription );
        if ( ret != ErrorCode_OK || ! g_serverDescription.HostPresent )
        {
            printf("Unable to connect to server. Host not present. Exiting.");
            return 1;
        }
        printf("\n[SampleClient] Server application info:\n");
        printf("Application: %s (ver. %d.%d.%d.%d)\n", g_serverDescription.szHostApp, g_serverDescription.HostAppVersion[0],
               g_serverDescription.HostAppVersion[1], g_serverDescription.HostAppVersion[2], g_serverDescription.HostAppVersion[3]);
        printf("NatNet Version: %d.%d.%d.%d\n", g_serverDescription.NatNetVersion[0], g_serverDescription.NatNetVersion[1],
               g_serverDescription.NatNetVersion[2], g_serverDescription.NatNetVersion[3]);
        printf("Client IP:%s\n", g_connectParams.localAddress );
        printf("Server IP:%s\n", g_connectParams.serverAddress );
        printf("Server Name:%s\n", g_serverDescription.szHostComputerName);

        // get mocap frame rate
        ret = g_pClient->SendMessageAndWait("FrameRate", &pResult, &nBytes);
        if (ret == ErrorCode_OK)
        {
            float fRate = *((float*)pResult);
            printf("Mocap Framerate : %3.2f\n", fRate);
        }
        else
            printf("Error getting frame rate.\n");

        // get # of analog samples per mocap frame of data
        ret = g_pClient->SendMessageAndWait("AnalogSamplesPerMocapFrame", &pResult, &nBytes);
        if (ret == ErrorCode_OK)
        {
            g_analogSamplesPerMocapFrame = *((int*)pResult);
            printf("Analog Samples Per Mocap Frame : %d\n", g_analogSamplesPerMocapFrame);
        }
        else
            printf("Error getting Analog frame rate.\n");
    }

    return ErrorCode_OK;
}
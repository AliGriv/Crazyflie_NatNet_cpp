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
bool stopExperiment = false;
//NatNet Variables
static const ConnectionType kDefaultConnectionType = ConnectionType_Multicast;
NatNetClient* g_pClient = NULL;
sNatNetClientConnectParams g_connectParams;
sServerDescription g_serverDescription;
int g_analogSamplesPerMocapFrame = 0;
bool IsNatNetReceiverInCall = false;
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
    stopExperiment = true;
}
void mainThread_run(Sensor &sensor,
                    Trajectory_Planner &trajPlanner,
                    PB_Control &controller,
                    Recorder &rec){
    int loop_counter = 0;
    std::vector <Eigen::Vector3d> position_cache;
    std::vector <Eigen::Vector4d> orientation_cache;
    std::vector <bool> trackingFlags_cache;
    std::chrono::steady_clock::time_point expInitTime;
    std::chrono::steady_clock::time_point time_temp;

    if (!stopExperiment) { //proceed with natural main code
        if (trajPlanner.ARM_FLAG && !trajPlanner.FAILSAFE_FLAG && !sensor.FAILSAFE_FLAG) {
            if (!IsNatNetReceiverInCall) {
                //I hope this helps, as a sort of event handling procedure
                position_cache = positions;
                orientation_cache = orientations;
                trackingFlags_cache = trackingFlags;
                if (!sensor.initFlag) {
                    sensor.process(position_cache, orientation_cache, trackingFlags_cache);
                    //precious_positions = position;
                    expInitTime = std::chrono::steady_clock::now();
                }
                else {
                    time_temp = std::chrono::steady_clock::now();
                    std::chrono::steady_clock::duration time_span = time_temp - expInitTime;
                    expTime = time_span.count();
                    sensor.process(position_cache, orientation_cache, trackingFlags_cache);
                    //positions_previous = positions
                    trajPlanner.generate(expTime, sensor.Position, sensor.Velocity);
                    controller.control_allocation(expTime, sensor.yawFiltered,
                                                  trajPlanner.errors, trajPlanner.phase,
                                                  trajPlanner.rampUpDuration,
                                                  trajPlanner.rampDownDuration);

                    m_in_main.lock();
                    commandsToGo = controller.mappedCommands;
                    m_in_main.unlock();


                }
            }
        }
    }
    else {
        std::cout << "Main thread is closing due to keyboard interrupt" << std::endl;
        return;
    }


    return;
}

void comThread_run(){
    int rate = 100;
    int loop_counter = 0;

    while (!stopExperiment) {
        for (int i = 0; i < numRigidBodies; i++) {
            cfs.at(i).sendSetpoint(commandsToGo.at(i).roll, commandsToGo.at(i).pitch,
                                   commandsToGo.at(i).yawRate, commandsToGo.at(i).throttle);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds (1000*(1/rate)-numRigidBodies));

        loop_counter++;
    }
    for (int i = 0; i < numRigidBodies; i++) {
        cfs.at(i).sendSetpoint(0.0,0.0,0.0,0);
    }
    std::cout << "comThread is finished" << std::endl;
}



int main() {

    std::vector <std::string> uri_list;
    uri_list.emplace_back("radio://0/80/2M/E7E7E7E7E3");
    uri_list.emplace_back("radio://0/80/2M/E7E7E7E7E6");

    int numCopters = uri_list.size();
    int numRigidBodies = numCopters;

    for (int i = 0; i < numCopters; i++) {
        Crazyflie temp_cf(uri_list.at(i));
        cfs.push_back(temp_cf);
        cfs.at(i).reboot();
        // Let's prepare the for the flight
        cfs.at(i).sendSetpoint(0.0,0.0,0.0,0);
    }


    //initializing global variables
    for (int i = 0; i < numCopters; i++) {
        positions.push_back(Eigen::Vector3d::Zero());
        orientations.push_back(Eigen::Vector4d::Zero());
        trackingFlags.push_back(false);
        commandsToGo.emplace_back(0.0,0.0,0,0.0);
    }
    expTime = 0.0;

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


    std::thread comThread(comThread_run);
    std::thread mainThread(mainThread_run,
                           std::ref(sensor),
                           std::ref(trajPlanner),
                           std::ref(controller),
                           std::ref(rec));


    comThread.join();
    mainThread.join();
    signal(SIGINT, signal_callback_handler); //This will help us to close the program controllably (with ctrl+c)

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
    IsNatNetReceiverInCall = true;
    NatNetClient* pClient = (NatNetClient*) pUserData;
    // FrameOfMocapData params
    bool bIsRecording = ((data->params & 0x01)!=0);
    bool bTrackedModelsChanged = ((data->params & 0x02)!=0);
    if(bIsRecording)
        printf("RECORDING\n");
    if(bTrackedModelsChanged)
        printf("Models Changed.\n");
    // Rigid Bodies
    printf("Rigid Bodies [Count=%d]\n", data->nRigidBodies);
    if (numRigidBodies != data->nRigidBodies) {
        std::cout << "Number of received rigid bodies does not match the expectarion" << std::endl;
        return;
    }
    std::lock_guard<std::mutex> lg(m);
    for(int i=0; i < data->nRigidBodies; i++) {
        trackingFlags.at(i) = data->RigidBodies[i].params & 0x01;
        positions.at(i)(0) = data->RigidBodies[i].x;
        positions.at(i)(1) = data->RigidBodies[i].y;
        positions.at(i)(2) = data->RigidBodies[i].z;
        orientations.at(i)(0) = data->RigidBodies[i].qx;
        orientations.at(i)(1) = data->RigidBodies[i].qy;
        orientations.at(i)(2) = data->RigidBodies[i].qz;
        orientations.at(i)(3) = data->RigidBodies[i].qw;
    }
//    position_storage.push_back(positions);
    IsNatNetReceiverInCall = false;
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
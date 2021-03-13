//
// Created by AliGriv on 2021-03-11.
//
#include <iostream>

#include <inttypes.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <thread>
#include <mutex>
#include "NatNetUtility.h"
#include "NatNetTypes.h"
#include "NatNetCAPI.h"
#include "NatNetClient.h"
#include <vector>
#include <Eigen/Dense>
#include <chrono>


static const ConnectionType kDefaultConnectionType = ConnectionType_Multicast;
NatNetClient* g_pClient = NULL;
sNatNetClientConnectParams g_connectParams;
sServerDescription g_serverDescription;
int g_analogSamplesPerMocapFrame = 0;


int numRigidBodies;
std::vector <Eigen::Vector3d> positions;
std::vector <bool> trackingFlag;
std::vector <Eigen::Vector4d> orientations;
std::vector <std::vector <Eigen::Vector3d>> position_storage;

void NATNET_CALLCONV receiveRigidBodyFrame(sFrameOfMocapData* data, void* pUserData);
int ConnectClient();
int main (){

    numRigidBodies = 1;
    //Initialize global variables
    for (int i = 0; i < numRigidBodies; i++) {
        positions.push_back(Eigen::Vector3d::Zero());
        orientations.push_back(Eigen::Vector4d::Zero());
        trackingFlag.push_back(false);
    }
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

    std::cout << "Let's listen to the data and collect them for 60 seconds" << std::endl;

    auto start = std::chrono::steady_clock::now();
    while (true) {
        if(std::chrono::steady_clock::now() - start > std::chrono::seconds(60)) {
            break;
        }
    }

    std::cout << "okay let's disconnect" << std::endl;
    if (g_pClient)
    {
        g_pClient->Disconnect();
        delete g_pClient;
        g_pClient = NULL;
    }

    std::cout << "There are " << position_storage.size() << " samples stored for position of rigid bodies" << std::endl;

    return 0;
}

void NATNET_CALLCONV receiveRigidBodyFrame(sFrameOfMocapData* data, void* pUserData) {
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
    for(int i=0; i < data->nRigidBodies; i++) {
        trackingFlag.at(i) = data->RigidBodies[i].params & 0x01;
        positions.at(i)(0) = data->RigidBodies[i].x;
        positions.at(i)(1) = data->RigidBodies[i].y;
        positions.at(i)(2) = data->RigidBodies[i].z;
        orientations.at(i)(0) = data->RigidBodies[i].qx;
        orientations.at(i)(1) = data->RigidBodies[i].qy;
        orientations.at(i)(2) = data->RigidBodies[i].qz;
        orientations.at(i)(3) = data->RigidBodies[i].qw;
    }
    position_storage.push_back(positions);
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
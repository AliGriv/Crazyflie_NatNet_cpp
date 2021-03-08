//
// Created by AliGriv on 2021-02-09.
//

#ifndef NATNET_EXAMPLE_NATNETUTILITY_H
#define NATNET_EXAMPLE_NATNETUTILITY_H

#include <iostream>
#include <vector>
#include "NatNetTypes.h"
#include "NatNetCAPI.h"
#include "NatNetClient.h"

class data_NatNet;

int run_NatNet( int argc, std::vector<char*> argv );
std::vector<data_NatNet> get_data();
int get_data_size();

std::vector <std::vector <data_NatNet>>* data_vec = new std::vector <std::vector <data_NatNet>>;
bool IsRunning = false;



class data_NatNet {
public:
    std::vector<double> pos{std::vector<double>(3,0.0)};
    std::vector<double> orient{std::vector<double>(4,0.0)};
    bool bTrackingValid;
    int ID;
};
data_NatNet* test_data = new data_NatNet;

void clean_data(){
    delete data_vec;
    delete test_data;
}
//class client_NatNet {
//public:
//    static int initialization_flag;
//    NatNetClient* g_pClient;
//    static std::vector <data_NatNet> data_vec;
//
//    client_NatNet(){
//        g_pClient = new NatNetClient;
//    };
//
//    void run();
////    void NATNET_CALLCONV DataHandler2(sFrameOfMocapData* data, void* pUserData);
//    std::vector<data_NatNet> getData_NatNet();
//    void test_function();
//    ~client_NatNet() {
//        std::cout << "Destructor called" << std::endl;
//        if (g_pClient)
//        {
//            g_pClient->Disconnect();
//            delete g_pClient;
//            std::cout << "g_pClient is true" << std::endl;
//            g_pClient = NULL;
//        }
//    };
//private:
//};


//class client_NatNet : public NatNetClient {
//public:
//    static int initialization_flag;
//    static std::vector <data_NatNet> data_vec;
//
//    client_NatNet(){};
//
//    void run();
////    void NATNET_CALLCONV DataHandler2(sFrameOfMocapData* data, void* pUserData);
//    std::vector<data_NatNet> getData_NatNet();
//    void test_function();
//    ~client_NatNet() {
//        std::cout << "Destructor called" << std::endl;
////        if (g_pClient)
////        {
////            g_pClient->Disconnect();
////            delete g_pClient;
////            std::cout << "g_pClient is true" << std::endl;
////            g_pClient = NULL;
////        }
//    };
//private:
//};

//int client_NatNet::initialization_flag = 0;
//std::vector <data_NatNet> client_NatNet::data_vec {std::vector<data_NatNet> (1,data_NatNet())};
//


class test_class {
public:
    void do_something();
};


//NatNetClient* create_NatNetClient() {
//    NatNetClient* g_pClient = new NatNetClient();
//
//    return g_pClient;
//}
//
////void delete_NatNetClient()
//
//void run_NatNetClient(NatNetClient* g_pClient) {
//
//    // set the frame callback handler
//    g_pClient->SetFrameReceivedCallback( DataHandler, g_pClient );	// this function will receive data from the server
//
//    // Do asynchronous server discovery.
//    printf( "Looking for servers on the local network.\n" );
//    printf( "The first Motive that is found will be connected\n" );
//
//    NatNetDiscoveryHandle discovery;
//    NatNet_CreateAsyncServerDiscovery( &discovery, ServerDiscoveredCallback );
//
//    const int c = '1';
//    const size_t serverIndex = c - '1';
//    if ( serverIndex < g_discoveredServers.size() ) {
//        const sNatNetDiscoveredServer &discoveredServer = g_discoveredServers[serverIndex];
//
//        if (discoveredServer.serverDescription.bConnectionInfoValid) {
//            // Build the connection parameters.
//#ifdef _WIN32
//            _snprintf_s(
//#else
//            snprintf(
//#endif
//                    g_discoveredMulticastGroupAddr, sizeof g_discoveredMulticastGroupAddr,
//                    "%" PRIu8 ".%" PRIu8".%" PRIu8".%" PRIu8"",
//                    discoveredServer.serverDescription.ConnectionMulticastAddress[0],
//                    discoveredServer.serverDescription.ConnectionMulticastAddress[1],
//                    discoveredServer.serverDescription.ConnectionMulticastAddress[2],
//                    discoveredServer.serverDescription.ConnectionMulticastAddress[3]
//            );
//
//            g_connectParams.connectionType = discoveredServer.serverDescription.ConnectionMulticast
//                                             ? ConnectionType_Multicast : ConnectionType_Unicast;
//            g_connectParams.serverCommandPort = discoveredServer.serverCommandPort;
//            g_connectParams.serverDataPort = discoveredServer.serverDescription.ConnectionDataPort;
//            g_connectParams.serverAddress = discoveredServer.serverAddress;
//            g_connectParams.localAddress = discoveredServer.localAddress;
//            g_connectParams.multicastAddress = g_discoveredMulticastGroupAddr;
//        } else {
//            // We're missing some info because it's a legacy server.
//            // Guess the defaults and make a best effort attempt to connect.
//            g_connectParams.connectionType = kDefaultConnectionType;
//            g_connectParams.serverCommandPort = discoveredServer.serverCommandPort;
//            g_connectParams.serverDataPort = 0;
//            g_connectParams.serverAddress = discoveredServer.serverAddress;
//            g_connectParams.localAddress = discoveredServer.localAddress;
//            g_connectParams.multicastAddress = NULL;
//        }
//    }
//    else {
//        printf("There is something wrong with the number of discovered servers\n");
//    }
//    NatNet_FreeAsyncServerDiscovery( discovery );
//
//    int iResult;
//
//    // Connect to Motive
//    printf("Now trying to connect to motive");
//    iResult = ConnectClient(g_pClient);
//    if (iResult != ErrorCode_OK)
//    {
//        printf("Error initializing client.  See log for details.  Exiting");
////        return 1;
//    }
//    else
//    {
//        printf("Client initialized and ready.\n");
//    }
//    // Send/receive test request
//    void* response;
//    int nBytes;
//    printf("[SampleClient] Sending Test Request\n");
//    iResult = g_pClient->SendMessageAndWait("TestRequest", &response, &nBytes);
//    if (iResult == ErrorCode_OK)
//    {
//        printf("[SampleClient] Received: %s", (char*)response);
//    }
//
//}


















#endif //NATNET_EXAMPLE_NATNETUTILITY_H

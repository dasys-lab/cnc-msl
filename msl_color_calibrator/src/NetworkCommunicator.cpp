#include "NetworkCommunicator.h"

#include <sys/socket.h> // Needed for the socket functions
#include <netdb.h>      // Needed for the socket functions

#include <iostream>
#include <sstream>

#include <pthread.h>

using namespace std;

NetworkCommunicator* NetworkCommunicator::m_instance;

std::queue<int> NetworkCommunicator::receiverIDs;

NetworkCommunicator *NetworkCommunicator::getInstance() {
    if (m_instance == NULL) {
        m_instance = new NetworkCommunicator();
    }
    return m_instance;
}

NetworkCommunicator::NetworkCommunicator() {

}

void NetworkCommunicator::requestImage(int receiverID) {
    receiverIDs.push(receiverID);
    pthread_t serverThread;
    pthread_create(&serverThread, NULL, &NetworkCommunicator::getImageFromClient, NULL);
}

void *NetworkCommunicator::getImageFromClient(void *arg) {
    int receiverID = receiverIDs.front();
    receiverIDs.pop();

    int status;
    struct addrinfo host_info;
    struct addrinfo *host_info_list;

    memset(&host_info, 0, sizeof host_info);

    // Setting up the structs..."
    host_info.ai_family = AF_UNSPEC;
    host_info.ai_socktype = SOCK_STREAM;

    std::ostringstream oss;
    oss << CC_ROBOT_IP_BASE_0 << "." <<
            CC_ROBOT_IP_BASE_1 << "." <<
            CC_ROBOT_IP_BASE_2 << "."
     << (CC_ROBOT_IP_BASE_3 /*+ receiverID*/);
    string hostname = oss.str();
    cout << hostname << endl;
    oss.str("");
    oss.clear();
    oss << CC_ROBOT_PORT;
    string port = oss.str();
    status = getaddrinfo(hostname.c_str(), port.c_str(), &host_info, &host_info_list);
    if (status != 0) {
        std::cout << "getaddrinfo error" << gai_strerror(status) << std::endl;
        pthread_exit((void *) -1);
    }

    // Creating a socket..."
    int socketfd;
    socketfd = socket(host_info_list->ai_family, host_info_list->ai_socktype, host_info_list->ai_protocol);
    if (socketfd == -1) {
        std::cout << hostname << ": socket error " << std::endl;
        pthread_exit((void *) -2);
    }

    // Connect()ing..."
    status = ::connect(socketfd, host_info_list->ai_addr, host_info_list->ai_addrlen);
    if (status == -1) {
        std::cout << hostname << ": connect error" << std::endl;
        pthread_exit((void *) -3);
    }

    // Waiting to recieve data...
    ssize_t bytes_recieved;
    int buffersize = CC_IMAGE_WIDTH * CC_IMAGE_HEIGHT * 3;
    char incomming_data_buffer[buffersize];
    unsigned char * content = (unsigned char *) malloc(CC_IMAGE_WIDTH * CC_IMAGE_HEIGHT * 3);
    memset(content, ' ', sizeof(content));
    int i = 0;
    while ((bytes_recieved = recv(socketfd, incomming_data_buffer, buffersize, 0)) > 0) {
        for (int j = 0; j < bytes_recieved; ++j) {
            content[i] = incomming_data_buffer[j];
            i++;
        }
    }
    if (bytes_recieved == -1) {
        std::cout << hostname << ": recieve error!" << std::endl;
        pthread_exit((void *) -4);
    }

    // Receiving complete. Closing socket...
    freeaddrinfo(host_info_list);
    ::close(socketfd);

    // Building the image...
    ImageResource* img = new ImageResource(content, CC_IMAGE_WIDTH, CC_IMAGE_HEIGHT);
    img->setSender(receiverID);
    img->setSenderIsFixed(true);
    img->m_filename = "";
    free(content);

    getInstance()->emit receivedImage(img);
}

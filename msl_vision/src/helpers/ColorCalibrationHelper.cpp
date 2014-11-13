#include "ColorCalibrationHelper.h"

#include <pthread.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <time.h>

#include <iostream>

using namespace ColorCalibration;

pthread_mutex_t lock;

std::queue<int> ColorCalibrationHelper::connections;

void ColorCalibrationHelper::initialize() {
    pthread_t listenerThread;
    pthread_create(&listenerThread, NULL, &ColorCalibrationHelper::listenerThread, NULL);
}

void *ColorCalibrationHelper::listenerThread(void *threadid) {
    int listenfd = 0, connfd = 0;
    struct sockaddr_in serv_addr;

    listenfd = socket(AF_INET, SOCK_STREAM, 0);
    memset(&serv_addr, '0', sizeof(serv_addr));

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    serv_addr.sin_port = htons(CC_ROBOT_PORT);

    bind(listenfd, (struct sockaddr*)&serv_addr, sizeof(serv_addr));

    listen(listenfd, 10);

    while(1) {
        connfd = accept(listenfd, (struct sockaddr*)NULL, NULL);

        pthread_mutex_lock(&lock);
        connections.push(connfd);
        pthread_mutex_unlock(&lock);
        sleep(1);
    }
}

void ColorCalibrationHelper::sendImage(unsigned char* img, int width, int height) {
    int connfd = 0;
    int buffersize = width * height * 2;
    char sendBuff[buffersize];

    pthread_mutex_lock(&lock);
    if (!connections.empty()) {
        memcpy(sendBuff, img, sizeof(sendBuff));
    }

    while (!connections.empty()) {
        connfd = connections.front();
        connections.pop();

//        std::cout << "ColorCalib " << buffersize << " " << sizeof(sendBuff) << " " << strlen(sendBuff) << std::endl;
//        write(connfd, sendBuff, strlen(sendBuff));
        write(connfd, sendBuff, sizeof(sendBuff));

        close(connfd);
    }
    pthread_mutex_unlock(&lock);
}

#ifndef NETWORKCOMMUNITATOR_H
#define NETWORKCOMMUNITATOR_H

#include <QObject>

#include <queue>

#include "ImageResource.h"
#include "mainwindow.h"
#include <helpers/ColorCalibrationHelper.h>

const int CC_ROBOT_IP_BASE_0 =  10;//172;
const int CC_ROBOT_IP_BASE_1 =  0; //16;
const int CC_ROBOT_IP_BASE_2 =  0; //40;
const int CC_ROBOT_IP_BASE_3 =  1; //10;

class NetworkCommunicator : public QObject {
    Q_OBJECT
public:
    static NetworkCommunicator *getInstance();

    static void requestImage(int receiverID);
Q_SIGNALS:
    void receivedImage(ImageResource *img);

private:
    NetworkCommunicator();

    static NetworkCommunicator* m_instance;

    static std::queue<int> receiverIDs;

    static void *getImageFromClient(void *arg);
};

#endif // NETWORKCOMMUNITATOR_H

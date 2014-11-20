#ifndef ROSCOMMUNICATOR_H
#define ROSCOMMUNICATOR_H

#include <QObject>

#include <vector>

#include "ros/ros.h"
#include <msl_sensor_msgs/CameraSettings.h>
#include <msl_sensor_msgs/CameraSettingsRequest.h>

#include "helpers/CameraCalibrationHelper.h"

class ROSCommunicator : public QObject
{
    Q_OBJECT
public:
    static ROSCommunicator *getInstance();

    static void initialize();

    static bool isInitialized() { return initialized; }

    static bool isROScoreRunning();

    static void requestSettings(std::vector<int>& receiverIDs);

    static void sendSettings(int receiverID, CameraCalibration::Settings* settings);
signals:
    void receivedSettings(const msl_sensor_msgs::CameraSettings::ConstPtr& msg);

private:
    ROSCommunicator();

    static bool rosInitCalled;
    static bool initialized;
    static ROSCommunicator* instance;


    static msl_sensor_msgs::CameraSettings* cameraSettingsMsgs;
    static msl_sensor_msgs::CameraSettingsRequest* cameraSettingsRequestMsgs;

    static ros::Subscriber settingSub;
    static ros::Publisher settingPub;
    static ros::Publisher settingRequestPub;
    static ros::NodeHandle* visionNode;

    static void *rosSpin(void *threadid);

    static void handleSettings(const msl_sensor_msgs::CameraSettings::ConstPtr& msg);
};

#endif // ROSCOMMUNICATOR_H

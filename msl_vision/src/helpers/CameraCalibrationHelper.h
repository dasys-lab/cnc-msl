#ifndef CameraCalibrationHelper_h
#define CameraCalibrationHelper_h

#include "ros/ros.h"
#include <msl_sensor_msgs/CameraSettings.h>
#include <msl_sensor_msgs/CameraSettingsRequest.h>
#include "../driver/imagingsource.h"

namespace CameraCalibration {
    class Settings {
    public:
        bool useBrightness;
        int brightness;
        int exposure;
        bool autoWhiteBalance;
        int whiteBalance1;
        int whiteBalance2;
        int hue;
        int saturation;
        bool enabledGamma;
        int gamma;
        bool autoShutter;
        int shutter;
        bool autoGain;
        int gain;
    };

    class CameraCalibrationHelper {
    public:
        static void initialize();

        static bool settingsAreRequested;
        static bool setSettings;
        static Settings* cameraSettings;

        static msl_sensor_msgs::CameraSettings* cameraSettingsMsgs;

        static ros::Publisher settingPub;
        static ros::Subscriber settingSub;
        static ros::Subscriber settingRequestSub;
        static ros::NodeHandle* visionNode;

        static void sendSettings(Settings* settings);
        static void handleCameraSettings(const msl_sensor_msgs::CameraSettings::ConstPtr& msg);
        static void handleCameraSettingsRequest(const msl_sensor_msgs::CameraSettingsRequest::ConstPtr& msg);
        static void setCameraSettings(camera::ImagingSource* cam);
    };
}

#endif

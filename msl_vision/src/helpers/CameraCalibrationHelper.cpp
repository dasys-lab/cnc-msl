#include "CameraCalibrationHelper.h"

#include <SystemConfig.h>

#include <iostream>

using namespace CameraCalibration;

bool CameraCalibrationHelper::settingsAreRequested;
bool CameraCalibrationHelper::setSettings;
Settings* CameraCalibrationHelper::cameraSettings;

msl_sensor_msgs::CameraSettings* CameraCalibrationHelper::cameraSettingsMsgs;

ros::Publisher CameraCalibrationHelper::settingPub;
ros::Subscriber CameraCalibrationHelper::settingSub;
ros::Subscriber CameraCalibrationHelper::settingRequestSub;
ros::NodeHandle* CameraCalibrationHelper::visionNode;

void CameraCalibrationHelper::initialize() {
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "CNCalibration");
    visionNode = new ros::NodeHandle();
    settingsAreRequested = false;
    setSettings = false;

    cameraSettings = new Settings();
    cameraSettingsMsgs = new msl_sensor_msgs::CameraSettings();
    cameraSettingsMsgs->senderID = supplementary::SystemConfig::getOwnRobotID();
    cameraSettingsMsgs->receiverID = -1;

    settingPub = visionNode->advertise<msl_sensor_msgs::CameraSettings>("CNCalibration/CameraSettings", 1);

    settingSub = visionNode->subscribe<msl_sensor_msgs::CameraSettings>("CNCalibration/CameraSettings", 1, &CameraCalibrationHelper::handleCameraSettings);

    settingRequestSub = visionNode->subscribe<msl_sensor_msgs::CameraSettingsRequest>("CNCalibration/CameraSettingsRequest", 1, &CameraCalibrationHelper::handleCameraSettingsRequest);
}

void CameraCalibrationHelper::sendSettings(Settings* settings) {
    std::cout << "CamCalib\tsendSettings" << std::endl;
    cameraSettingsMsgs->useBrightness = settings->useBrightness;
    cameraSettingsMsgs->brightness = settings->brightness;
    cameraSettingsMsgs->exposure = settings->exposure;
    cameraSettingsMsgs->autoWhiteBalance = settings->autoWhiteBalance;
    cameraSettingsMsgs->whiteBalance1 = settings->whiteBalance1;
    cameraSettingsMsgs->whiteBalance2 = settings->whiteBalance2;
    cameraSettingsMsgs->hue = settings->hue;
    cameraSettingsMsgs->saturation = settings->saturation;
    cameraSettingsMsgs->gamma = settings->gamma;
    cameraSettingsMsgs->autoShutter = settings->autoShutter;
    cameraSettingsMsgs->shutter = settings->shutter;
    cameraSettingsMsgs->autoGain = settings->autoGain;
    cameraSettingsMsgs->gain = settings->gain;
    settingPub.publish(*cameraSettingsMsgs);
}

void CameraCalibrationHelper::handleCameraSettings(const msl_sensor_msgs::CameraSettings::ConstPtr& msg) {
    if(supplementary::SystemConfig::getOwnRobotID() != msg->receiverID) return;
    std::cout << "CamCalib\thandleCameraSettings" << std::endl;
    cameraSettings->useBrightness = msg->useBrightness;
    cameraSettings->brightness = msg->brightness;
    cameraSettings->exposure = msg->exposure;
    cameraSettings->autoWhiteBalance = msg->autoWhiteBalance;
    cameraSettings->whiteBalance1 = msg->whiteBalance1;
    cameraSettings->whiteBalance2 = msg->whiteBalance2;
    cameraSettings->hue = msg->hue;
    cameraSettings->saturation = msg->saturation;
    cameraSettings->gamma = msg->gamma;
    cameraSettings->autoShutter = msg->autoShutter;
    cameraSettings->shutter = msg->shutter;
    cameraSettings->autoGain = msg->autoGain;
    cameraSettings->gain = msg->gain;

    setSettings = true;
}

void CameraCalibrationHelper::handleCameraSettingsRequest(const msl_sensor_msgs::CameraSettingsRequest::ConstPtr& msg) {
    for (int i = 0; i < msg->receiverID.size(); ++i) {
        if (msg->receiverID.at(i) == supplementary::SystemConfig::getOwnRobotID()) {
            std::cout << "CamCalib\thandleCameraSettingsRequest" << std::endl;
            settingsAreRequested = true;
            return;
        }
    }
}

void CameraCalibrationHelper::setCameraSettings(camera::ImagingSource* cam) {
	cout << "CamCalib\tsetSettings" << endl;
	CameraCalibration::Settings* settings = CameraCalibration::CameraCalibrationHelper::cameraSettings;
	// BRIGHTNESS
	if (settings->useBrightness == true) {
		if (cam->getBrightness() != settings->brightness) {
			cout << "CamCalib\tchanging brightness\tfrom: ";
			cout << cam->getBrightness() << " to: ";
			cout << settings->brightness;
			cout << endl;

			cam->setBrightness(settings->brightness);

			CameraCalibration::CameraCalibrationHelper::settingsAreRequested = true;
		}
	}
	// EXPOSURE
	if (cam->getExposure() != settings->exposure) {
		cout << "CamCalib\tchanging exposure\tfrom: ";
		cout << cam->getExposure() << " to: ";
		cout << settings->exposure;
		cout << endl;

		cam->setExposure(settings->exposure);

		CameraCalibration::CameraCalibrationHelper::settingsAreRequested = true;
	}
	// WHITEBALANCE
	if (settings->autoWhiteBalance == false) {
		bool setWB = false;
		camera::ImagingSource::white_balance_t wb = cam->getWhiteBalance();
		if (wb.bu != settings->whiteBalance1) {
			cout << "CamCalib\tchanging whiteBalance1 (bu)\tfrom: ";
			cout << wb.bu << " to: ";
			cout << settings->whiteBalance1;
			cout << endl;

			wb.bu = settings->whiteBalance1;

			setWB = true;
		}
		if (wb.rv != settings->whiteBalance2) {
			cout << "CamCalib\tchanging whiteBalance2 (rv)\tfrom: ";
			cout << wb.rv << " to: ";
			cout << settings->whiteBalance2;
			cout << endl;

			wb.rv = settings->whiteBalance2;

			setWB = true;
		}
		if (setWB) {
			cam->setWhiteBalance(wb);

			CameraCalibration::CameraCalibrationHelper::settingsAreRequested = true;
		}
	}
	// HUE
	if (cam->getHue() != settings->hue) {
		cout << "CamCalib\tchanging hue\tfrom: ";
		cout << cam->getHue() << " to: ";
		cout << settings->hue;
		cout << endl;

		cam->setHue(settings->hue);

		CameraCalibration::CameraCalibrationHelper::settingsAreRequested = true;
	}
	// SATURATION
	if (cam->getSaturation() != settings->saturation) {
		cout << "CamCalib\tchanging saturation\tfrom: ";
		cout << cam->getSaturation() << " to: ";
		cout << settings->saturation;
		cout << endl;

		cam->setSaturation(settings->saturation);

		CameraCalibration::CameraCalibrationHelper::settingsAreRequested = true;
	}
	// ENABLE GAMMA
	if (cam->isGamma() != settings->enabledGamma) {
		cout << "CamCalib\tchanging enabledGamma\tfrom: ";
		cout << (cam->isGamma() ? "true" : "false") << " to: ";
		cout << (settings->enabledGamma ? "true" : "false");
		cout << endl;

		cam->enableGamma(settings->enabledGamma);

		CameraCalibration::CameraCalibrationHelper::settingsAreRequested = true;
	}
	// GAMMA
	if (cam->isGamma() &&
			cam->getGamma() != settings->gamma) {
		cout << "CamCalib\tchanging gamma\tfrom: ";
		cout << cam->getGamma() << " to: ";
		cout << settings->gamma;
		cout << endl;

		cam->setGamma(settings->gamma);

		CameraCalibration::CameraCalibrationHelper::settingsAreRequested = true;
	}
	// AUTOSHUTTER
	if (cam->isAutoShutter() != settings->autoShutter) {
		cout << "CamCalib\tchanging autoShutter\tfrom: ";
		cout << (cam->isAutoShutter() ? "true" : "false") << " to: ";
		cout << (settings->autoShutter ? "true" : "false");
		cout << endl;

		cam->enableAutoShutter(settings->autoShutter);

		CameraCalibration::CameraCalibrationHelper::settingsAreRequested = true;
	}
	// SHUTTER
	if (!cam->isAutoShutter() &&
			cam->getShutter() != settings->shutter) {
		cout << "CamCalib\tchanging shutter\tfrom: ";
		cout << cam->getShutter() << " to: ";
		cout << settings->shutter;
		cout << endl;

		cam->setShutter(settings->shutter);

		CameraCalibration::CameraCalibrationHelper::settingsAreRequested = true;
	}
	// AUTOGAIN
	if (cam->isAutoGain() != settings->autoGain) {
		cout << "CamCalib\tchanging autoGain\tfrom: ";
		cout << (cam->isAutoGain() ? "true" : "false") << " to: ";
		cout << (settings->autoGain ? "true" : "false");
		cout << endl;

		cam->enableAutoGain(settings->autoGain);

		CameraCalibration::CameraCalibrationHelper::settingsAreRequested = true;
	}
	// GAIN
	if (!cam->isAutoGain() &&
			cam->getGain() != settings->gain) {
		cout << "CamCalib\tchanging gain\tfrom: ";
		cout << cam->getGain() << " to: ";
		cout << settings->gain;
		cout << endl;

		cam->setGain(settings->gain);

		CameraCalibration::CameraCalibrationHelper::settingsAreRequested = true;
	}
}

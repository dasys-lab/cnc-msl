/*
 * CarpetCalibratorNodelet.cpp
 *
 *  Created on: May 20, 2015
 *      Author: cn
 */
#include "CarpetCalibratorNodelet.h"

#include <string>
#include <pluginlib/class_list_macros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


//nodelet manager starten:
//rosrun nodelet nodelet manager __name:=nodelet_manager
//
//unser nodelet starten
//rosrun nodelet nodelet load carpet_calibrator/CarpetCalibratorNodelet nodelet_manager __name:=carpet_calibrator
//
//starten von camera1394
//rosrun nodelet nodelet load camera1394/driver nodelet_manager __name:=camera1394
namespace msl_vision
{

	CarpetCalibratorNodelet::CarpetCalibratorNodelet()
	{
		sc = supplementary::SystemConfig::getInstance();
		vision = (*sc)["Vision"];

		mx = vision->get<short>("Vision", "CameraMX", NULL);
		my = vision->get<short>("Vision", "CameraMY", NULL);
		radius = vision->get<short>("Vision", "CameraRadius", NULL);
		area = vision->get<short>("Vision", "ImageArea", NULL);

		segmented = new cv::Mat(area,area,CV_8UC1);
		for(int i = 0; i< area*area; i++) {
			segmented->data[i] = 0;
		}

		filterLines = new FilterLinePointsCalib(area);
	}

	CarpetCalibratorNodelet::~CarpetCalibratorNodelet()
	{
	}

	void CarpetCalibratorNodelet::imageCb(const sensor_msgs::ImageConstPtr& image_msg,
				                             const sensor_msgs::CameraInfoConstPtr& info_msg) {
		const cv::Mat grayMat = cv_bridge::toCvShare(image_msg)->image;
		width = image_msg->width;
		height = image_msg->height;
		int size = info_msg->height * info_msg->width;

		double angle = currAngle + (M_PI / 2);

		double nx = cos(angle);
		double ny = sin(angle);

		double d = nx*mx+ny*my;

		int startIndexX = mx - area/2;
		int startIndexY = my - area/2;

//		cout << "s-indY, s-indX, area: " << startIndexY << ", " << startIndexX << ", " << area << endl;

		//XXX nicht schön, aber macht die Sache einfacher!
		if(startIndexY % 2 != 0)
					startIndexY++;

//		cv::cvtColor(grayMat, grayMat, CV_BGR2RGB); // if needed

		cv::Mat test(area,area,CV_8UC1);

		for(int i = 0; i < area; i++) {
			unsigned char * ptr = &(grayMat.data[((startIndexX + i)*width + startIndexY)]);
			for(int j = 0; j < area; j++, ptr++) {
				test.data[i*area+j] = *ptr;

				if(newAngle) {
					if (-2 <= nx*(i-area/2)+ny*(j-area/2) && nx*(i-area/2)+ny*(j-area/2) <= 2) {
						if(0 <= -ny*(i-area/2)+nx*(j-area/2)) {
							segmented->data[i*area+j] = test.data[i*area+j];
						}
					}
				}
			}
		}
		//cv::imshow("RGB2GRAY", grayMat);
		//cv::imshow("test", test);
		//cv::imshow("segmented", *segmented);
		//cv::waitKey(1);
		/*
		 * jede iteration gerade mit hessischer normalform erstellen.
		 * angle + M_PI/2 mit länge 1 ist normalenvektor
		 * mittelpunkt des bildes liegt immer auf der geraden -> einsetzen und man erhält kleinste distanz d zur geraden
		 * iterieren über das komplette bild und gucken ob pixel in gerade eingesetzt d+-x entfernt ist
		 */


	}

	void CarpetCalibratorNodelet::onInit()
	{

		NODELET_INFO("CarpetCalibrator: Initialize Nodelet");

		const vector<string> argv = this->getMyArgv();
		nh = getNodeHandle();

		it_.reset(new image_transport::ImageTransport(nh));

		int argc = argv.size();

		rawOdomSub = nh.subscribe("/RawOdometry", 10, &CarpetCalibratorNodelet::onRawOdometryInfo, this);
		angleSub = nh.subscribe("/CarpetCalibrator/CarpetAngle", 10, &CarpetCalibratorNodelet::onCarpetCalibratorAngle, this);
		alicaEngineInfoSub = nh.subscribe("/AlicaEngine/AlicaEngineInfo", 10, &CarpetCalibratorNodelet::onAlicaEngineInfo, this);

		std::string yaml_filename;

		deviceThread_ = boost::shared_ptr< boost::thread >
		    (new boost::thread(boost::bind(&msl_vision::CarpetCalibratorNodelet::run, this)));

		image_transport::TransportHints hints("raw", ros::TransportHints(), getNodeHandle());
		sub_camera_ = it_->subscribeCamera("camera/image_raw",1, &CarpetCalibratorNodelet::imageCb,this,  hints);

	}

	void CarpetCalibratorNodelet::run()
	{
		//cout << " carpet blubb"  << endl;
	}
	void CarpetCalibratorNodelet::onCarpetCalibratorAngle(std_msgs::Float64ConstPtr msg) {
		if(msg->data != currAngle) {
			currAngle = msg->data;
			newAngle = true;
		} else {
			newAngle = false;
		}
	}

	void CarpetCalibratorNodelet::onRawOdometryInfo(msl_actuator_msgs::RawOdometryInfoPtr msg) {
		if(lastRawOdom == NULL) {
			firstAngle = msg.get()->position.angle;
			lastRawOdom = msg;
		}
		if(lastRawOdom != NULL) {
			currAngle = msg.get()->position.angle - firstAngle;
			newAngle = true;
		} else {
			newAngle = false;
		}
	}
	void CarpetCalibratorNodelet::onAlicaEngineInfo(alica_ros_proxy::AlicaEngineInfoConstPtr msg) {
		if(msg->currentState.compare("FinishSpin")==0 && !imgSaved){
			string directory = sc->getConfigPath() + sc->getHostname() + "/CarpetCalibImage.raw";

			string pngDir = sc->getConfigPath() + sc->getHostname() + "/CarpetCalibImage.png";
			ofstream ofs(directory);

			vector<int> compression_params;
			compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
			compression_params.push_back(9);
			cv::imwrite(pngDir,*segmented,compression_params);

			if(ofs.is_open()) {
				for(int i = 0; i < area * area; i++) {
					ofs << segmented->data[i];
				}
				ofs.close();
				ROS_INFO("saved carpet calibrator image to file.");
			} else {
				ROS_ERROR("%s", string(string("Couldn't save carpet calibrator image: " + directory)).c_str());
			}
			imgSaved = true;
		}
	}

}

PLUGINLIB_DECLARE_CLASS(carpet_calibrator, msl_vision::CarpetCalibratorNodelet,
						msl_vision::CarpetCalibratorNodelet, nodelet::Nodelet);


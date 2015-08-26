/*
 * CarpetCalibratorNodelet.cpp
 *
 *  Created on: May 20, 2015
 *      Author: cn
 */

#include <string>
#include "CarpetCalibratorNodelet.h"
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <boost/thread.hpp>
#include <signal.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv.h>

//nodelet manager starten:
//rosrun nodelet nodelet manager __name:=nodelet_manager
//
//unser nodelet starten
//rosrun nodelet nodelet load carpet_calibrator/CarpetCalibratorNodelet nodelet_manager __name:=nodelet1
//
//starten von camera1394
//rosrun nodelet nodelet standalone camera1394/driver
namespace msl_vision
{

	CarpetCalibratorNodelet::CarpetCalibratorNodelet()
	{
		lastAngle = 0;
	}

	CarpetCalibratorNodelet::~CarpetCalibratorNodelet()
	{
	}

	void CarpetCalibratorNodelet::imageCb(const sensor_msgs::ImageConstPtr& image_msg,
				                             const sensor_msgs::CameraInfoConstPtr& info_msg) {
		const cv::Mat image = cv_bridge::toCvShare(image_msg)->image;
		YUV422DoublePixel* pColor = (YUV422DoublePixel*) image.data;
		cv::Mat rgbImage;
		cv::Mat image2rgb(image_msg->height, image_msg->width, CV_8UC1);
		cv::Mat image2rgb1(image_msg->height, image_msg->width, CV_8UC1);
		cv::Mat image2rgb2(image_msg->height, image_msg->width, CV_8UC1);
		int counter = 0;
		int size = info_msg->height * info_msg->width;
		for(int i = 0; i < (size); i++) {
			image2rgb.data[i] = image.data[i*3];
			image2rgb1.data[i] = image.data[i*3+1];
			image2rgb2.data[i] = image.data[i*3+2];
			/*image2rgb.data[counter++] = pColor[i].y1;
			image2rgb.data[counter++] = pColor[i].v;
			image2rgb.data[counter++] = pColor[i].u;

			image2rgb.data[counter++] = pColor[i].y2;
			image2rgb.data[counter++] = pColor[i].v;
			image2rgb.data[counter++] = pColor[i].u;*/
		}

		//cv::cvtColor(image2rgb, rgbImage, CV_YUV2BGR); //
		cv::imshow("lol", image2rgb);
		cv::imshow("lol1", image2rgb1);
		cv::imshow("lol2", image2rgb2);
		cv::waitKey(1);
		cout << "Image Callback, size: " << image.size.p[1] << endl;


	}

	void CarpetCalibratorNodelet::onInit()
	{
		NODELET_INFO("CarpetCalibrator: Initialize Nodelet");

		const vector<string> argv = this->getMyArgv();
		nh = getNodeHandle();

		it_.reset(new image_transport::ImageTransport(nh));

		int argc = argv.size();

		rawOdomSub = nh.subscribe("/RawOdometry", 10, &CarpetCalibratorNodelet::onRawOdometryInfo, this);

		std::string yaml_filename;

		deviceThread_ = boost::shared_ptr< boost::thread >
		    (new boost::thread(boost::bind(&msl_vision::CarpetCalibratorNodelet::run, this)));

		image_transport::TransportHints hints("raw", ros::TransportHints(), getNodeHandle());
		sub_camera_ = it_->subscribeCamera("camera/image_raw",1, &CarpetCalibratorNodelet::imageCb,this,  hints);
	}

	void CarpetCalibratorNodelet::run()
	{
		cout << " carpet blubb"  << endl;
	}

	void CarpetCalibratorNodelet::onRawOdometryInfo(msl_actuator_msgs::RawOdometryInfoPtr msg) {
		if(msg.get()->position.angle - lastAngle >= M_PI / 180) {
			lastAngle = msg.get()->position.angle;


		}


	}


}

PLUGINLIB_DECLARE_CLASS(carpet_calibrator, msl_vision::CarpetCalibratorNodelet,
						msl_vision::CarpetCalibratorNodelet, nodelet::Nodelet);


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
		sc = supplementary::SystemConfig::getInstance();
		vision = (*sc)["Vision"];
	}

	CarpetCalibratorNodelet::~CarpetCalibratorNodelet()
	{
	}

	void CarpetCalibratorNodelet::imageCb(const sensor_msgs::ImageConstPtr& image_msg,
				                             const sensor_msgs::CameraInfoConstPtr& info_msg) {
		const cv::Mat image = cv_bridge::toCvShare(image_msg)->image;
		YUV422DoublePixel* pColor = (YUV422DoublePixel*) image.data;
		RGB888Pixel* rgbColor = (RGB888Pixel*) image.data;
		cv::Mat rgbImage;
		cv::Mat grayMat;
		cv::Mat image2rgb(image_msg->height, image_msg->width, CV_8UC3);
		int counter = 0;

		int size = info_msg->height * info_msg->width;

		for(int i = 0; i < (size/2); i++) {
			image2rgb.data[counter++] = pColor[i].y1;
			image2rgb.data[counter++] = pColor[i].v;
			image2rgb.data[counter++] = pColor[i].u;

			image2rgb.data[counter++] = pColor[i].y2;
			image2rgb.data[counter++] = pColor[i].v;
			image2rgb.data[counter++] = pColor[i].u;
		}

		cv::cvtColor(image, rgbImage, CV_RGB2BGR);
		cv::cvtColor(image, grayMat, CV_RGB2GRAY);



		cv::imshow("RGB2BGR", rgbImage);
		cv::imshow("RGB2GRAY", grayMat);

//		cv::imshow("lol1", image2rgb1);
//		cv::imshow("lol2", image2rgb2);
		cv::waitKey(1);
		cout << "Image Callback, size: " << image.size.p[1] << endl;

        short mx = vision->get<short>("Vision", "CameraMX", NULL);
        short my = vision->get<short>("Vision", "CameraMY", NULL);
		short radius = vision->get<short>("Vision", "CameraRadius", NULL);
		short area = vision->get<short>("Vision", "ImageArea", NULL);

		double angle = currAngle + (M_PI / 2);

		double nx = cos(angle);
		double ny = sin(angle);

		double d = nx*mx+ny*my;

		int startIndexX = mx - area/2;
		int startIndexY = my - area/2;

		//XXX nicht schön, aber macht die Sache einfacher!
		if(startIndexY % 2 != 0)
					startIndexY++;

		cv::Mat test(area,area,CV_8UC1);

		for(int i = 0; i < area; i++) {
			unsigned char * ptr = &(grayMat.data[((startIndexX + i)*info_msg->width + startIndexY)]);
			for(int j = 0; j < area; j++) {
				test.data[i*area+j] = *ptr;
				ptr++;



			}

		}

		cv::imshow("test", test);
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
		if(lastRawOdom == NULL) {
			firstAngle = msg.get()->position.angle;
			lastRawOdom = msg;
		}
		if(lastRawOdom != NULL) {
			currAngle = msg.get()->position.angle - firstAngle;
		}
	}


}

PLUGINLIB_DECLARE_CLASS(carpet_calibrator, msl_vision::CarpetCalibratorNodelet,
						msl_vision::CarpetCalibratorNodelet, nodelet::Nodelet);


/*
 * $Id: Filter.cpp 1804 2007-01-07 21:53:55Z saur $
 *
 *
 * Copyright 2005,2006 Carpe Noctem, Distributed Systems Group,
 * University of Kassel. All right reserved.
 *
 * The code is derived from the software contributed to Carpe Noctem by
 * the Carpe Noctem Team.
 *
 * The code is licensed under the Carpe Noctem Userfriendly BSD-Based
 * License (CNUBBL). Redistribution and use in source and binary forms,
 * with or without modification, are permitted provided that the
 * conditions of the CNUBBL are met.
 *
 * You should have received a copy of the CNUBBL along with this
 * software. The license is also available on our website:
 * http://carpenoctem.das-lab.net/license.txt
 *
 *
 * <description>
 */
#include "SpicaHelper.h"
#include <SystemConfig.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include "msl_sensor_msgs/LinePointList.h"

using namespace std;
using namespace msl_sensor_msgs;
using namespace msl_actuator_msgs;

WorldModelData* SpicaHelper::wm;
VisionDebug* SpicaHelper::vdd;
VisionImage* SpicaHelper::vi;
//LinePointMessagePtr SpicaHelper::lpmp;
//Point2dInfoPtrListPtr SpicaHelper::linePointList;
ros::NodeHandle* SpicaHelper::visionNode;
ros::Publisher SpicaHelper::womopub;
ros::Publisher SpicaHelper::statepub;
ros::Publisher SpicaHelper::debugPub;
ros::Publisher SpicaHelper::Imagepub;
ros::Publisher SpicaHelper::linePointsPub;

ros::Subscriber SpicaHelper::VCsub;
ros::Subscriber SpicaHelper::RelocSub;
char SpicaHelper::key;
struct sockaddr_in SpicaHelper::si_other;
int SpicaHelper::s, SpicaHelper::slen;
unsigned char* SpicaHelper::buf;
bool SpicaHelper::reloc;
char SpicaHelper::haveBall;
char SpicaHelper::duel;



void SpicaHelper::initialize() {
	int argc = 0;
	char **argv = NULL;
	ros::init(argc, argv, "CNVision");
	visionNode = new ros::NodeHandle();
	reloc = false;

	VCsub = visionNode->subscribe<msl_sensor_msgs::VisionControl>("/CNVision/VisionControl", 1, &SpicaHelper::handleVisionControl);
    RelocSub = visionNode->subscribe<msl_actuator_msgs::VisionRelocTrigger>("CNActuator/VisionRelocTrigger",
				1, &SpicaHelper::handleVisionRelocTrigger);

	womopub = visionNode->advertise<WorldModelData>("/WorldModel/WorldModelData", 1);
    statepub = visionNode->advertise<VisionGameState>("/CNVision/VisionGameState", 1);
	debugPub = visionNode->advertise<VisionDebug>("/CNVision/VisionDebug", 1);
	Imagepub = visionNode->advertise<VisionImage>("/CNVision/VisionImage", 1);
	linePointsPub = visionNode->advertise<LinePointList>("CNVision/LinePointList", 1);

	wm = new WorldModelData();
	vdd = new VisionDebug();
	vi = new VisionImage();
	//linePointList = lpmp->getLinepoints()->getLps();
	//comm->start();
	key = EOF;

    duel=-1;
    haveBall=-1;
	// Socket for mjpeg stream
	/*slen=sizeof(si_other);
	buf = new unsigned char[1024*1024];
	s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	memset((char *) &si_other, 0, sizeof(si_other));
	si_other.sin_family = AF_INET;
	si_other.sin_port = htons(9999);
	inet_aton("127.0.0.1", &si_other.sin_addr);
	*/
}


void SpicaHelper::sendGameState() {
    VisionGameState vgs;
    vgs.duel = duel;
    vgs.haveball = haveBall;
    statepub.publish(vgs);
}

void SpicaHelper::handleVisionControl(const msl_sensor_msgs::VisionControl::ConstPtr& msg) {
	if(supplementary::SystemConfig::getOwnRobotID() != msg->receiverID) return;
	key = msg->key;
}

void SpicaHelper::handleVisionRelocTrigger(const
		msl_actuator_msgs::VisionRelocTrigger::ConstPtr& msg) {
	if(supplementary::SystemConfig::getOwnRobotID() != msg->receiverID) return;
	reloc = true;
}

void SpicaHelper::sendDebugMsg() {
	std::cout << "Sending LinePointsData " << std::endl;
	debugPub.publish(*vdd);
	//vdd->list.clear();
}

void SpicaHelper::send() {
	std::cout << "Sending WorldModelData " << std::endl;
	womopub.publish(*wm);
	wm->distanceScan.sectors.clear();
}

void SpicaHelper::sendLinePoints(std::vector<LinePoint> linePoints)
{
	msl_sensor_msgs::LinePointList lpl;
	for(LinePoint l :linePoints)
	{
		msl_msgs::Point2dInfo lp;
		lp.x = l.x;
		lp.y = l.y;
		lpl.linePoints.push_back(lp);
	}
	linePointsPub.publish(lpl);
}

void SpicaHelper::streamGreyMJPEG(unsigned char* img, int width, int height) {
	static int run=0;
	vector<int> cpar;
	if(run++%3==0) return;
	cpar.push_back(CV_IMWRITE_JPEG_QUALITY);
	cpar.push_back(45);

	vi->senderID = supplementary::SystemConfig::getOwnRobotID();
	vi->imageData.clear();
	vi->width = width;
	vi->height = height;

	cv::Mat m(height, width, CV_8UC1, img);
	vector<unsigned char> buff;
	cv::imencode(".jpeg", m, buff, cpar);

	for(int i=0; i<buff.size(); i++) {
		//buf[i]=buff[i];
		vi->imageData.push_back(buff[i]);
	}

	Imagepub.publish(*vi);
	//if(sendto(s, buf, buff.size(), 0, (const sockaddr*)&si_other, slen)==-1) cout << width << "Error while streaming image" << endl;
	vi->params.clear();
}


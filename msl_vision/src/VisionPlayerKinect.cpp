/*
 * $Id: VisionPlayer.cpp 2214 2007-04-17 17:44:39Z cn $
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <algorithm>
#include <iostream>
#include <byteswap.h>
#include <vector>
#include <sys/time.h>
#include <signal.h>
#include <libfreenect_sync.h>

#include "XVDisplay.h"
#include "SystemConfig.h"
#include <DateTime.h>

#include "helpers/BallHelperKinect.h"
#include "helpers/TimeHelper.h"
#include "helpers/SpicaDirectedHelper.h"

#define CLUSTER_INCR 2000

using namespace std;

void int_handler(int sig) {
	int mypid = getpid();
	printf("[%i] Ouch - shot in the ...\n", mypid);
	printf("[%i] exit\n", mypid);
	freenect_sync_set_led(LED_OFF, 0);
	freenect_sync_stop();
	exit(2);
}

void visualizeClusters(unsigned char* img, BallHelperKinect* ballHelper,
		bool display_gray, unsigned minClusterSize) {
	vector<int>* m_cluster = ballHelper->getMatrix();
	vector<KinectCluster*>* clustering = ballHelper->getClustering();

	// Clear small clusters and mark others as not small
	int clusterCount = 0;
	for (unsigned i = 0; i < clustering->size(); ++i) {
		if (clustering->at(i)->pixels.size() < minClusterSize) {
			for (unsigned j = 0; j < clustering->at(i)->pixels.size(); ++j) {
				if (m_cluster->at(clustering->at(i)->pixels[j]) < BALL_NSWE)
					m_cluster->at(clustering->at(i)->pixels[j]) = CLUSTER_INCR;
				else
					m_cluster->at(clustering->at(i)->pixels[j]) += CLUSTER_INCR;
			}
		} else {
			clusterCount++;
			for (unsigned j = 0; j < clustering->at(i)->pixels.size(); ++j) {
				if (m_cluster->at(clustering->at(i)->pixels[j]) < BALL_NSWE)
					m_cluster->at(clustering->at(i)->pixels[j]) = CLUSTER_INCR + clusterCount;
				else
					m_cluster->at(clustering->at(i)->pixels[j]) += CLUSTER_INCR;
			}
		}
	}

	ballHelper->doubleRes();
	if(display_gray) {
		for (unsigned i = 0; i < 4*FRAME_PIX; ++i) {
			switch (m_cluster->at(i) - CLUSTER_INCR) {
				case 0 - CLUSTER_INCR:
					img[3 * i + 0] = 0;
					img[3 * i + 1] = 0;
					img[3 * i + 2] = 0;
					break;
				case 0:
					img[3 * i + 0] = 255;
					img[3 * i + 1] = 255;
					img[3 * i + 2] = 255;
					break;
				case BALL_CLUSTER:
				case BALL_CLUSTER - CLUSTER_INCR:
					img[3 * i + 0] = 255;
					img[3 * i + 1] = 0;
					img[3 * i + 2] = 0;
					break;
				case BALL_CENTER:
				case BALL_CENTER - CLUSTER_INCR:
					img[3 * i + 0] = 255;
					img[3 * i + 1] = 255;
					img[3 * i + 2] = 0;
					break;
				case BALL_NSWE:
				case BALL_NSWE - CLUSTER_INCR:
					img[3 * i + 0] = 0;
					img[3 * i + 1] = 255;
					img[3 * i + 2] = 0;
					break;
				default:
					img[3 * i + 0] = 50;
					img[3 * i + 1] = 50;
					img[3 * i + 2] = 50;
					break;
			}
		}
	} else {
		for (unsigned i = 0; i < 4*FRAME_PIX; ++i) {
			switch (m_cluster->at(i) - CLUSTER_INCR) {
				case 0 - CLUSTER_INCR:
					img[3 * i + 0] = 0;
					img[3 * i + 1] = 0;
					img[3 * i + 2] = 0;
					break;
				case 0:
					img[3 * i + 0] = 255;
					img[3 * i + 1] = 255;
					img[3 * i + 2] = 255;
					break;
				case 1:
					img[3 * i + 0] = 25;
					img[3 * i + 1] = 180;
					img[3 * i + 2] = 100;
					break;
				case 2:
					img[3 * i + 0] = 0;
					img[3 * i + 1] = 0;
					img[3 * i + 2] = 255;
					break;
				case 3:
					img[3 * i + 0] = 255;
					img[3 * i + 1] = 127;
					img[3 * i + 2] = 0;
					break;
				case 4:
					img[3 * i + 0] = 255;
					img[3 * i + 1] = 0;
					img[3 * i + 2] = 255;
					break;
				case 5:
					img[3 * i + 0] = 0;
					img[3 * i + 1] = 0;
					img[3 * i + 2] = 100;
					break;
				case 6:
					img[3 * i + 0] = 0;
					img[3 * i + 1] = 100;
					img[3 * i + 2] = 255;
					break;
				case 7:
					img[3 * i + 0] = 0;
					img[3 * i + 1] = 50;
					img[3 * i + 2] = 255;
					break;
				case 8:
					img[3 * i + 0] = 100;
					img[3 * i + 1] = 0;
					img[3 * i + 2] = 255;
					break;
				case 9:
					img[3 * i + 0] = 50;
					img[3 * i + 1] = 0;
					img[3 * i + 2] = 255;
					break;
				case 10:
					img[3 * i + 0] = 255;
					img[3 * i + 1] = 20;
					img[3 * i + 2] = 210;
					break;
				case 11:
					img[3 * i + 0] = 255;
					img[3 * i + 1] = 100;
					img[3 * i + 2] = 100;
					break;
				case 12:
					img[3 * i + 0] = 255;
					img[3 * i + 1] = 80;
					img[3 * i + 2] = 60;
					break;
				case 13:
					img[3 * i + 0] = 150;
					img[3 * i + 1] = 80;
					img[3 * i + 2] = 60;
					break;
				case 14:
					img[3 * i + 0] = 255;
					img[3 * i + 1] = 120;
					img[3 * i + 2] = 60;
					break;
				case 15:
					img[3 * i + 0] = 120;
					img[3 * i + 1] = 60;
					img[3 * i + 2] = 60;
					break;
				case 16:
					img[3 * i + 0] = 55;
					img[3 * i + 1] = 87;
					img[3 * i + 2] = 20;
					break;
				case 17:
					img[3 * i + 0] = 230;
					img[3 * i + 1] = 20;
					img[3 * i + 2] = 220;
					break;
				case BALL_CLUSTER:
				case BALL_CLUSTER - CLUSTER_INCR:
					img[3 * i + 0] = 255;
					img[3 * i + 1] = 0;
					img[3 * i + 2] = 0;
					break;
				case BALL_CENTER:
				case BALL_CENTER - CLUSTER_INCR:
					img[3 * i + 0] = 255;
					img[3 * i + 1] = 255;
					img[3 * i + 2] = 0;
					break;
				case BALL_NSWE:
				case BALL_NSWE - CLUSTER_INCR:
					img[3 * i + 0] = 0;
					img[3 * i + 1] = 255;
					img[3 * i + 2] = 0;
					break;
				default:
					img[3 * i + 0] = 50;
					img[3 * i + 1] = 50;
					img[3 * i + 2] = 50;
					break;
			}
		}
	}
}

void visualizeDepth(unsigned char* img, unsigned short* depth, vector<unsigned short> &gamma) {
	for (int i = 0; i < 4*FRAME_PIX; ++i) {
		int pval = gamma[depth[i]];
		int lb = pval & 0xff;
		switch (pval >> 8) {
			case 0:
				img[3 * i + 0] = 255;
				img[3 * i + 1] = 255 - lb;
				img[3 * i + 2] = 255 - lb;
				break;
			case 1:
				img[3 * i + 0] = 255;
				img[3 * i + 1] = lb;
				img[3 * i + 2] = 0;
				break;
			case 2:
				img[3 * i + 0] = 255 - lb;
				img[3 * i + 1] = 255;
				img[3 * i + 2] = 0;
				break;
			case 3:
				img[3 * i + 0] = 0;
				img[3 * i + 1] = 255;
				img[3 * i + 2] = lb;
				break;
			case 4:
				img[3 * i + 0] = 0;
				img[3 * i + 1] = 255 - lb;
				img[3 * i + 2] = 255;
				break;
			case 5:
				img[3 * i + 0] = 0;
				img[3 * i + 1] = 0;
				img[3 * i + 2] = 255 - lb;
				break;
			default:
				img[3 * i + 0] = 0;
				img[3 * i + 1] = 0;
				img[3 * i + 2] = 0;
				break;
		}
	}
}

void paintPixel(unsigned char* img, int index) {
	img[3 * index + 0] = 0;
	img[3 * index + 1] = 255;
	img[3 * index + 2] = 0;
}

int main(int argc,char *argv[]) {
	signal(SIGINT, int_handler);
	ros::init(argc, argv, "CNVisionPlayerKonect");
	SpicaDirectedHelper sdh();

	bool display_frames = true;
	bool display_cluster = true;
	bool display_gray = false;
	bool display_rgb = false;
	bool display_yuv = false;
	// TODO format of ir output
	bool display_ir = false;
	bool display_depth = false;
	bool simple = false;
	bool offline = false;
	bool fixed_image = false;
	int fixed_imageNr = 0;
	bool log_images = false;
	bool help = false;

	vector<unsigned short> m_gamma(2048);

	if(argc > 1) {
		for(int i = 1; i < argc; ++i) {
			if(string(argv[i]) == "--false")
				display_frames = false;
			if(string(argv[i]) == "--noCluster")
				display_cluster = false;
			if(string(argv[i]) == "--gray") {
				display_cluster = true;
				display_gray = true;
			}
			if(string(argv[i]) == "--rgb") {
				display_rgb = true;
				display_yuv = false;
				display_ir = false;
				offline = false;
			}
			if(string(argv[i]) == "--yuv") {
				display_rgb = false;
				display_yuv = true;
				display_ir = false;
				offline = false;
			}
			if(string(argv[i]) == "--ir") {
				display_rgb = false;
				display_yuv = false;
				display_ir = true;
				offline = false;
			}
			if(string(argv[i]) == "--depth")
				display_depth = true;
			if(string(argv[i]) == "--simple")
				simple = true;
			if(string(argv[i]) == "--offline") {
				offline = true;
				display_rgb = false;
				display_yuv = false;
				display_ir = false;
			}
			if(std::string(argv[i]) == "--fixed") {
				offline = true;
				display_rgb = false;
				display_yuv = false;
				display_ir = false;

				fixed_image = true;
				if(i+1 < argc)
					fixed_imageNr = atoi(argv[++i]);
				if(fixed_imageNr == 0) {
					cerr << "Missing argument for '--fixed'" << endl;
					exit(1);
				}
			}
			if(string(argv[i]) == "--log")
				log_images = true;
			if(string(argv[i]) == "--help")
				help = true;
		}
	}

	if(help) {
		cout << endl << "Options for " << argv[0] << ": " << endl << endl;
		cout << argv[0] << " --help" << endl;
		cout << argv[0] << " [--simple] [--offline|--fixed n|--log] "
			<< "--false" << endl;
		cout << argv[0] << " [--simple] [--offline|--fixed n|--log] "
			<< "[--rgb|--yuv|--ir] [--depth] [--noCluster|--gray]"
			<< endl << endl;
		cout << "\t--false:\thides window" << endl;
		cout << "\t--rgb:\t\tdisplays RGB image" << endl;
		cout << "\t--yuv:\t\tdisplays YUV image" << endl;
		cout << "\t--ir:\t\tdisplays image from ir camera" << endl;
		cout << "\t--depth:\tdisplays depth image" << endl;
		cout << "\t--noCluster:\thides cluster image" << endl;
		cout << "\t--gray:\t\tmakes all non-ball clusers gray in cluster image"
			<< endl;
		cout << "\t--simple:\tuse simple algorithm to find ball in clusters"
			<< endl;
		cout << "\t--offline:\toffline mode using images in $VISION_LOG"
			<< endl << "\t\t\tShowing rgb, yuv or ir image in offline "
			<< "mode is not supported!" << endl;
		cout << "\t--fixed n:\tonly the image n from the $VISION_LOG is "
			<< "used (implies --offline)" << endl;
		cout << "\t--log:\t\tlog images to $VISION_LOG for use in --offline"
			<< endl;
		cout << "\t--help:\t\tshows this help text" << endl << endl;
		exit(0);
	}

	if(!offline && freenect_sync_set_led(LED_BLINK_RED_YELLOW, 0)) {
		cout << "Error: No camera connected?" << endl;
		return -1;
	}

	XVDisplay* xvDisplay = NULL;
	XVDisplay* xvDisplayRGB = NULL;
	XVDisplay* xvDisplayDepth = NULL;
	if(display_frames) {
		if(display_cluster) {
			xvDisplay = new XVDisplay(2*FRAME_W, 2*FRAME_H, XV_UYVY);
			xvDisplay->setTitle((char*)"clustered");
		}

		if(display_rgb || display_yuv || display_ir) {
			xvDisplayRGB = new XVDisplay(2*FRAME_W, 2*FRAME_H, XV_UYVY);
			if(display_rgb)
				xvDisplayRGB->setTitle((char*)"rgb");
			else if(display_yuv)
				xvDisplayRGB->setTitle((char*)"yuv");
			else
				xvDisplayRGB->setTitle((char*)"ir");
		}

		if(display_depth) {
			xvDisplayDepth = new XVDisplay(2*FRAME_W, 2*FRAME_H, XV_UYVY);
			xvDisplayDepth->setTitle((char*)"depth");

			for (unsigned int i = 0; i < 2048; i++)
				m_gamma[i] = pow(i / 2048.0, 3) * 36 * 256;
		}
	}

	unsigned short* depth = NULL;
	unsigned short* depthCopy = NULL;
	unsigned char* imageClustered = NULL;
	unsigned char* imageRGB = NULL;
	unsigned char* imageDepth = NULL;
	if(display_frames) {
		if(display_cluster)
			imageClustered = new unsigned char[4*FRAME_PIX*3];
		if(display_depth) {
			depthCopy = new unsigned short[4*FRAME_PIX];
			imageDepth = new unsigned char[4*FRAME_PIX*3];
		}
	}
	if(offline)
		depth = new unsigned short[4*FRAME_PIX];

	char* logPath = NULL;
	if(offline || log_images) {
		logPath = getenv("VISION_LOG");
		if(logPath == NULL) {
			cerr << "$VISION_LOG is not set!" << endl;
			exit(1);
		}
	}

	SystemConfig* sc = SystemConfig::getInstance();
	BallHelperKinect ballHelper;

	const char* vision = "Vision";
	const char* kinect = "KinectSettings";
	int centerX = (*sc)[vision]->get<int>(vision, kinect, "CenterX", NULL);
	int centerY = (*sc)[vision]->get<int>(vision, kinect, "CenterY", NULL);
	unsigned minClusterSize = (*sc)[vision]->get<unsigned>(vision, kinect,
			"MinClusterSize", NULL);

	int frameCount = 0;
	double clusterTimeSum = 0;
	double ballTimeSum = 0;
	int counter = fixed_imageNr;

	while(ros::ok()) {
		unsigned long long visionTimeOmniCamLong =
				supplementary::DateTime::getUtcNowC();
		visionTimeOmniCamLong -= 1000000;
		TimeHelper::getInstance()->setVisionTimeOmniCam(visionTimeOmniCamLong);

		unsigned timestamp;
		if(!offline) {
			if(freenect_sync_get_depth((void**) &depth, &timestamp, 0,
					FREENECT_DEPTH_11BIT)) {
				cout << "Error: No camera connected?" << endl;
				return -1;
			}

			if(log_images) {
				counter++;
				char filename[20];
				sprintf(filename, "/log-image-%04d.raw", counter);
				char path_filename[strlen(logPath)+strlen(filename)+1];
				strcpy(path_filename, logPath);
				strcat(path_filename, filename);

				FILE* logfile = fopen(path_filename, "w");
				fwrite(depth, sizeof(short), 4*FRAME_PIX, logfile);
				fclose(logfile);
			}
		} else {
			if(!fixed_image)
				counter++;

			char filename[20];
			sprintf(filename, "/log-image-%04d.raw", counter);
			char path_filename[strlen(logPath)+strlen(filename)+1];
			strcpy(path_filename, logPath);
			strcat(path_filename, filename);

			cout << "Processing LogFile: " << path_filename << endl;
			FILE* logfile = fopen(path_filename, "r");

			if(logfile != NULL) {
				int read = fread(depth, sizeof(short), 4*FRAME_PIX, logfile);
				if(read != sizeof(char)*4*FRAME_PIX)
					cout << "Warning: Error while reading from logfile."<< endl;
				fclose(logfile);
			} else {
				cout << "Log file not found ... Restarting" << endl;
				if(!fixed_image)
					counter = 1;

				sprintf(filename, "/log-image-%04d.raw", counter);
				strcpy(path_filename, logPath);
				strcat(path_filename, filename);

				logfile = fopen(path_filename, "r");
				if(logfile == NULL) {
					cerr << "Logfile " << path_filename << " not found!"<< endl;
					exit(1);
				} else
					fclose(logfile);
			}
		}

		if(display_frames) {
			if(display_depth)
				copy(depth, depth+4*FRAME_PIX, depthCopy);
		}

		struct timeval start, end;
		gettimeofday(&start, NULL);

		ballHelper.assignCluster(depth);

		gettimeofday(&end, NULL);
		double time = (end.tv_sec - start.tv_sec) * 1000
				+ (end.tv_usec - start.tv_usec) / 1000.0;
		clusterTimeSum += time;

		if(display_frames) {
			if(display_rgb) {
				unsigned timestamp;
				if(freenect_sync_get_video((void**) &imageRGB, &timestamp, 0,
						FREENECT_VIDEO_RGB)) {
					cout << "Error: No camera connected?" << endl;
					return -1;
				}
			} else if(display_yuv) {
				unsigned timestamp;
				if(freenect_sync_get_video((void**) &imageRGB, &timestamp, 0,
						FREENECT_VIDEO_YUV_RGB)) {
					cout << "Error: No camera connected?" << endl;
					return -1;
				}
			} else if(display_ir) {
				unsigned timestamp;
				if(freenect_sync_get_video((void**) &imageRGB, &timestamp, 0,
						FREENECT_VIDEO_IR_8BIT)) {
					cout << "Error: No camera connected?" << endl;
					return -1;
				}
			}
		}

		gettimeofday(&start, NULL);

		Point p;
		if(simple)
			p = ballHelper.getSimpleBallCluster();
		else
			p = ballHelper.getBallCluster();

		gettimeofday(&end, NULL);
		time = (end.tv_sec - start.tv_sec) * 1000
				+ (end.tv_usec - start.tv_usec) / 1000.0;
		ballTimeSum += time;
		frameCount++;
		cout << "Cluster-time-avg: "<< clusterTimeSum / frameCount<<" ms"<<endl;
		cout << "Ball-time-avg: " << ballTimeSum / frameCount << " ms" << endl;
		if(frameCount > 500) {
			clusterTimeSum = 0;
			ballTimeSum = 0;
			frameCount = 0;
		}

		if(display_frames) {
			if(display_cluster) {
				visualizeClusters(imageClustered, &ballHelper,
						display_gray, minClusterSize);
				xvDisplay->displayFrameRGB((char*)imageClustered);
			}

			if(display_rgb || display_ir) {
				int c = 2*centerY * 2*FRAME_W + 2*centerX;
				paintPixel(imageRGB, c);
				paintPixel(imageRGB, c + 1);
				paintPixel(imageRGB, c + 2);
				paintPixel(imageRGB, c - 1);
				paintPixel(imageRGB, c - 2);
				paintPixel(imageRGB, c + 1 * 2*FRAME_W);
				paintPixel(imageRGB, c + 2 * 2*FRAME_W);
				paintPixel(imageRGB, c - 1 * 2*FRAME_W);
				paintPixel(imageRGB, c - 2 * 2*FRAME_W);
				xvDisplayRGB->displayFrameRGB((char*)imageRGB);
			} else if(display_yuv)
				xvDisplayRGB->displayFrameYUV((char*)imageRGB);

			if(display_depth) {
				visualizeDepth(imageDepth, depthCopy, m_gamma);
				xvDisplayDepth->displayFrameRGB((char*)imageDepth);
			}
		}
	}
	exit(0);
}

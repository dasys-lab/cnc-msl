#include <iostream>
#include <math.h>
#include <sstream>
#include <string.h>
#include <unistd.h>
#include <vector>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <stdio.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#define GNUPLOT_ENABLE_PTY
#include "helpers/KeyHelper.h"
#include "helpers/LocalizeDebug.h"
#include "helpers/SpicaHelper.h"
#include "helpers/gnuplot-iostream.h"
#include "ros/ros.h"

#include <opencv2/opencv.hpp>

#include <SystemConfig.h>

#include <signal.h>

using namespace std;
using namespace castor;

SpicaHelper *sh;

void int_handler(int sig)
{
    int mypid = getpid();
    printf("[%i] Ouch - shot in the ...\n", mypid);
    printf("[%i] exit\n", mypid);
    sh->sendVisionControl((int)'l', 0);
    sh->sendVisionControl((int)'o', 0);
    delete sh;
    ros::shutdown();
    exit(2);
}

int main(int argc, char *argv[])
{
    int mb = 1;
    sh = new SpicaHelper();
    sh->initialize("CNImageViewer", true);
    sh->receiverID = 0;

    if (argc > 1)
    {
        for (int i = 1; i < argc; i++)
        {
            if (std::string(argv[i]) == "-id")
            {
                if (i + 1 < argc)
                {
                    sh->receiverID = atoi(argv[i + 1]);
                    i++;
                }
            }
            if (std::string(argv[i]) == "-n")
            {
                if (i + 1 < argc)
                {
                    Configuration *globals = (*SystemConfig::getInstance())["Globals"];
                    sh->receiverID = globals->get<int>("Globals", "Team", argv[i + 1], "ID", NULL);
                    cout << "Robot: " << argv[i + 1] << " [" << sh->receiverID << "]" << endl;
                    i++;
                }
            }
        }
    }

    if (sh->receiverID == 0)
    {
        cout << "Robot ID: ";
        cin >> sh->receiverID;
    }

    int currentKey = EOF;
    char buf[1024 * 1024];

    cvNamedWindow("CameraImage", 1);
    void *windowHandle = cvGetWindowHandle("CameraImage");
    cvMoveWindow("CameraImage", 100, 100);

    signal(SIGINT, int_handler);
    sh->sendVisionControl((int)'L', 0);
    while (ros::ok())
    {
        if (sh->vidirty)
        {
            for (int i = 0; i < sh->imageData.size(); i++)
            {
                buf[i] = sh->imageData[i];
            }

            stringstream text;
            for (int i = 0; i < sh->params.size(); i++)
            {
                text << sh->params.at(i);
                text << " ";
            }
            if (sh->params.size() > 0)
                cout << "Min / Max: " << text.str() << endl;

            cv::Mat m(sh->width, sh->height, CV_8UC1, buf);
            IplImage img = imdecode(m, 0);
            cvShowImage(cvGetWindowName(windowHandle), &img);
            // cv::displayStatusBar(cvGetWindowName(windowHandle), text.c_str(), 4000);

            sh->vidirty = false;
        }
        // currentKey = KeyHelper::checkKeyPress();
        currentKey = cvWaitKey(10);
        // if(currentKey!=EOF && currentKey > 0) {
        if (currentKey > 0)
        {
            sh->sendVisionControl(currentKey, 0);
        }
    }
    sh->sendVisionControl((int)'o', 0);
    sh->sendVisionControl((int)'l', 0);
}

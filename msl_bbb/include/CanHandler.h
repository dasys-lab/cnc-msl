#pragma once

#include "ROSMsgFuncs.h"

#include <Can.h>

#include <ros/ros.h>
#include <usbcanconnection.h>

#include <signal.h>
#include <stdio.h>
#include <sys/time.h>
#include <vector>

namespace msl_actuator_msgs{
  ROS_DECLARE_MESSAGE(CanMsg)
}

namespace msl_bbb
{
class CanHandler : public CanListener
{

  public:
    CanHandler();
    virtual ~CanHandler();

    void stop();
    void sendCanMsg(const msl_actuator_msgs::CanMsg &msg);

    UsbCanConnection *usbCanConnection;

  protected:
    // while reading from usb -> publisher
    // ros::AsyncSpinner *spinner;
    // ros::Time lastMotion;

    // publisher for direct bundle restart trigger

    std::vector<unsigned short> receivers;

    void resetInterface();
    void receive(unsigned int canid, unsigned char *data, int len);
    void receive();
};
}

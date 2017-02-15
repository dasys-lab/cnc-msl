#pragma once

#include "CanHandler.h"

#include <ros/ros.h>

#include <arpa/inet.h>
#include <boost/asio.hpp>
#include <net/if.h>
#include <netinet/in.h>
#include <sys/ioctl.h>

// Forward declarations of ROS messages

namespace msl_actuator_msgs
{
ROS_DECLARE_MESSAGE(BallHandleCmd);
ROS_DECLARE_MESSAGE(BallHandleMode);
ROS_DECLARE_MESSAGE(ShovelSelectCmd);
ROS_DECLARE_MESSAGE(MotionLight);
ROS_DECLARE_MESSAGE(VisionRelocTrigger);
ROS_DECLARE_MESSAGE(MotionBurst);
ROS_DECLARE_MESSAGE(RawOdometryInfo);
ROS_DECLARE_MESSAGE(CanMsg);
ROS_DECLARE_MESSAGE(IMUData);
}

namespace std_msgs
{
ROS_DECLARE_MESSAGE(Bool);
}

namespace process_manager
{
ROS_DECLARE_MESSAGE(ProcessCommand);
}

namespace boost
{
class thread;
}

namespace msl_bbb
{

using boost::asio::ip::udp;

class BallHandler;
class OpticalFlow;
class ShovelSelection;
class Communication
{
  public:
    Communication();
    virtual ~Communication();

    void run_udp();
    void handleCanSub(const msl_actuator_msgs::CanMsg &msg);

    // methods for sending ROS msgs per udp multicast
    void onRosBallHandleCmd1334345447(msl_actuator_msgs::BallHandleCmd &message);
    void onRosBallHandleMode297244167(msl_actuator_msgs::BallHandleMode &message);
    void onRosShovelSelectCmd1418208429(msl_actuator_msgs::ShovelSelectCmd &message);
    void onRosMotionLight2056271736(msl_actuator_msgs::MotionLight &message);
    void onRosProcessCommand554624761(process_manager::ProcessCommand &message);
    void onRosVisionRelocTrigger2772566283(msl_actuator_msgs::VisionRelocTrigger &message);
    void onRosMotionBurst1028144660(msl_actuator_msgs::MotionBurst &message);
    void onRosBool2802967882(std_msgs::Bool &message);
    void onRosRawOdometryInfo3134514216(msl_actuator_msgs::RawOdometryInfo &message);
    void onRosCanMsg1267609526(msl_actuator_msgs::CanMsg &message);
    void onRosCanMsg217678336(msl_actuator_msgs::CanMsg &message);
    void onRosCanMsg418700403(msl_actuator_msgs::CanMsg &message);
    void onRosCanMsg3391245383(msl_actuator_msgs::CanMsg &message);
    void onRosIMUData3455796956(msl_actuator_msgs::IMUData &message);

    void listenForPacket();
    void handleUdpPacket(const boost::system::error_code &error, std::size_t bytes_transferred);

    void setActuators(BallHandler *ballHandler, OpticalFlow *opticalFlow, ShovelSelection *shovelSelection);

    boost::array<char, 64000> inBuffer;
    std::string ownRosName;

    udp::socket *insocket;
    udp::endpoint otherEndPoint;
    udp::endpoint destEndPoint;
    boost::asio::ip::address multiCastAddress;
    boost::asio::io_service io_service;
    boost::thread *iothread;

    // Can hack
    CanHandler canHandler;

    BallHandler *ballHandler;
    OpticalFlow *opticalFlow;
    ShovelSelection *shovelSelection;
};

} /* namespace msl_bbb */

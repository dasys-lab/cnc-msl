#ifndef DomainBehaviour_H_
#define DomainBehaviour_H_

#include "engine/BasicBehaviour.h"
#include "ros/ros.h"

namespace msl_helper_msgs{
	ROS_DECLARE_MESSAGE(WatchBallMsg)
	ROS_DECLARE_MESSAGE(PassMsg)
	ROS_DECLARE_MESSAGE(DebugMsg)
}

namespace msl_actuator_msgs{
	ROS_DECLARE_MESSAGE(MotionControl)
	ROS_DECLARE_MESSAGE(BallHandleCmd)
	ROS_DECLARE_MESSAGE(KickControl)
	ROS_DECLARE_MESSAGE(ShovelSelectCmd)
}

namespace supplementary {
	class SystemConfig;
}

namespace msl{
	class MSLRobot;
	class MSLWorldModel;
	class Logger;
}

namespace alica
{
class DomainBehaviour : public BasicBehaviour
{
	public:
		DomainBehaviour(string name);
		virtual ~DomainBehaviour();
		void send(msl_actuator_msgs::MotionControl& mc);
		void send(msl_actuator_msgs::BallHandleCmd& bh);
		void send(msl_actuator_msgs::KickControl& kc);
		void send(msl_actuator_msgs::ShovelSelectCmd& ssc);
		void send(msl_helper_msgs::PassMsg& pm);
		void send(msl_helper_msgs::PassMsg& pm, int senderID);
		void send(msl_helper_msgs::WatchBallMsg& wb);
		void send(msl_helper_msgs::DebugMsg& dbm);
		msl::MSLRobot* robot;
		msl::MSLWorldModel* wm;
		msl::Logger* logger;

	protected:
		supplementary::SystemConfig* sc;

	private:

		double __maxTranslation;
		int ownID;
		ros::Publisher simlatorPub;
		ros::Publisher motionControlPub;
		ros::Publisher ballHandlePub;
		ros::Publisher kickControlPub;
		ros::Publisher shovelSelectPublisher;
		ros::Publisher passMsgPublisher;
		ros::Publisher watchBallMsgPublisher;
		ros::Publisher debugMsgPublisher;
	};
} /* namespace alica */

#endif /* DomainBehaviour_H_ */


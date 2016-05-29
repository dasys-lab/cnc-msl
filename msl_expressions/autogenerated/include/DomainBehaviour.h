#ifndef DomainBehaviour_H_
#define DomainBehaviour_H_

#include "engine/BasicBehaviour.h"
#include "ros/ros.h"
#include "msl_actuator_msgs/MotionControl.h"
#include "msl_actuator_msgs/BallHandleCmd.h"
#include "msl_actuator_msgs/KickControl.h"
#include "msl_actuator_msgs/ShovelSelectCmd.h"
#include "msl_helper_msgs/PassMsg.h"
#include "msl_helper_msgs/WatchBallMsg.h"
#include "msl_helper_msgs/DebugMsg.h"

namespace msl{
	class MSLWorldModel;
}

namespace supplementary{
	class SystemConfig;
}
namespace geometry{
	class CNPosition;
}

namespace alica
{
	class EntryPoint;
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
		shared_ptr<geometry::CNPosition> getTeammatesPosition(EntryPoint* ep);

	protected:
		msl::MSLWorldModel* wm;
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


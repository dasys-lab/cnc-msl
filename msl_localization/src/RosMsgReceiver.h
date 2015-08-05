#ifndef RosMsgReceiver_h
#define RosMsgReceiver_h


#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseArray.h"


class RosMsgReceiver {
	public:
		void initialize();
		static RosMsgReceiver* getInstance();
		
		void sendParticleCloud(geometry_msgs::PoseArray &p);

		nav_msgs::MapMetaData* getMapInfo();
		unsigned char* getMap();
		sensor_msgs::LaserScan::ConstPtr getScan() {return msgptr;}
		geometry_msgs::PoseWithCovarianceStamped::ConstPtr getPose() {return poseptr;}
		
		ros::Time getObservationTime() {return observTime;}
		bool mapReceived() {return mpReceived;};
		bool scanReceived() {return scnReceived;};
		bool poseReceived() {return pseReceived;};
		void poseProcessed() {pseReceived=false;};
		bool dirty;
		
	protected:
		void handleMapMessage(const nav_msgs::OccupancyGrid::ConstPtr& message);
		void handleScanMessage(const sensor_msgs::LaserScan::ConstPtr& scan);
		void handlePoseMessage(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose);
		static RosMsgReceiver* instance;
		ros::Subscriber Mapsub;
		ros::Subscriber LaserSub;
		ros::Subscriber Iniposesub;
		
		ros::Publisher particlepub;
		
		ros::Time observTime;
		
		nav_msgs::MapMetaData mapInfo;
		ros::AsyncSpinner *spinner;
		unsigned char* map;
		bool mpReceived, scnReceived, pseReceived;
		sensor_msgs::LaserScan::ConstPtr msgptr;
		geometry_msgs::PoseWithCovarianceStamped::ConstPtr poseptr;
};

#endif

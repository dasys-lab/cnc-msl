#ifndef RosMsgReceiver_h
#define RosMsgReceiver_h


#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "msl_sensor_msgs/LinePointList.h"
#include "msl_actuator_msgs/RawOdometryInfo.h"
#include "msl_actuator_msgs/IMUData.h"
#include "msl_sensor_msgs/CorrectedOdometryInfo.h"
#include "msl_actuator_msgs/VisionRelocTrigger.h"

#include "SystemConfig.h"

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
		msl_sensor_msgs::LinePointListPtr getCurrentLinePointList();
		msl_actuator_msgs::RawOdometryInfoPtr getOdometryInfo();
		msl_actuator_msgs::IMUDataPtr getIMUData();
		bool mapReceived() {return mpReceived;};
		bool scanReceived() {return scnReceived;};
		bool poseReceived() {return pseReceived;};
		void poseProcessed() {pseReceived=false;};
		bool dirty;
		bool reloc;
		
		ros::Publisher coipub;

	protected:
		void handleMapMessage(const nav_msgs::OccupancyGrid::ConstPtr& message);
		void handleScanMessage(const sensor_msgs::LaserScan::ConstPtr& scan);
		void handlePoseMessage(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose);
		void handleOdometryInfoMessage(msl_actuator_msgs::RawOdometryInfoPtr msg);
		void handleLinePointListMessage(msl_sensor_msgs::LinePointListPtr msg);
		void handleIMUData(msl_actuator_msgs::IMUDataPtr msg);
		void handleVisionRelocTriggerMessage(const msl_actuator_msgs::VisionRelocTrigger::ConstPtr& msg);

		static RosMsgReceiver* instance;
		ros::Subscriber Mapsub;
		ros::Subscriber LaserSub;
		ros::Subscriber Iniposesub;
		ros::Subscriber OdometrySub;
		ros::Subscriber RelocSub;
		ros::Subscriber LinePointListSub;
		ros::Subscriber imuDataSub;

		ros::Publisher particlepub;

		ros::Time observTime;
		
		nav_msgs::MapMetaData mapInfo;
		ros::AsyncSpinner *spinner;
		unsigned char* map;
		bool mpReceived, scnReceived, pseReceived;
		sensor_msgs::LaserScan::ConstPtr msgptr;
		geometry_msgs::PoseWithCovarianceStamped::ConstPtr poseptr;
		msl_actuator_msgs::RawOdometryInfoPtr odometryInfoMsg;
		msl_sensor_msgs::LinePointListPtr currentLinePoints;
		msl_actuator_msgs::IMUDataPtr imuData;


};

#endif

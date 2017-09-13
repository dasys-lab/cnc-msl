/*
 * RobotMovement.h
 *
 *  Created on: 17.12.2014
 *      Author: tobi
 */

#ifndef CNC_MSL_MSL_WORLDMODEL_SRC_ROBOTMOVEMENT_ROBOTMOVEMENT_H_
#define CNC_MSL_MSL_WORLDMODEL_SRC_ROBOTMOVEMENT_ROBOTMOVEMENT_H_

#include "DateTime.h"
#include "SystemConfig.h"
#include "msl_actuator_msgs/MotionControl.h"
#include <memory>

namespace geometry
{
class CNPoint2D;
class CNPosition;
}

using namespace std;

namespace msl
{

class MovementQuery;
class SearchArea;
class MSLWorldModel;
class PathProxy;
/**
 * Provides Methods to steer the robot while avoiding obstacles
 */
class RobotMovement
{
  public:
    RobotMovement();
    virtual ~RobotMovement();

    // TODO query was const before check
    /**
     * Move to a given point regarding the information given
     * @param m_Query shared_ptr<MovementQuery> encapsulated information needed to move to the given point
     * @return msl_actuator_msgs::MotionControl msg containing the motion information, NaN if anything went wrong
     */
    msl_actuator_msgs::MotionControl moveToPoint(shared_ptr<MovementQuery> m_Query);
    /**
     * Rotate towards given point
     * @param m_Query shared_ptr<MovementQuery> encapsulated information needed to rotate to the given point
     * @return msl_actuator_msgs::MotionControl msg containing the motion information, NaN if anything went wrong
     */
    msl_actuator_msgs::MotionControl alignTo(shared_ptr<MovementQuery> m_Query);
    msl_actuator_msgs::MotionControl experimentallyAlignTo(shared_ptr<MovementQuery> m_Query);
    /**
     * Check if the robot is violation any rule and react respectively
     * @param m_Query shared_ptr<MovementQuery> encapsulated information needed to move to the given point
     * @return msl_actuator_msgs::MotionControl msg containing the motion information, NaN if no rule is violated
     */
    msl_actuator_msgs::MotionControl ruleActionForBallGetter();
    /**
     * Drive randomly when not localized
     * @param translation double speed to use
     * @return msl_actuator_msgs::MotionControl msg containing the motion information, NaN if anything went wrong
     */
    msl_actuator_msgs::MotionControl driveRandomly(double translation);
    /**
     * Move to free space to accept a pass
     * @param m_Query shared_ptr<MovementQuery> encapsulated information needed to move to the given point
     * @return msl_actuator_msgs::MotionControl msg containing the motion information, NaN if anything went wrong
     */
    msl_actuator_msgs::MotionControl moveToFreeSpace(shared_ptr<MovementQuery> m_Query);

    /**
     * Read parameters from Config (Drive.conf)
     */
    void readConfigParameters();
    /**
     * Default normal translation
     */
    double defaultTranslation;
    /**
     * Default fast translation speed
     */
    double fastTranslation;
    /**
     * Default carefully translation speed
     */
    double carefullyTranslation;
    /**
     * Default rotation P controller value
     */
    double defaultRotation;
    /**
     * Default fast rotation speed
     */
    double fastRotation;
    /**
     * Default carefully rotation speed
     */
    double carefullyRotation;

  private:
    static int randomCounter;
    static int beamSize;
    static shared_ptr<vector<shared_ptr<SearchArea>>> fringe;
    static shared_ptr<vector<shared_ptr<SearchArea>>> next;
    static shared_ptr<geometry::CNPoint2D> randomTarget;

    MSLWorldModel *wm;
    PathProxy *pp;
    msl_actuator_msgs::MotionControl placeRobot(shared_ptr<geometry::CNPoint2D> dest, shared_ptr<geometry::CNPoint2D> headingPoint);
    double evalPointDynamic(shared_ptr<geometry::CNPoint2D> alloP, shared_ptr<geometry::CNPoint2D> alloPassee, shared_ptr<geometry::CNPosition> ownPos,
                            shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> opponents);
    msl_actuator_msgs::MotionControl setNAN();

    // for alignTO()
    double rotationP;
    double rotationD;
    double transP;
    double transI;

  protected:
    static double assume_enemy_velo;
    static double assume_ball_velo;
    static double interceptQuotient;
    static double robotRadius;
};
}

#endif /* CNC_MSL_MSL_WORLDMODEL_SRC_ROBOTMOVEMENT_ROBOTMOVEMENT_H_ */

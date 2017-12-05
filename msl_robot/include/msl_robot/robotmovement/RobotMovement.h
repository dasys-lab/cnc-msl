#pragma once

#include "DateTime.h"
#include "SystemConfig.h"
#include "msl_actuator_msgs/MotionControl.h"

#include <cnc_geometry/CNPointAllo.h>
#include <cnc_geometry/CNPointEgo.h>
#include <nonstd/optional.hpp>

#include <queue>
#include <valarray>
#include <memory>

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
     * @param query std::shared_ptr<MovementQuery> encapsulated information needed to move to the given point
     * @return msl_actuator_msgs::MotionControl msg containing the motion information, NaN if anything went wrong
     */
    msl_actuator_msgs::MotionControl moveToPoint(MovementQuery &query);
    /**
     * Rotate towards given point
     * @param query std::shared_ptr<MovementQuery> encapsulated information needed to rotate to the given point
     * @return msl_actuator_msgs::MotionControl msg containing the motion information, NaN if anything went wrong
     */
    msl_actuator_msgs::MotionControl alignTo(MovementQuery &query);
    /**
     * Check if the robot is violation any rule and react respectively
     * @param m_Query std::shared_ptr<MovementQuery> encapsulated information needed to move to the given point
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
     * @param query std::shared_ptr<MovementQuery> encapsulated information needed to move to the given point
     * @return msl_actuator_msgs::MotionControl msg containing the motion information, NaN if anything went wrong
     */
    msl_actuator_msgs::MotionControl moveToFreeSpace(MovementQuery &query);

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
    static std::shared_ptr<vector<std::shared_ptr<SearchArea>>> fringe;
    static std::shared_ptr<vector<std::shared_ptr<SearchArea>>> next;
    static nonstd::optional<geometry::CNPointEgo> randomTarget;

    MSLWorldModel *wm;
    PathProxy *pp;
    msl_actuator_msgs::MotionControl placeRobot(geometry::CNPointEgo dest,
                                                nonstd::optional<geometry::CNPointEgo> headingPoint = nonstd::nullopt);
    double evalPointDynamic(geometry::CNPointAllo alloP, geometry::CNPointAllo alloPassee,
                            geometry::CNPositionAllo ownPos, const std::vector<geometry::CNPointAllo> &opponents);
    msl_actuator_msgs::MotionControl setNAN();

  protected:
    static double assume_enemy_velo;
    static double assume_ball_velo;
    static double interceptQuotient;
    static double robotRadius;
    static double asymptoticGain;

    //PT STUFF

    /**
     * PT-Controller for smooth translation acceleration
     */
    std::valarray<double> ptController(MovementQuery &query, double translation, double rotation);
    /**
     * Initialize all needed parameters and queues for the PT-Controller
     */
    void initializePTControllerParameters();

    void clearPTControllerQueues();

    void stopTranslation();

    /**
     * Past sent translation for PT-Controller
     */
    std::queue<std::valarray<double>> pastControlledValues;

    /**
     * Past translation input for PT-Controller
     */
    std::queue<std::valarray<double>> pastControlInput;

    /**
     * Carefully value for PT-Controller (Drive.conf)
     */
    double carefullyControllerVelocity;
    /**
     * Default value for PT-Controller (Drive.conf)
     */
    double defaultControllerVelocity;
    /**
     * fast value for PT-Controller (Drive.conf)
     */
    double fastControllerVelocity;
    /**
     * Behaviour frequency
     */
    double sampleTime = 1.0 / 30.0;
};

} /* namespace msl */

/*
 * MSLRobot.h
 *
 *  Created on: Jun 12, 2016
 *      Author: Stephan Opfer
 */

#ifndef SRC_MSLROBOT_H_
#define SRC_MSLROBOT_H_


namespace msl
{
	class MSLWorldModel;
	class RobotMovement;
	class Kicker;
	class MSLRobot
	{
	public:
		static MSLRobot* get();

		MSLWorldModel* wm;
		RobotMovement* robotMovement;
		Kicker* kicker;

	private:
		MSLRobot();
		virtual ~MSLRobot();
	};

} /* namespace msl */

#endif /* SRC_MSLROBOT_H_ */

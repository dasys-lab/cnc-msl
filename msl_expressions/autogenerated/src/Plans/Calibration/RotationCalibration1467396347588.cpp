#include "Plans/Calibration/RotationCalibration1467396347588.h"
using namespace alica;
/*PROTECTED REGION ID(eph1467396347588) ENABLED START*/ //Add additional using directives here
/*PROTECTED REGION END*/
namespace alicaAutogenerated
{
	//Plan:RotationCalibration

	/* generated comment

	 Task: DefaultTask  -> EntryPoint-ID: 1467396347590

	 */
	shared_ptr<UtilityFunction> UtilityFunction1467396347588::getUtilityFunction(Plan* plan)
	{
		/*PROTECTED REGION ID(1467396347588) ENABLED START*/

		shared_ptr<UtilityFunction> defaultFunction = make_shared<DefaultUtilityFunction>(plan);
		return defaultFunction;

		/*PROTECTED REGION END*/
	}

	//State: Stop in Plan: RotationCalibration

	/*
	 *
	 * Transition:
	 *   - Name: MISSING_NAME, ConditionString: , Comment : Status Start?
	 *
	 * Plans in State:
	 *
	 * Tasks:
	 *   - DefaultTask (1225112227903) (Entrypoint: 1467396347590)
	 *
	 * States:
	 *   - Stop (1467396347589)
	 *   - Rotating (1467396438734)
	 *
	 * Vars:
	 */
	bool TransitionCondition1467396619848::evaluate(shared_ptr<RunningPlan> rp)
	{
		/*PROTECTED REGION ID(1467396616225) ENABLED START*/
		return wm->game->checkSituation(msl::Situation::Start);
		/*PROTECTED REGION END*/

	}

	//State: Rotating in Plan: RotationCalibration

	/*
	 *
	 * Transition:
	 *   - Name: MISSING_NAME, ConditionString: , Comment : Rotation done? Or Status Stop?
	 *
	 * Plans in State:
	 *
	 * Tasks:
	 *   - DefaultTask (1225112227903) (Entrypoint: 1467396347590)
	 *
	 * States:
	 *   - Stop (1467396347589)
	 *   - Rotating (1467396438734)
	 *
	 * Vars:
	 */
	bool TransitionCondition1467396709878::evaluate(shared_ptr<RunningPlan> rp)
	{
		/*PROTECTED REGION ID(1467396705635) ENABLED START*/
		if (wm->game->checkSituation(msl::Situation::Stop) == true)
			return true;

		return false;
		/*PROTECTED REGION END*/

	}

}

/*
 * Base.h
 *
 *  Created on: 22.10.2014
 *      Author: endy
 */

#ifndef BASE_H_
#define BASE_H_

#include <engine/AlicaEngine.h>

namespace msl
{

	class Base
	{
	public:
		Base();
		virtual ~Base();

		alica::AlicaEngine* ae;
		alicaTests::TestBehaviourCreator* bc;
		alicaTests::TestConditionCreator* cc;
		alicaTests::TestUtilityFunctionCreator* uc;
		alicaTests::TestConstraintCreator* crc;

	protected:
		//blabla...
	};

} /* namespace msl */

#endif /* BASE_H_ */

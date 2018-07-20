/*
 * PriorityList.h
 *
 *  Created on: Oct 27, 2014
 *      Author: Tobias Schellien
 */

#ifndef ALICA_ALICA_TEST_INCLUDE_PRIORITYLIST_H_
#define ALICA_ALICA_TEST_INCLUDE_PRIORITYLIST_H_

#include <engine/USummand.h>
#include <container/CNPoint2D.h>
#include <vector>
#include <string>
#include <engine/IAssignment.h>

using namespace std;

namespace alica
{

	class UtilityInterval;
	class IAssignment;

	class PriorityList : public USummand
	{
	public:
		PriorityList(double weight, string name, long id, vector<long> relevantEntryPointIds);
		virtual ~PriorityList();

		virtual UtilityInterval eval(IAssignment* ass);
		string toString();


	protected:
		shared_ptr<vector<double>> w;
		shared_ptr<vector<int>> c;
		double norm;
	};

} /* namespace alica */

#endif /* ALICA_ALICA_TEST_INCLUDE_PRIORITYLIST_H_ */

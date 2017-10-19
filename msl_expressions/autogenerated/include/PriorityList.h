#pragma once
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


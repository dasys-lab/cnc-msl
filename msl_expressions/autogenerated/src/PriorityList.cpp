/*
 * PriorityList.cpp
 *
 *  Created on: Oct 27, 2014
 *      Author: Tobias Schellien
 */

#include <PriorityList.h>
#include "MSLWorldModel.h"

namespace alica
{

	PriorityList::PriorityList(double weight, string name, long id, vector<long> relevantEntryPointIds)
	{
		this->weight = weight;
		this->name = name;
		this->id = id;
		this->relevantEntryPointIds = relevantEntryPointIds;

		this->w = make_shared<vector<double>>();
		this->c = make_shared<vector<int>>();
		double j = 1;
		for (int i=0; i< this->relevantEntryPointIds.size(); i++)
		{
			this->w->push_back(1/j);
			j*=2.0;
			//j++;
			this->norm+= this->w->at(i);
			this->c->push_back(0);
			for(int k=0; k<i; k++) {
				if(this->relevantEntryPointIds.at(i) == this->relevantEntryPointIds.at(k)) {
					this->c->at(i)++;
				}
			}

		}
	}

	PriorityList::~PriorityList()
	{
	}

	UtilityInterval PriorityList::eval(IAssignment* ass)
	{
		int unassignedRobots = ass->getNumUnAssignedRobotIds();
		UtilityInterval ui(0.0, 0.0);

		for (int i = 0; i< this->relevantEntryPoints.size(); i++)
		{
			shared_ptr<list<int> > robots = ass->getUniqueRobotsWorkingAndFinished(relevantEntryPoints.at(i));
			if(robots->size() > c->at(i))
			{
				ui.setMin(ui.getMin() + this->w->at(i));
			}
			else
			{
				if(unassignedRobots > 0)
				{
					ui.setMax(ui.getMax() + this->w->at(i));
					unassignedRobots--;
				}
			}
		}

		ui.setMax(ui.getMax() + ui.getMin());

		ui.setMax(ui.getMax()/this->norm);
		ui.setMin(ui.getMin()/this->norm);

		return ui;
	}

	string PriorityList::toString ()
	{
		string retString = this->name + ": ";
		retString += string("W: ") + to_string(this->weight) + string(" EntryPoints: ");
		for(long epId : this->relevantEntryPointIds)
		{
			retString += to_string(epId) + string(" ");
		}
		return retString;
	}


} /* namespace alica */

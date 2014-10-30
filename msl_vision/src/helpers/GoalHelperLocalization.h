/*
 * $Id: GoalHelperLocalization.h 1935 2007-03-19 19:50:12Z phbaer $
 *
 *
 * Copyright 2005,2006 Carpe Noctem, Distributed Systems Group,
 * University of Kassel. All right reserved.
 *
 * The code is derived from the software contributed to Carpe Noctem by
 * the Carpe Noctem Team.
 *
 * The code is licensed under the Carpe Noctem Userfriendly BSD-Based
 * License (CNUBBL). Redistribution and use in source and binary forms,
 * with or without modification, are permitted provided that the
 * conditions of the CNUBBL are met.
 *
 * You should have received a copy of the CNUBBL along with this
 * software. The license is also available on our website:
 * http://carpenoctem.das-lab.net/license.txt
 *
 *
 * <description>
 */
#ifndef GoalHelperLocalization_H
#define GoalHelperLocalization_H

//#include <libAnja/DatagramSocket.h>
//#include <libAnja/UnixSocket.h>

#include "../global/Types.h"
#include <vector>

class GoalHelperLocalization{


	public:
		GoalHelperLocalization();
		~GoalHelperLocalization();
	
		void getGoalsFromPosition(Position & position);

	protected:
		
		void init();
		void cleanup();

		Point getAlloPoint(Point point, Position position);

		unsigned short msgid;

};



#endif


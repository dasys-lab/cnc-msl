/*
 * $Id: LineDistanceHelper.h 1874 2007-03-02 20:35:47Z rreichle $
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
#ifndef LineDistanceHelper_H
#define LineDistanceHelper_H

#define IWIDTH 750
#define IHEIGHT 750
#define RESOLUTION 30.0
#define undefined 100000.0
#define MAX_LDIST 255

class LineDistanceHelper{

	public:
		LineDistanceHelper();
		~LineDistanceHelper();

		unsigned char getLineDistance(double px, double py);
		unsigned char * getLineLookup();

	private:

		unsigned char * LineLookup;
		void init();
		void cleanup();



};


#endif


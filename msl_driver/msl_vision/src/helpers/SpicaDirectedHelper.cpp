/*
 * $Id: Filter.cpp 1804 2007-01-07 21:53:55Z saur $
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
#include "SpicaDirectedHelper.h"

// CarpeNoctem::Net::CEP::Vision SpicaDirectedHelper::visionCEP;
ros::NodeHandle* SpicaDirectedHelper::visionDirectedNode;

SpicaDirectedHelper::SpicaDirectedHelper() {
	initialize();
}

void SpicaDirectedHelper::initialize() {
	visionDirectedNode = new ros::NodeHandle();
}


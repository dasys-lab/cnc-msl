#ifndef KEYHELPER_H
#define KEYHELPER_H

#include "SpicaHelper.h"


class KeyHelper {
	
public:
	static void checkKeyPress();
	static void checkRemoteKey();
	static bool checkKey(unsigned char k);

	//static unsigned char key;

};


#endif

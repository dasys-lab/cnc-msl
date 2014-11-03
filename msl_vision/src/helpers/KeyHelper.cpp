#include "KeyHelper.h"
#include <termios.h>
#include <stdio.h>



int key=EOF;
void KeyHelper::checkKeyPress()
{
        struct termios initial_settings, new_settings;

        tcgetattr(0,&initial_settings);

        new_settings = initial_settings;
        new_settings.c_lflag &= ~ICANON;
        new_settings.c_lflag &= ~ECHO;
        //new_settings.c_lflag &= ~ISIG;
        new_settings.c_cc[VMIN] = 0;
        new_settings.c_cc[VTIME] = 0;

        tcsetattr(0, TCSANOW, &new_settings);
        int n = getchar();
	tcsetattr(0, TCSANOW, &initial_settings);
	key = n;
	if(SpicaHelper::key != EOF) {
		key = SpicaHelper::key;
		key = EOF;
	}
}

void KeyHelper::checkRemoteKey() {
        if(SpicaHelper::key != EOF) {
                key = SpicaHelper::key;
                SpicaHelper::key = EOF;
        }
}


bool KeyHelper::checkKey(unsigned char k) {

	if(key != EOF)
	{
		if(key==k) {
			return true;
		}
    	}
	return false;
}

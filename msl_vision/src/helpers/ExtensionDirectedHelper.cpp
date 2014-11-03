#include "ExtensionDirectedHelper.h"
#include <msl_actuator_msgs/KickControl.h>
#include "SpicaDirectedHelper.h"
#include "SpicaHelper.h"

#define BUF_SIZE 500

using namespace msl_actuator_msgs;

//SpicaDirectedHelper ExtensionDirectedHelper::helper;
//CommunicationPtr ExtensionDirectedHelper::comm(Communication::getInstance());
//VisionDirectedPtr ExtensionDirectedHelper::visionDirectedCEP;

ros::Publisher ExtensionDirectedHelper::kcpub;


void ExtensionDirectedHelper::initialize() {
	 kcpub = SpicaHelper::visionNode->advertise<KickControl>("KickControl", 1);
}

//Not Testet
int ExtensionDirectedHelper::extendUpperExtension()
{
	KickControl kick;
	kick.extension = (1);
	kick.extTime = (1000);
        kcpub.publish(kick);
}

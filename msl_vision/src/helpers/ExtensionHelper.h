#ifndef EXTENSIONHELPER_H
#define EXTENSIONHELPER_H

#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/time.h>
//#include <ExtendExtension.h>
#include "ros/ros.h"

class ExtensionHelper {
  public:
    static void initialize();
    static int extendUpperExtension();
    static ros::Publisher kcpub;
};

#endif /* EXTENSIONHELPER_H */

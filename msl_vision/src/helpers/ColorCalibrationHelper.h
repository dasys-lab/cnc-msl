#ifndef ColorCalibrationHelper_h
#define ColorCalibrationHelper_h

#include <queue>

const int CC_ROBOT_PORT =       62303;

namespace ColorCalibration {
    class ColorCalibrationHelper {
    public:
        static void initialize();

        static void *listenerThread(void *threadid);

        static void sendImage(unsigned char* img, int width, int height);

        static std::queue<int> connections;
    };
}

#endif

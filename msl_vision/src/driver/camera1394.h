#ifndef CAMERA_CAMERA1394_H
#define CAMERA_CAMERA1394_H

#include "camera.h"

namespace camera
//namespace castor
{

    class Camera1394 : public Camera
    {
        public:
            Camera1394() : video_mode() {}

            /**
             * Set the video mode
             * @param format the dc1394 video format and mode. Throws an exception
             * if the given configuration is not supported
             */
            void setVideoMode(dc1394video_mode_t mode) { this->video_mode = mode; }

        protected:
            dc1394video_mode_t video_mode;
    };

}

#endif /* CAMERA_CAMERA1394_H */


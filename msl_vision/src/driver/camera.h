#ifndef CAMERA_CAMERA_H
#define CAMERA_CAMERA_H 1

#include "frame.h"
#include <stdexcept>

namespace camera
//namespace castor
{

    /**
     * A simple stub for a exception class used by the camera
     * subsystem
     */
    class CameraException : public std::runtime_error
    {
        public:
            CameraException(const std::string r) : std::runtime_error(r) {}
    };

    struct camera_roi_t
    {
        uint16_t x;
        uint16_t y;
        uint16_t w;
        uint16_t h;
    };

    /**
     * An generic interface for camera drivers
     */
    class Camera
    {
        public:
            Camera() : initialised(false), running(false), last_error() {}
            virtual ~Camera() { reset(); }

            /**
             * Set the region of interest (for format7 only)
             * @param roi a structure the defined x and y offsets and the width and height
             * of the region
             */
            virtual void setROI(const camera_roi_t &r)
                throw(CameraException)
            {
                throw(CameraException("setting ROI not supported"));
            }

            std::string getLastError() const { return this->last_error; }

        protected:

            /**
             * Initialise the camera using the previsously specified video mode
             * and other settings. This method most be implemented.
             */
            virtual void initInternal() throw(CameraException) = 0;

            /**
             * Resets additional parameters/values of the camera. May be overwritten.
             */
            virtual void resetInternal() throw(CameraException) {}

            virtual void startCaptureInternal() throw(CameraException) = 0;
            virtual void stopCaptureInternal() throw(CameraException) = 0;

            virtual bool getFrameInternal(Frame &frame) = 0;

        public:

            inline void reset()
                throw(CameraException)
            {
                if (this->running) stopCapture();

                this->running = false;
                this->initialised = false;

                this->last_error = "";

                resetInternal();
            }

            inline void init()
                throw(CameraException)
            {
                initInternal();

                this->initialised = true;
            }

#define ASSURE(cond, message)                    \
            if (!(cond))                         \
            {                                    \
                this->last_error = message;      \
                return false;                    \
            }
#define ASSURE_THROW(cond, message)              \
            if (!(cond))                         \
            {                                    \
                this->last_error = message;      \
                throw(CameraException(message)); \
            }


            inline void startCapture()
                throw(CameraException)
            {
                ASSURE_THROW(this->initialised, "camera not initialised");
                ASSURE_THROW(!this->running, "image capture already started");

                startCaptureInternal();

                this->running = true;
            }

            inline void stopCapture()
                throw(CameraException)
            {
                ASSURE_THROW(this->initialised, "camera not initialised");
                ASSURE_THROW(this->running, "image capture not started");

                stopCaptureInternal();

                this->running = false;
            }

            inline bool getFrame(Frame &frame)
            {
                ASSURE(this->initialised, "camera not initialised");
                ASSURE(this->running, "image capture not started");

                return getFrameInternal(frame);
            }

        private:

            bool initialised;
            bool running;

            std::string last_error;
    };

}

#endif /* CAMERA_CAMERA_h */


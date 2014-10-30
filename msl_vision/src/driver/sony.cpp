#include "sony.h"
#include <string.h>
#include <arpa/inet.h>

#include <libraw1394/raw1394.h>

namespace camera
//namespace castor
{

    Sony::Sony(const char* vendor) :
        index(0),   data_depth(8),
        dc_camera(NULL), dc_device(NULL), dc_frame(NULL)
    {
        //awb_enabled(true), roi(),
        sonyInitInternal(vendor);
    }

    void Sony::sonyInitInternal(const char* vendor)
    {
        this->dc_device = dc1394_new();

        dc1394camera_list_t *list;
        dc1394error_t err;

        // get list of available cameras

        err = dc1394_camera_enumerate (this->dc_device, &list);
        DC1394_ERR_THROW(err, "Failed to enumerate cameras");

        if (list->num == 0)
        {
            DC1394_ERR_THROW(DC1394_FAILURE, "no cameras found");
        }

        if (list->num < index)
        {
            DC1394_ERR_THROW(DC1394_FAILURE, "camera not found, index too high");
        }

    for(unsigned int i=0; i<list->num; i++) {
		this->dc_camera = dc1394_camera_new(this->dc_device, list->ids[i].guid);
		if(strstr(vendor, this->dc_camera->vendor)!=NULL) {
			break;
		} else {
			dc1394_camera_free(this->dc_camera);
		}
	}

        // initialize new camera structure
        //this->dc_camera = dc1394_camera_new(this->dc_device, list->ids[camInd].guid);
        if (!this->dc_camera)
        {
            DC1394_ERR_THROW(DC1394_CAMERA_NOT_INITIALIZED, "unable to initialise camera");
        }

        // clean up
        dc1394_camera_free_list(list);
        dc1394_reset_bus(this->dc_camera);
    }

    void Sony::resetInternal()
        throw(CameraException)
    {
        if (this->dc_camera)
        {
            dc1394_camera_free(this->dc_camera);
        }

        dc1394_free(this->dc_device);
    }

    void Sony::printFeatures()
    {
        dc1394featureset_t features;
        dc1394error_t err;

        err = dc1394_feature_get_all(this->dc_camera, &features);
        DC1394_ERR_THROW(err, "unable to query features");

        err = dc1394_feature_print_all(&features, stdout);
        DC1394_ERR_THROW(err, "unable to print features");

    }
/*
    Sony::version_t Sony::getVersion()
    {
        version_t version;
        dc1394error_t err;

        err = dc1394_avt_get_version(this->dc_camera,
                                     &version.uctype, &version.version,
                                     &version.camera_id, &version.fpga_version);
        DC1394_ERR_THROW(err, "unable to get camera version");

        return NULL;
    }
    */

    void Sony::printCameraModell()
    {
        std::cout << "Camera modell: " << this->dc_camera->vendor << " " << this->dc_camera->model << std::endl;
    }

    void Sony::opAutoWhiteBalance()
    {
        setFeatureMode(DC1394_FEATURE_WHITE_BALANCE,
                       DC1394_FEATURE_MODE_ONE_PUSH_AUTO);
    }

	void Sony::disableAutoWhiteBalance()
    {
        setFeatureMode(DC1394_FEATURE_WHITE_BALANCE,
                       DC1394_FEATURE_MODE_MANUAL); 
    }
    void Sony::setWhiteBalance(const white_balance_t wb)
    {
        if (!this->dc_camera) return;

        dc1394error_t err;

        err = dc1394_feature_whitebalance_set_value(this->dc_camera,
                                                    wb.bu, wb.rv);

        DC1394_ERR_THROW(err, "unable to set white balance parameters");
    }

    Sony::white_balance_t Sony::getWhiteBalance()
    {
        if (!this->dc_camera) return white_balance_t();

        white_balance_t wb;
        dc1394error_t err;

        err = dc1394_feature_whitebalance_get_value(this->dc_camera,
                                                    &wb.bu, &wb.rv);

        DC1394_ERR_THROW(err, "unable to set white balance parameters");

        return wb;
    }

    void Sony::setBrightness(unsigned char value)
    {
        setFeatureValue(DC1394_FEATURE_BRIGHTNESS, value);
    }

    unsigned char Sony::getBrightness()
    {
        return getFeatureValue<unsigned char>(DC1394_FEATURE_BRIGHTNESS);
    }

    void Sony::setHue(unsigned short value)
    {
        setFeatureValue(DC1394_FEATURE_HUE, value);
    }

    unsigned short Sony::getHue()
    {
        return getFeatureValue<unsigned char>(DC1394_FEATURE_HUE);
    }

    void Sony::setSaturation(unsigned char value)
    {
	enableFeature(DC1394_FEATURE_SATURATION, true);
        setFeatureValue(DC1394_FEATURE_SATURATION, value);
    }

    unsigned char Sony::getSaturation()
    {
        return getFeatureValue<unsigned char>(DC1394_FEATURE_SATURATION);
    }

    void Sony::setExposure(unsigned short value)
    {
        setFeatureValue(DC1394_FEATURE_EXPOSURE, value);
    }

    unsigned short Sony::getExposure()
    {
        return getFeatureValue<unsigned char>(DC1394_FEATURE_EXPOSURE);
    }

    void Sony::enableGamma(bool enable)
    {
        enableFeature(DC1394_FEATURE_GAMMA, enable);
    }

    bool Sony::isGamma()
    {
        return isFeature(DC1394_FEATURE_GAMMA);
    }

    void Sony::setGamma(unsigned char value)
    {
        setFeatureValue(DC1394_FEATURE_GAMMA, value);
    }

    unsigned char Sony::getGamma()
    {
        return getFeatureValue<unsigned char>(DC1394_FEATURE_GAMMA);
    }

    void Sony::setShutter(unsigned short value)
    {
        setFeatureValue(DC1394_FEATURE_SHUTTER, value);
    }

    unsigned short Sony::getShutter()
    {
        return getFeatureValue<unsigned short>(DC1394_FEATURE_SHUTTER);
    }

    void Sony::enableAutoShutter(bool value)
    {
        if (value)
        {
            setFeatureMode(DC1394_FEATURE_SHUTTER,
                           DC1394_FEATURE_MODE_AUTO);
        }
        else
        {
            setFeatureMode(DC1394_FEATURE_SHUTTER,
                           DC1394_FEATURE_MODE_MANUAL);
        }
    }

    void Sony::enableAutoShutter(bool value, uint32_t min, uint32_t max)
    {
        enableAutoShutter(value);

/*        dc1394error_t err;

        err = dc1394_avt_set_auto_shutter(this->dc_camera, min, max);
        DC1394_ERR_THROW(err, "unable to set min/max values for AVT auto shutter");
*/
    }

    bool Sony::isAutoShutter()
    {
        return (getFeatureMode(DC1394_FEATURE_SHUTTER) ==
                DC1394_FEATURE_MODE_AUTO);
    }

    void Sony::setGain(unsigned short value)
    {
        setFeatureValue(DC1394_FEATURE_GAIN, value);
    }

    unsigned short Sony::getGain()
    {
        return getFeatureValue<unsigned char>(DC1394_FEATURE_GAIN);
    }

    void Sony::enableAutoGain(bool value)
    {
        if (value)
        {
            setFeatureMode(DC1394_FEATURE_GAIN,
                           DC1394_FEATURE_MODE_AUTO);
        }
        else
        {
            setFeatureMode(DC1394_FEATURE_GAIN,
                           DC1394_FEATURE_MODE_MANUAL);
        }
    }

    void Sony::enableAutoGain(bool value, uint32_t min, uint32_t max)
    {
        enableAutoGain(value);

/*        dc1394error_t err;

        err = dc1394_avt_set_auto_gain(this->dc_camera, min, max);
        DC1394_ERR_THROW(err, "unable to set min/max values for AVT auto gain");
*/
    }

    bool Sony::isAutoGain()
    {
        return (getFeatureMode(DC1394_FEATURE_GAIN) ==
                DC1394_FEATURE_MODE_AUTO);
    }

    void Sony::enableTrigger (bool value)
    {
        enableFeature(DC1394_FEATURE_TRIGGER, value);
    }

    bool Sony::isTrigger()
    {
        return isFeature(DC1394_FEATURE_TRIGGER);
    }

    void Sony::setTriggerDelay(unsigned short value)
    {
        setFeatureValue(DC1394_FEATURE_TRIGGER_DELAY, value);
    }

    unsigned short Sony::getTriggerDelay()
    {
        return getFeatureValue<unsigned short>(DC1394_FEATURE_TRIGGER_DELAY);
    }

    void Sony::enableTriggerDelay(bool value)
    {
        enableFeature(DC1394_FEATURE_TRIGGER, value);
    }

    bool Sony::isTriggerDelay()
    {
        return isFeature(DC1394_FEATURE_TRIGGER);
    }

//by me
    void Sony::setFramerate(unsigned short value)
    {
	// not possible with FORMAT7 ... have to be done with packet size
        //setFeatureValue(DC1394_FEATURE_FRAME_RATE, (dc1394framerate_t)value);
	dc1394error_t err;
	err = dc1394_video_set_framerate(this->dc_camera,(dc1394framerate_t)value);

	DC1394_ERR_THROW(err, "unable to set framerate");

    }

    void Sony::initInternal()
        throw(CameraException)
    {
        if (!this->dc_camera)
        {
            DC1394_ERR_THROW(DC1394_CAMERA_NOT_INITIALIZED, "unable to initialise camera");
        }

        dc1394error_t err;

        // reset camera (in case it is being busy)
	//TODO: DOMINIK reset does not work
        err = dc1394_camera_reset(this->dc_camera);
        DC1394_ERR_OUT(err, "camera reset failed");

        // set the isochronous transmission speed
        err = dc1394_video_set_iso_speed(this->dc_camera, DC1394_ISO_SPEED_400);
        DC1394_ERR_THROW(err, "set iso speed failed");

        dc1394video_modes_t modes;

        err = dc1394_video_get_supported_modes(this->dc_camera, &modes);
        DC1394_ERR_THROW(err, "unable to query supported modes");

        bool supported = false;

        std::cout << "Supported video modes:" << std::endl;
        for (unsigned int i = 0; i < modes.num; i++)
        {
            std::cout << modes.modes[i] << std::endl;
            if (modes.modes[i] == this->video_mode)
            {
                supported = true;
            }
        }

        if (!supported)
        {
            DC1394_ERR_THROW(DC1394_FUNCTION_NOT_SUPPORTED, "video mode not supported");
        }

        dc1394_video_set_mode(this->dc_camera, this->video_mode); // currently supported: DC1394_VIDEO_MODE_FORMAT7_0

	//get supported framrates
	//dc1394error_t err;
	dc1394framerates_t framerates;
	err = dc1394_video_get_supported_framerates( this->dc_camera, DC1394_VIDEO_MODE_640x480_YUV422, &framerates);
        std::cout << "Supported framerate modes:" << std::endl;
        for (unsigned int i = 0; i < framerates.num; i++)
        {
            std::cout << framerates.framerates[i] << std::endl;
            /*if (modes.modes[i] == this->video_mode)
            {
                supported = true;
            }*/
        }
	
	dc1394video_mode_t mode;
	//video modes
	err = dc1394_video_get_mode(this->dc_camera, &mode);
        std::cout << "video mode:"<< mode << std::endl;
    
    }

    void Sony::startCaptureInternal()
        throw(CameraException)
    {

        dc1394error_t err;
        // Frame Buffer 10
        err = dc1394_capture_setup(this->dc_camera, 10, DC1394_CAPTURE_FLAGS_DEFAULT);
        DC1394_ERR_THROW(err, "unable to setup capturing");

        err = dc1394_video_set_transmission(this->dc_camera, DC1394_ON);
        DC1394_ERR_THROW(err, "unable to start transmission");

    }

    void Sony::stopCaptureInternal()
        throw(CameraException)
    {
        if (!this->dc_camera) return;

        dc1394_video_set_transmission(this->dc_camera, DC1394_OFF);
        dc1394_capture_stop(this->dc_camera);
    }

    bool Sony::getFrameInternal(Frame &frame)
    {
        // has the camera been initialized properly?
        if (!this->dc_camera)
        {
            DC1394_ERR_THROW(DC1394_CAMERA_NOT_INITIALIZED, "camera not initialised");
        }

        dc1394error_t err;
        dc1394video_frame_t *camFrame;

        // receive a frame
        // wait indefitnitly long:  DC1394_CAPTURE_POLICY_WAIT
        // return immediately if no frame arrived : DC1394_CAPTURE_POLICY_POLL ... error
        err = dc1394_capture_dequeue(this->dc_camera, DC1394_CAPTURE_POLICY_WAIT, &camFrame);
        DC1394_ERR_THROW(err, "failed to receive frame");

        // frame has been captured?
        if (!camFrame)
        {
            return false;
        }

        frame.init(camFrame->size[0], camFrame->size[1], this->data_depth, camera::MODE_YUV422);
        //frame.init(camFrame->size[0], camFrame->size[1], this->data_depth, castor::MODE_COLOR);
        //std::cout << camFrame->size[0] << " " << camFrame->size[1] << " " << this->data_depth<<std::endl;
        /// checking the framerate
        frame.stamp = camFrame->timestamp;
        frame.setImage((const char *)camFrame->image, camFrame->size[0] * camFrame->size[1] * 2); //YUV422

        // release frame structure
        dc1394_capture_enqueue(this->dc_camera, camFrame);

        return true;
    }

}


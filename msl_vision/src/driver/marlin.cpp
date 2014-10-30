#include "marlin.h"
#include <dc1394/vendor/avt.h>
#include <arpa/inet.h>

namespace camera
{

    Marlin::Marlin(uint32_t idx) :
        index(idx), roi(), awb_enabled(true), hdr_enabled(false), data_depth(8),
        dc_camera(NULL), dc_device(NULL), dc_frame(NULL)
    {
        marlinInitInternal(idx);
    }

    void Marlin::marlinInitInternal(uint32_t index)
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

        // initialize new camera structure
        this->dc_camera = dc1394_camera_new(this->dc_device, list->ids[index].guid);
        if (!this->dc_camera)
        {
            DC1394_ERR_THROW(DC1394_CAMERA_NOT_INITIALIZED, "unable to initialise camera");
        }

        // clean up
        dc1394_camera_free_list(list);
        dc1394_reset_bus(this->dc_camera);
    }

    void Marlin::resetInternal()
        throw(CameraException)
    {
        if (this->dc_camera)
        {
            dc1394_camera_free(this->dc_camera);
        }

        dc1394_free(this->dc_device);
    }

    void Marlin::enableHDR(bool enable, uint32_t knee1, uint32_t knee2)
    {
        uint32_t reg;
        dc1394error_t err;

#define REG_CAMERA_AVT_HDR_CONTROL 0x280U
#define REG_CAMERA_AVT_KNEEPOINT_1 0x284U
#define REG_CAMERA_AVT_KNEEPOINT_2 0x288U

        // Retrieve current hdr parameters */
        err = dc1394_get_adv_control_register(this->dc_camera,
                                              REG_CAMERA_AVT_HDR_CONTROL, &reg);
        DC1394_ERR_THROW(err,"Could not get AVT HDR control reg");

        if (enable)
        {
            // Setup knee points
            uint32_t k1 = ntohl(knee1) | 0xFFFFUL; // Manual: value [16..31] needs to be > 0
            err = dc1394_set_adv_control_register(this->dc_camera,
                                                  REG_CAMERA_AVT_KNEEPOINT_1, k1);
            DC1394_ERR_THROW(err,"Could not set AVT kneepoint 1");

            uint32_t k2 = ntohl(knee2) | 0xFFFFUL; // Manual: value [16..31] needs to be > 0
            err = dc1394_set_adv_control_register(this->dc_camera,
                                                  REG_CAMERA_AVT_KNEEPOINT_2, k2);
            DC1394_ERR_THROW(err,"Could not set AVT kneepoint 2");
        }

        // HDR on or off (bit 6)
        reg = (reg & 0xFDFFFFFFUL) | (enable << 25);

        // Define two knee points (bits 28..31)
        reg = ((reg & 0xFFFFFFF0UL) | (0x02UL));

        // Set new hdr parameters
        err = dc1394_set_adv_control_register(this->dc_camera,
                                              REG_CAMERA_AVT_HDR_CONTROL, reg);
        DC1394_ERR_THROW(err,"Could not set AVT HDR control reg");
    }

    bool Marlin::isHDR()
    {
        uint32_t reg;
        dc1394error_t err;

        // Retrieve current hdr parameters
        err = dc1394_get_adv_control_register(this->dc_camera,
                                              REG_CAMERA_AVT_HDR_CONTROL, &reg);
        DC1394_ERR_THROW(err,"Could not get AVT HDR control reg");

        return (reg & 0x02000000); // HDR on or off?
    }

    void Marlin::printFeatures()
    {
        dc1394featureset_t features;
        dc1394error_t err;

        err = dc1394_feature_get_all(this->dc_camera, &features);
        DC1394_ERR_THROW(err, "unable to query features");

        err = dc1394_feature_print_all(&features, stdout);
        DC1394_ERR_THROW(err, "unable to print features");

        std::cout << "AVT Advanced Features" << std::endl;
        dc1394_avt_adv_feature_info_t adv_feature;

        err = dc1394_avt_get_advanced_feature_inquiry(this->dc_camera, &adv_feature);
        DC1394_ERR_THROW(err, "unable to query AVT advanced features");

        err = dc1394_avt_print_advanced_feature(&adv_feature);
        DC1394_ERR_THROW(err, "unable to print AVT advanced features");

    }

    Marlin::version_t Marlin::getVersion()
    {
        version_t version;
        dc1394error_t err;

        err = dc1394_avt_get_version(this->dc_camera,
                                     &version.uctype, &version.version,
                                     &version.camera_id, &version.fpga_version);
        DC1394_ERR_THROW(err, "unable to get camera version");

        return version;
    }

    void Marlin::opAutoWhiteBalance()
    {
        setFeatureMode(DC1394_FEATURE_WHITE_BALANCE,
                       DC1394_FEATURE_MODE_ONE_PUSH_AUTO);
    }

    void Marlin::setWhiteBalance(const white_balance_t wb)
    {
        if (!this->dc_camera) return;

        dc1394error_t err;

        err = dc1394_feature_whitebalance_set_value(this->dc_camera,
                                                    wb.bu, wb.rv);

        DC1394_ERR_THROW(err, "unable to set white balance parameters");
    }

    Marlin::white_balance_t Marlin::getWhiteBalance()
    {
        if (!this->dc_camera) return white_balance_t();

        white_balance_t wb;
        dc1394error_t err;

        err = dc1394_feature_whitebalance_get_value(this->dc_camera,
                                                    &wb.bu, &wb.rv);

        DC1394_ERR_THROW(err, "unable to set white balance parameters");

        return wb;
    }

    void Marlin::setBrightness(unsigned char value)
    {
        setFeatureValue(DC1394_FEATURE_BRIGHTNESS, value);
    }

    unsigned char Marlin::getBrightness()
    {
        return getFeatureValue<unsigned char>(DC1394_FEATURE_BRIGHTNESS);
    }

    void Marlin::setExposure(unsigned char value)
    {
        setFeatureValue(DC1394_FEATURE_EXPOSURE, value);
    }

    unsigned char Marlin::getExposure()
    {
        return getFeatureValue<unsigned char>(DC1394_FEATURE_EXPOSURE);
    }

    void Marlin::enableGamma(bool enable)
    {
        enableFeature(DC1394_FEATURE_GAMMA, enable);
    }

    bool Marlin::isGamma()
    {
        return isFeature(DC1394_FEATURE_GAMMA);
    }

    void Marlin::setGamma(unsigned char value)
    {
        setFeatureValue(DC1394_FEATURE_GAMMA, value);
    }

    unsigned char Marlin::getGamma()
    {
        return getFeatureValue<unsigned char>(DC1394_FEATURE_GAMMA);
    }

    void Marlin::setShutter(unsigned short value)
    {
        setFeatureValue(DC1394_FEATURE_SHUTTER, value);
    }

    unsigned short Marlin::getShutter()
    {
        return getFeatureValue<unsigned short>(DC1394_FEATURE_SHUTTER);
    }

    void Marlin::enableAutoShutter(bool value)
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

    void Marlin::enableAutoShutter(bool value, uint32_t min, uint32_t max)
    {
        enableAutoShutter(value);

        dc1394error_t err;

        err = dc1394_avt_set_auto_shutter(this->dc_camera, min, max);
        DC1394_ERR_THROW(err, "unable to set min/max values for AVT auto shutter");
    }

    bool Marlin::isAutoShutter()
    {
        return (getFeatureMode(DC1394_FEATURE_SHUTTER) ==
                DC1394_FEATURE_MODE_AUTO);
    }

    void Marlin::setGain(unsigned char value)
    {
        setFeatureValue(DC1394_FEATURE_GAIN, value);
    }

    unsigned char Marlin::getGain()
    {
        return getFeatureValue<unsigned char>(DC1394_FEATURE_GAIN);
    }

    void Marlin::enableAutoGain(bool value)
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

    void Marlin::enableAutoGain(bool value, uint32_t min, uint32_t max)
    {
        enableAutoGain(value);

        dc1394error_t err;

        err = dc1394_avt_set_auto_gain(this->dc_camera, min, max);
        DC1394_ERR_THROW(err, "unable to set min/max values for AVT auto gain");
    }

    bool Marlin::isAutoGain()
    {
        return (getFeatureMode(DC1394_FEATURE_GAIN) ==
                DC1394_FEATURE_MODE_AUTO);
    }

    void Marlin::enableTrigger(bool value)
    {
        enableFeature(DC1394_FEATURE_TRIGGER, value);
    }

    bool Marlin::isTrigger()
    {
        return isFeature(DC1394_FEATURE_TRIGGER);
    }

    void Marlin::setTriggerDelay(unsigned short value)
    {
        setFeatureValue(DC1394_FEATURE_TRIGGER_DELAY, value);
    }

    unsigned short Marlin::getTriggerDelay()
    {
        return getFeatureValue<unsigned short>(DC1394_FEATURE_TRIGGER_DELAY);
    }

    void Marlin::enableTriggerDelay(bool value)
    {
        enableFeature(DC1394_FEATURE_TRIGGER, value);
    }

    bool Marlin::isTriggerDelay()
    {
        return isFeature(DC1394_FEATURE_TRIGGER);
    }

//by me
    void Marlin::setFramerate(unsigned short value)
    {
	// not possible with FORMAT7 ... have to be done with packet size
        setFeatureValue(DC1394_FEATURE_FRAME_RATE, value);
    }

    void Marlin::enableMirror(bool value)
    {
        dc1394error_t err;

        err = dc1394_avt_set_mirror(this->dc_camera, static_cast<dc1394bool_t>(value));
        DC1394_ERR_THROW(err, "unable to set AVT mirror mode");
    }

    bool Marlin::isMirror()
    {
        dc1394bool_t value;
        dc1394error_t err;

        err = dc1394_avt_get_mirror(this->dc_camera, &value);
        DC1394_ERR_THROW(err, "unable to get AVT mirror mode");

        return static_cast<bool>(value);
    }

    Marlin::lut_info_t Marlin::getLUTInfo()
    {
        lut_info_t lutinfo;
        dc1394error_t err;

        err = dc1394_avt_get_lut_info(this->dc_camera,
                                      &lutinfo.lut_count, &lutinfo.max_size);
        DC1394_ERR_THROW(err, "unable to query AVT LUT info");

        return lutinfo;
    }

    void Marlin::enableLUT(bool value)
    {
        lut_info_t lutinfo = getLUTInfo();
        if (lutinfo.lut_count < 1)
        {
            DC1394_ERR_THROW(DC1394_FAILURE, "no LUT detected, cannot enable LUT");
        }

        dc1394error_t err;

        err = dc1394_avt_set_lut(this->dc_camera,
                                 static_cast<dc1394bool_t>(value), 0);
        DC1394_ERR_THROW(err, "unable to enable LUT");
    }

    bool Marlin::isLUT()
    {
        dc1394bool_t enabled;
        uint32_t lut_nb;
        dc1394error_t err;

        err = dc1394_avt_get_lut(this->dc_camera, &enabled, &lut_nb);
        DC1394_ERR_THROW(err, "unable to query LUT status");

        return static_cast<bool>(enabled);
    }

    std::vector<uint8_t> Marlin::getLUT()
    {
        dc1394error_t err;

        // Get GPDATA buffer size
        uint32_t bufsize;
        err = dc1394_avt_get_gpdata_info(this->dc_camera, &bufsize);
        DC1394_ERR_THROW(err, "unable to get GPDATA size");

        // Get GPDATA buffer
        std::vector<uint8_t> buf(bufsize);
        err = dc1394_avt_read_gpdata(this->dc_camera, &buf[0], buf.size());
        DC1394_ERR_THROW(err, "unable to read from GPDATA");

        return buf;
    }

    void Marlin::setLUT(std::vector<uint8_t> &lut)
    {
        dc1394bool_t enabled = static_cast<dc1394bool_t>(true);
        dc1394error_t err;

        // Initialise write
        err = dc1394_avt_set_lut_mem_ctrl(this->dc_camera, enabled, 0, 0);
        DC1394_ERR_THROW(err, "unable to enable LUT mem access to GPDATA");

        // Get GPDATA buffer size
        uint32_t bufsize;
        err = dc1394_avt_get_gpdata_info(this->dc_camera, &bufsize);
        DC1394_ERR_THROW(err, "unable to get GPDATA size");

        if (bufsize != lut.size())
        {
            DC1394_ERR_THROW(DC1394_FAILURE, "wrong LUT size");
        }

        // Write GPDATA buffer
        err = dc1394_avt_write_gpdata(this->dc_camera, &lut[0], lut.size());
        DC1394_ERR_THROW(err, "unable to write to GPDATA");

        // Set memory to reon only again
        enabled = static_cast<dc1394bool_t>(false);
        err = dc1394_avt_set_lut_mem_ctrl(this->dc_camera, enabled, 0, 0);
        DC1394_ERR_THROW(err, "unable to disable LUT mem access to GPDATA");
    }

    uint32_t Marlin::getTestImage()
    {
        uint32_t image;
        dc1394error_t err;

        err = dc1394_avt_get_test_images(this->dc_camera, &image);
        DC1394_ERR_THROW(err, "unable to get active AVT test image");

        return image;
    }

    void Marlin::enableTestImage(bool enable, uint32_t number)
    {
        dc1394error_t err;

        err = dc1394_avt_set_test_images(this->dc_camera, (enable ? number : 0));
        DC1394_ERR_THROW(err, "unable to set AVT test image");
    }

    uint32_t Marlin::getFrameCount()
    {
        uint32_t count;
        dc1394error_t err;

        err = dc1394_avt_get_frame_info(this->dc_camera, &count);
        DC1394_ERR_THROW(err, "unable to set AVT frame info");

        return count;
    }

    void Marlin::resetFrameCount()
    {
        dc1394error_t err;

        err = dc1394_avt_reset_frame_info(this->dc_camera);
        DC1394_ERR_THROW(err, "unable to reset AVT frame info");
    }

    void Marlin::softReset()
    {
        dc1394error_t err;

        stopCapture();

        err = dc1394_avt_reset(this->dc_camera);
        DC1394_ERR_THROW(err, "uanble to soft reset AVT Marlin");

        sleep(5);

        reset();

        marlinInitInternal(this->index);
        init();

        startCapture();
    }

    void Marlin::initInternal()
        throw(CameraException)
    {
        if (!this->dc_camera)
        {
            DC1394_ERR_THROW(DC1394_CAMERA_NOT_INITIALIZED, "unable to initialise camera");
        }

        dc1394error_t err;

        // reset camera (in case it is being busy)
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
    }

    void Marlin::startCaptureInternal()
        throw(CameraException)
    {

        if ((this->roi.x == 0) && (this->roi.y == 0) &&
            (this->roi.w == 0) && (this->roi.h == 0))
        {
            std::cout << "No ROI set, querying max reachable resolution" << std::endl;

            uint32_t max_width;
            uint32_t max_height;
            dc1394error_t err;

            err = dc1394_avt_get_MaxResolution(this->dc_camera,
                                               &max_width, &max_height);
            DC1394_ERR_THROW(err, "unable to query max reachable resolution");

            //this->roi.w = max_width;
	//this->roi.w = 640; 1024, 768
	this->roi.w = 776;
            //this->roi.h = max_height;
	//this->roi.h = 480;
	this->roi.h = 776;

            std::cout << "Setting ROI to (0, 0, " << roi.w << ", " << roi.h << ")" << std::endl;
        }

        dc1394error_t err;
        err = dc1394_format7_set_roi(this->dc_camera,
                                     DC1394_VIDEO_MODE_FORMAT7_0,
                                     DC1394_COLOR_CODING_YUV422, //DC1394_COLOR_CODING_MONO8, //
                                     DC1394_USE_MAX_AVAIL, //DC1394_USE_RECOMMENDED, // use max packet size
                                     this->roi.x, this->roi.y,
                                     this->roi.w, this->roi.h);
        DC1394_ERR_THROW(err, "unable to initialise roi");

	//err = dc1394_format7_set_packet_size(this->dc_camera, DC1394_VIDEO_MODE_FORMAT7_0, 2468);
        //DC1394_ERR_THROW(err, "unable set packet size");

	 err = dc1394_capture_setup(this->dc_camera, 1, DC1394_CAPTURE_FLAGS_DEFAULT);
        DC1394_ERR_THROW(err, "unable to setup capturing");
        err = dc1394_video_set_transmission(this->dc_camera, DC1394_ON);
        DC1394_ERR_THROW(err, "unable to start transmission");

    }

    void Marlin::stopCaptureInternal()
        throw(CameraException)
    {
        if (!this->dc_camera) return;

        dc1394_video_set_transmission(this->dc_camera, DC1394_OFF);
        dc1394_capture_stop(this->dc_camera);
    }

    bool Marlin::getFrameInternal(Frame &frame)
    {
        // has the camera been initialized properly?
        if (!this->dc_camera)
        {
            DC1394_ERR_THROW(DC1394_CAMERA_NOT_INITIALIZED, "camera not initialised");
        }

        dc1394error_t err;
        dc1394video_frame_t *camFrame;

        // receive a frame
        err = dc1394_capture_dequeue(this->dc_camera, DC1394_CAPTURE_POLICY_WAIT, &camFrame);
        DC1394_ERR_THROW(err, "failed to receive frame");

        // frame has been captured?
        if (!camFrame)
        {
            return false;
        }

        frame.init(camFrame->size[0], camFrame->size[1], this->data_depth, camera::MODE_COLOR);
        //MODE_RAW
        /// checking the framerate
        frame.stamp = camFrame->timestamp;
        //*3 for 3 bytes per image point
        frame.setImage((const char *)camFrame->image, camFrame->size[0] * camFrame->size[1] * 3);

        // release frame structure
        dc1394_capture_enqueue(this->dc_camera, camFrame);

        return true;
    }

	void Marlin::DummyTestMethod(){

	//
//	float value;
//	dc1394_format7_get_frame_interval(this->dc_camera, DC1394_VIDEO_MODE_FORMAT7_0, &value);
//printf(": %f\n",value);

	//uint32_t value;
	//dc1394_format7_get_packet_size(this->dc_camera,DC1394_VIDEO_MODE_FORMAT7_0, &value);
	//std::cout << "p size: " << value << std::endl;
	std::cout << "hdr: " << isHDR() << std::endl;
	//dc1394_video_set_framerate(this->dc_camera, DC1394_FRAMERATE_60);
	}
}


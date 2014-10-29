#ifndef CAMERA_PS3EYE_H
#define CAMERA_PS3EYE_H 1

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <string>

#include <fcntl.h>              /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <malloc.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <asm/types.h>          /* for videodev2.h */
#include <linux/videodev2.h>

//#include <SystemConfig.h>

#define CLEAR(x) memset (&(x), 0, sizeof (x))

#define V4L2_CID_SHARPNESS			(V4L2_CID_PRIVATE_BASE+2)

#include "cameraUSB.h"
#include "frame.h"

namespace camera
//namespace castor
{

typedef enum {
        IO_METHOD_READ,
        IO_METHOD_MMAP,
        IO_METHOD_USERPTR,
} io_method;

struct buffer {
        void *                  start;
        size_t                  length;
};


class CameraQuickCamException : public std::exception {
    protected:
        std::string cause;

    public:
        CameraQuickCamException(
                std::string cause)
        {
            this->cause = cause;
        }

        virtual ~CameraQuickCamException() throw() {
        };

        virtual const char *what() {
            return this->cause.c_str();
        }
};


    class Ps3Eye : public CameraUsb
    {
        public:
			static const unsigned short usImageWidth = 640;
			static const unsigned short usImageHeight = 480;

            Ps3Eye(uint32_t index = 0);

	    
	    //////////////////// GENERAL USE FROM CAMERA.H /////////////////////////////
	    
            void setROI(const camera_roi_t &roi) throw(CameraException) { this->roi = roi; }

            void resetInternal() throw(CameraException);
	    
            void initInternal() throw(CameraException);

            void startCaptureInternal() throw(CameraException);

	    void stopCaptureInternal() throw(CameraException);

            bool getFrameInternal(Frame &frame);

            //unsigned short getImageWidth();

            //unsigned short getImageHeight();
	    
	    char * getCaptureBuffer();
	    
	    //////////// CAMERA SPECIFIC FUNCTIONS ///////////////////////////////////////
	    
	    int set_control(__u32 id, __s32 value);


//             void opAutoWhiteBalance();
// 	void disableAutoWhiteBalance();
//             struct white_balance_t {
//                 uint32_t bu;
//                 uint32_t rv;
//             };
// 
//             void setWhiteBalance(const white_balance_t wb);
//             white_balance_t getWhiteBalance();
// 
//             void setBrightness(unsigned char value); //0 255
//             unsigned char getBrightness();
// 
//             // set Hue 0 359
//             void setHue (unsigned short value);
//             unsigned short getHue();
// 
//             //set Saturation ... 0 255
//             void setSaturation (unsigned char value);
//             unsigned char getSaturation();
// 
//             // set Exposure ... 0 2000
//             void setExposure(unsigned short value);
//             unsigned short getExposure();
// 
//             void enableGamma(bool value);
//             bool isGamma();
// 
//             void setGamma(unsigned char value);
//             unsigned char getGamma();
// 
//             //set Shutter ... 0 4000
//             void setShutter(unsigned short value);
//             unsigned short getShutter();
// 
//             void enableAutoShutter(bool value);
//             void enableAutoShutter(bool value, uint32_t min, uint32_t max);
//             bool isAutoShutter();
// 
            //set Gain ... 0 664
            void setGain(__s32 value);
            unsigned short getGain();
// 
//             void enableAutoGain(bool value);
//             void enableAutoGain(bool value, uint32_t min, uint32_t max);
//             bool isAutoGain();
// 
//             void enableTrigger(bool value);
//             bool isTrigger();
// 
//             void setTriggerDelay(unsigned short value);
//             unsigned short getTriggerDelay();
// 
//             void enableTriggerDelay(bool value);
//             bool isTriggerDelay();
// 
//             void setFramerate(unsigned short value);
// 
//             void enableMirror(bool value);
//             bool isMirror();
// 
//             struct lut_info_t
//             {
//                 uint32_t lut_count;
//                 uint32_t max_size;
//             };
// 
//             lut_info_t getLUTInfo();
// 
//             void enableLUT(bool value);
//             bool isLUT();
// 
//             std::vector<uint8_t> getLUT();
//             void setLUT(std::vector<uint8_t> &lut);
// 
//             uint32_t getTestImage();
//             void enableTestImage(bool enable, uint32_t number);
// 
//             uint32_t getFrameCount();
//             void resetFrameCount();

//            void softReset();	    


        protected:

	  char * dev_name;
	  io_method io;
	  int fd;
	  struct buffer * buffers;
	  unsigned int n_buffers;
	  int id;
	  char * captureBuffer;
	  
	  uint32_t index;
	  camera_roi_t roi;
	  bool awb_enabled;
	  //bool hdr_enabled;
	  unsigned int data_depth;
		
	  Ps3Eye() {}

	  void ps3eyeInitInternal(uint32_t index);
	  
	  void errno_exit(const char * s);
	  int xioctl(int fd, int request, void * arg);
	  int read_frame();
	  void mainloop();
	  void stop_capturing();
	  void start_capturing();
	  void uninit_device();
	  void init_read(unsigned int buffer_size);
	  void init_mmap();
	  void init_device();
	  void close_device();
	  void open_device();

    };

}

#endif /* CAMERA_PS3EYE_H */


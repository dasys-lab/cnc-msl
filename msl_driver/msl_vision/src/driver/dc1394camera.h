#ifndef DC1394CAMERA_H
#define DC1394CAMERA_H

#undef BIG_ENDIAN

#include "frame.h"
//#include "../calibration/calibration.h"
//#include <cv.h>
//#include <cvaux.h>
#include <dc1394/dc1394.h>
#include <dc1394/register.h>
#include <string>

namespace camera
//namespace castor
{

	class DC1394Camera
	{
		public:
			DC1394Camera();
			~DC1394Camera();
			/* Camera error codes */
			enum error_t
			{
				OK = 0,   /* Success is zero */
				FAILURE,     /* Errors are positive numbers */
				ENUM_FAILED, /* Failed to enumerate camera */
				NO_CAMERA_FOUND, /* Failed to detect a camera */
				COULD_NOT_INIT, /* Failed to init camera */
				CAMERA_INDEX_NOT_FOUND, /* camera index out of bounds */
				UNSUPPORTED_DATA_DEPTH, /* required data depth is not supported in setupCamera */
				CONFIG_FAILED,
				CAPTURE_FAILED,

				INVALID_INPUT_FORMAT /*! the frame given as input is not in
                                           a valid format for the requested
                                           operation */
			};

			enum   calibration_type_t
			{
				BLACK_FRAME,
				WHITE_FRAME,
			};

            enum IMAGE_MODES
            {
                MODE_640x512_8,
                MODE_640x512_12,
                MODE_1280x1024_8,
                MODE_1280x1024_12
            };

			/** select the camera by index
			*/
			bool selectCamera ( unsigned int index = 0 );

			/** camera setup TODO: add configuration options here
			*/
                        bool setupCamera ( IMAGE_MODES mode );

			/** Returns the last error code registered by the driver */
			error_t error() const { return mError; }
			/** Returns the string describing the given error code */
			static char const* errorString ( int errorCode );
			/** Returns a string describing the current error */
			char const* errorString() const { return errorString ( mError ); }

                        /**
                        * turns on HDR mode
                        */
			void turnOnHDR();
                        /**
                         * turns off HDR mode (both normal and unfolding HDR)
                         */
			void turnOffHDR();
                        /**
                         * turns on HDR with afterwards 'unfolding' it. This means, that, using the HDR
                         * parameters the theoreticaly (but actually not representable) pixel values are
                         * calculated and then the pixels are normalized to
                         * 0 ... 'theoretical max value' -> 0 ... 2^image_depth-1
                         */
                        void turnONunfoldHDR();


                        /**
                        * setters for HDR mode (integration times and reset values)
                        */
                        void setResetValueAndIntegrationTime ( float reset_val1, float reset_val2, float integ_time_1, float integ_time_2, float integ_time_3 );
                        void setResetValue ( float reset_val1, float reset_val2);
                        void setIntegrationTime ( float integ_time_1, float integ_time_2, float integ_time_3 );



                        /**
                         * returns a raw frame
                         */
                        bool getRawFrame       ( Frame& frame );
                        /**
                         * returns a debayerized color frame
                         */
                        bool getColorFrame     ( Frame& frame);
                        /**
                         * returns the width of the capured frames in this mode
                         */
                        int get_mode_width() { return mode_width; }
                        /**
                         * returns the height of the capured frames in this mode
                         */
                        int get_mode_height() { return mode_height; }

                        int get_mode_depth() { return depth;  }


		private:

			bool error ( error_t code );

                        bool sendBci5Cmd ( uint16_t cmd );

			/** The current error */
			error_t mError;

			/**
			 * integration_times represents the times between the two reset
			 * points and the final read, in milliseconds. So
			 * integration_times[0] is the time between start and first reset,
			 * integration_times[1] the time between the two resets and
			 * integration_times[2] the time between the second reset and the
			 * final read. All those times are set in milliseconds.
			 *
			 * reset_values are the two reset values, in pixel values.
			 * Therefore, 0 means always reset (the pixel is always set to 0 at
			 * the given reset point) and 255 never.
			 */
			struct hdr_t
			{
				float integration_times[3]; /*! integration time in milliseconds */
				float  reset_values[2];    /*! reset values. 0 means always
                                              reset and 255 never */
			};
			static const int HDR_SLOPE_COUNT = 3;
                        bool unfoldHDR ( Frame& camFrame );
			bool usingHDR, unfold_HDR;
			hdr_t hdr;

			template<int channel_size>
			void unfoldHDR ( Frame& camFrame );

			dc1394camera_t *mCamera;
			dc1394_t *md;

			dc1394error_t err;
			std::string calibration_folder;

                        dc1394video_frame_t *camFrame;
                        IMAGE_MODES selected_mode;

                        int mode_width, mode_height;
                        int depth;
	};
}

#endif

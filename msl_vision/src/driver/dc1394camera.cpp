#include <stdio.h>
#include <string>
#include <iostream>

#include "dc1394camera.h"
//#include "filter/frame2rggb.h"
#include <limits>


#define LIBDC1394_ERR(code,err)                              \
    if( code != DC1394_SUCCESS ) {                           \
        cerr << "libdc1394: " << dc1394_error_get_string(code) \
	<< " (" <<__FILE__ << ":" << __LINE__ << ")" << std::endl; 		\
        return error(err);                                   \
    }


using namespace std;
using namespace camera;
//using namespace castor;

DC1394Camera::DC1394Camera()
		: usingHDR ( false ), unfold_HDR ( false ), mCamera ( NULL )
{
	md = dc1394_new();
}


DC1394Camera::~DC1394Camera()
{
	// close camera
    if ( mCamera )
    {
        dc1394_video_set_transmission ( mCamera, DC1394_OFF );
        dc1394_capture_stop ( mCamera );
        dc1394_camera_free ( mCamera );
    }
    dc1394_free ( md );
}



bool DC1394Camera::selectCamera ( unsigned int index )
{
	dc1394camera_list_t * list;
	dc1394error_t err;

	// get list of available cameras
        err=dc1394_camera_enumerate ( md, &list );
	LIBDC1394_ERR ( err, ENUM_FAILED )

	if ( list->num == 0 )
		return error ( NO_CAMERA_FOUND );

	if ( list->num < index )
		return error ( CAMERA_INDEX_NOT_FOUND );

    // initialize new camera structure
	mCamera = dc1394_camera_new ( md, list->ids[index].guid );
	if ( !mCamera )
	{
		return error ( COULD_NOT_INIT );
	}

    // clean up
	dc1394_camera_free_list ( list );
	dc1394_reset_bus ( mCamera );

	return true;
}




bool DC1394Camera::setupCamera ( IMAGE_MODES mode )
{
        selected_mode = mode;

    // check if selectCamera has been successfully called before
	if ( !mCamera )
		return error ( COULD_NOT_INIT );


    // reset camera (in case it is being buisy)
    err=dc1394_camera_reset ( mCamera );
	LIBDC1394_ERR ( err, CONFIG_FAILED )

    // set the isochronous transmission speed
    err=dc1394_video_set_iso_speed ( mCamera, DC1394_ISO_SPEED_400 );
    LIBDC1394_ERR ( err, CONFIG_FAILED )


//         uint32_t value, sup_formats;
//         dc1394video_mode_t m;
//
//         err=dc1394_get_control_register(mCamera, REG_CAMERA_V_FORMAT_INQ, &sup_formats);
//         LIBDC1394_ERR(err, CONFIG_FAILED);


    // set the mode (FORMAT7_0 is currently the only working mode with this working set)
    dc1394_video_set_mode(mCamera, DC1394_VIDEO_MODE_FORMAT7_0);


    // set parmeters (depth, width, height, packet size)
    if ( mode == MODE_640x512_8 )
	{
		mode_width	= 640;
                mode_height 	= 512;
                depth		= 8;
                err = dc1394_format7_set_roi(mCamera, DC1394_VIDEO_MODE_FORMAT7_0,
					DC1394_COLOR_CODING_MONO8,
					DC1394_USE_MAX_AVAIL, // use max packet size
					0, 0, // left, top
					mode_width, mode_height);  // width, height
	}
        else if ( mode == MODE_640x512_12 )
	{
                mode_width	= 640;
                mode_height 	= 512;
                depth		= 12;
		err = dc1394_format7_set_roi(mCamera, DC1394_VIDEO_MODE_FORMAT7_0,
					DC1394_COLOR_CODING_MONO16,
					DC1394_USE_MAX_AVAIL, // use max packet size
					0, 0, // left, top
					mode_width, mode_height);  // width, height
	} else if ( mode == MODE_1280x1024_8 )
	{
                mode_width	= 1280;
                mode_height 	= 1024;
                depth		= 8;
		err = dc1394_format7_set_roi(mCamera, DC1394_VIDEO_MODE_FORMAT7_0,
					DC1394_COLOR_CODING_MONO8,
					DC1394_USE_MAX_AVAIL, // use max packet size
					0, 0, // left, top
					mode_width, mode_height);  // width, height
	}
	else if ( mode == MODE_1280x1024_12 )
        {
                mode_width	= 1280;
                mode_height 	= 1024;
                depth		= 8;
		err = dc1394_format7_set_roi(mCamera, DC1394_VIDEO_MODE_FORMAT7_0,
					DC1394_COLOR_CODING_MONO16,
					DC1394_USE_MAX_AVAIL, // use max packet size
					0, 0, // left, top
					mode_width, mode_height);  // width, height
        }
	else
		return error ( UNSUPPORTED_DATA_DEPTH );

    LIBDC1394_ERR ( err, CONFIG_FAILED )


    // after selecting a mode and setting the parameters, now setup the capturing
    err=dc1394_capture_setup(mCamera, 1, DC1394_CAPTURE_FLAGS_DEFAULT);
	LIBDC1394_ERR ( err, CONFIG_FAILED )


    // now start actual data transmission
	err=dc1394_video_set_transmission ( mCamera, DC1394_ON );
	LIBDC1394_ERR ( err, CONFIG_FAILED )


	// set HDR parameters
    setResetValueAndIntegrationTime ( this->hdr.reset_values[0], this->hdr.reset_values[1], this->hdr.integration_times[0], hdr.integration_times[1], hdr.integration_times[2] );

    // implement lower image resolution as subsampling the image
    if( mode == MODE_640x512_8 || mode == MODE_640x512_12 )
    {
        sendBci5Cmd (  0xF624);    // set subsampling XY
        sendBci5Cmd (  0xF770);         // enable subsampling
    }

	turnOffHDR();
	return true;
}



bool DC1394Camera::getRawFrame ( Frame& frame )
{
    	// has the camera been initialized properly?
	if ( !mCamera )
		return error ( COULD_NOT_INIT );

	dc1394video_frame_t *camFrame;

	// receive a frame
        err=dc1394_capture_dequeue ( mCamera, DC1394_CAPTURE_POLICY_WAIT, &camFrame );
	LIBDC1394_ERR ( err, CAPTURE_FAILED )


        // frame has been captured?
	if ( !camFrame )
		return false;


        frame.setDataDepth( get_mode_depth() );

        // HDR mode has been set ON?
        if ( usingHDR )
            frame.init ( camFrame->size[0], camFrame->size[1], frame.getDataDepth(), camera::MODE_HDR );
            //frame.init ( camFrame->size[0], camFrame->size[1], frame.getDataDepth(), castor::MODE_HDR );
        else
            frame.init ( camFrame->size[0], camFrame->size[1], frame.getDataDepth(), camera::MODE_RAW );
            //frame.init ( camFrame->size[0], camFrame->size[1], frame.getDataDepth(), castor::MODE_RAW );


        // checking the framerate
        frame.stamp = camFrame->timestamp;

        // Camera data is in big endian. Convert to little endian if the bit
	// depth is higher than 8 bits.

#ifdef BIG_ENDIAN
	memcpy ( &frame.image[0], camFrame->image, frame.image.size() );
#else
	size_t numPixels = camFrame->size[0] * camFrame->size[1];


        if ( frame.getDataDepth() > 8 )
	{
		uint16_t* src  = ( uint16_t* ) camFrame->image;
		uint16_t* dest = ( uint16_t* ) &frame.getImagePtr()[0];

		for ( size_t i = 0; i < numPixels; ++i )
		{
			uint16_t lb = ( src[i] & 0xFF00 ) >> 8;
			uint16_t hb = ( src[i] & 0x00FF ) << 8;
			dest[i] = ( hb | lb );
		}
	}
	else
		memcpy ( &frame.getImagePtr()[0], camFrame->image, frame.getPixelCount()*frame.getPixelSize());

#endif

	// release frame structure
	dc1394_capture_enqueue ( mCamera, camFrame );
	return true;
}



bool DC1394Camera::getColorFrame ( Frame& frameRGB)
{
    Frame frame;
    // get raw frame
    if ( ! getRawFrame ( frame ) )
    return false;

    // convert to unfolded HDR frame?
    if ( frame.isHDR() && unfold_HDR )
    unfoldHDR ( frame );

    // debayerize frame
	return true; //filter::Frame2RGGB::process(frame, frameRGB);
}


bool DC1394Camera::sendBci5Cmd ( uint16_t cmd )
{
    return dc1394_set_register ( mCamera, 0xF00A10, cmd );

}



bool DC1394Camera::error ( error_t errorCode )
{
	mError = errorCode;
	if ( mError == DC1394Camera::OK )
		return true;
	else
		return false;
}



// HDR related functions


bool DC1394Camera::unfoldHDR ( Frame& camFrame )
{
	if ( !camFrame.isHDR() )
		return false;

	if ( camFrame.getDataDepth() <= 8 )
		unfoldHDR<8> ( camFrame );
	else
		unfoldHDR<16> ( camFrame );
	return true;
}

template<int channel_size>
void DC1394Camera::unfoldHDR ( Frame& camFrame )
{
    // for optionally casting to >8 bit
	typedef typename PixelTraits<channel_size>::channel_t channel_t;
	channel_t *image = ( channel_t* ) &camFrame.getImagePtr()[0];

    // higher slope (of voltage current) means that HDR reseted a pixel
	float slope_limit1 = hdr.reset_values[0] * channel_size > 8? 256.0 : 1.0 / hdr.integration_times[0];
	float slope_limit2 = hdr.reset_values[1] * channel_size > 8? 256.0 : 1.0 / ( hdr.integration_times[1] + hdr.integration_times[2] );

        for ( uint32_t i = 0; i <camFrame.getPixelCount(); ++i )
	{
		float pixel = image[i];

        // calculate slope (voltage current)
		float total_slope =
		    pixel / ( hdr.integration_times[0] + hdr.integration_times[1] + hdr.integration_times[2] );

        // has HDR reseted the pixel value once?
		if ( total_slope > slope_limit1 )
		{
			float next_slope = ( pixel - ( total_slope * hdr.integration_times[0] ) ) / ( hdr.integration_times[1] + hdr.integration_times[2] );

            // has HDR reseted the pixel value twice?
			if ( next_slope > slope_limit2 )
			{
                // calculate the theoretical pixel value, if it was reseted once
				pixel = total_slope * hdr.integration_times[0] + next_slope * ( hdr.integration_times[1] + hdr.integration_times[2] );
			}
			else
                // calculate the theoretical pixel value, if it was reseted twice
				pixel = total_slope * ( hdr.integration_times[0] + hdr.integration_times[1] + hdr.integration_times[2] );

		}
                // maximum possible pixel value
                uint32_t max_pixel_value = ( 1 << static_cast<uint32_t> ( channel_size ) ) - 1;
                // set the pixel to its original value
		image[i] = static_cast<channel_t> ( pixel > max_pixel_value ? max_pixel_value : pixel );
	}
// 	camFrame.setMode( camera::MODE_RAW );
	camFrame.setDataDepth( channel_size );
}


void DC1394Camera::setResetValue ( float r1,float r2 )
{
    this->hdr.reset_values[0] = r1;
    this->hdr.reset_values[1] = r2;
    setResetValueAndIntegrationTime ( r1, r2, this->hdr.integration_times[0], this->hdr.integration_times[1], this->hdr.integration_times[2] );
}

void DC1394Camera::setIntegrationTime ( float i1, float i2, float i3 )
{
    this->hdr.integration_times[0] = i1;
    this->hdr.integration_times[1] = i2;
    this->hdr.integration_times[2] = i3;
    setResetValueAndIntegrationTime ( this->hdr.reset_values[0], this->hdr.reset_values[1], i1, i2, i3 );
}

void DC1394Camera::setResetValueAndIntegrationTime ( float reset_value1, float reset_value2, float integration_time1, float integration_time2, float integration_time3 )
{
    sendBci5Cmd ( 0xF200 );	// select slope 1
	// 1000 -> ms to sec, 40 -> values are needed
    sendBci5Cmd ( 0xE000 + ( ( static_cast<int> ( integration_time1 * 1000.0 * 40.0 ) >> 0 ) & 0xFF ) );//byte 0(LSB) set reset time
    sendBci5Cmd ( 0xE100 + ( ( static_cast<int> ( integration_time1 * 1000.0 * 40.0 ) >> 8 ) & 0xFF ) );	// byte 1
    sendBci5Cmd ( 0xE200 + ( ( static_cast<int> ( integration_time1 * 1000.0 * 40.0 ) >> 16 ) & 0xFF ) );	// byte 2
    sendBci5Cmd ( 0xE300 + ( ( static_cast<int> ( integration_time1 * 1000.0 * 40.0 ) >> 24 ) & 0xFF ) );	// byte 3
    sendBci5Cmd ( 0xEB00 );	// set reset voltage for slope 1 to maximum (4V)

    sendBci5Cmd ( 0xF201 );	// select slope 2
    sendBci5Cmd ( 0xE000 + ( ( static_cast<int> ( integration_time2 * 1000.0 * 40.0 ) >> 0 ) & 0xFF ) );	// byte 0 (LSB)
    sendBci5Cmd ( 0xE100 + ( ( static_cast<int> ( integration_time2 * 1000.0 * 40.0 ) >> 8 ) & 0xFF ) );	// byte 1
    sendBci5Cmd ( 0xE200 + ( ( static_cast<int> ( integration_time2 * 1000.0 * 40.0 ) >> 16 ) & 0xFF ) );	// byte 2
    sendBci5Cmd ( 0xE300 + ( ( static_cast<int> ( integration_time2 * 1000.0 * 40.0 ) >> 24 ) & 0xFF ) );	// byte 3
    sendBci5Cmd ( 0xE900 + ( static_cast<int> ( ( 4.0-reset_value1 ) / 0.1 * 6.375 ) ) );

    sendBci5Cmd ( 0xF202 );	// select slope 3
    sendBci5Cmd ( 0xE000 + ( ( static_cast<int> ( integration_time3 * 1000.0 * 40.0 ) >> 0 ) & 0xFF ) );	// byte 0 (LSB)
    sendBci5Cmd ( 0xE100 + ( ( static_cast<int> ( integration_time3 * 1000.0 * 40.0 ) >> 8 ) & 0xFF ) );	// byte 1
    sendBci5Cmd ( 0xE200 + ( ( static_cast<int> ( integration_time3 * 1000.0 * 40.0 ) >> 16 ) & 0xFF ) );	// byte 2
    sendBci5Cmd ( 0xE300 + ( ( static_cast<int> ( integration_time3 * 1000.0 * 40.0 ) >> 24 ) & 0xFF ) );	// byte 3
    sendBci5Cmd ( 0xE900 + ( static_cast<int> ( ( 4.0-reset_value2 ) / 0.1 * 6.375 ) ) );
}


void DC1394Camera::turnOnHDR()
{
    sendBci5Cmd ( 0xFFC3 ); // enable multi-slope
}

void DC1394Camera::turnOffHDR()
{
    usingHDR = false;
    unfold_HDR = false;
    sendBci5Cmd ( 0xFFC2 ); // disable multi-slope
}

void DC1394Camera::turnONunfoldHDR()
{
    usingHDR = true;
    unfold_HDR = true;
    sendBci5Cmd ( 0xFFC3 ); // enable multi-slope
}





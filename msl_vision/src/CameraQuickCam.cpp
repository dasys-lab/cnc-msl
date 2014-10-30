/*
 * $Id: CameraQuickCam.cpp 1941 2007-03-20 18:24:01Z cn $
 *
 *
 * Copyright 2005,2006 Carpe Noctem, Distributed Systems Group,
 * University of Kassel. All right reserved.
 *
 * The code is derived from the software contributed to Carpe Noctem by
 * the Carpe Noctem Team.
 *
 * The code is licensed under the Carpe Noctem Userfriendly BSD-Based
 * License (CNUBBL). Redistribution and use in source and binary forms,
 * with or without modification, are permitted provided that the
 * conditions of the CNUBBL are met.
 *
 * You should have received a copy of the CNUBBL along with this
 * software. The license is also available on our website:
 * http://carpenoctem.das-lab.net/license.txt
 *
 *
 * <description>
 */
#include "CameraQuickCam.h"
#include <iostream>


CameraQuickCam::CameraQuickCam() : sc()


{

	this->sc = SystemConfig::getInstance();

	io = IO_METHOD_MMAP;
	buffers = NULL;
	n_buffers  = 0;
	id = 0;

        dev_name = "/dev/video0";
        open_device();
        init_device();
        start_capturing ();

	//captureBuffer = (char *) malloc(640*480*2);


}


CameraQuickCam::~CameraQuickCam()
{
        stop_capturing ();
        uninit_device ();
	close_device ();



}


void CameraQuickCam::errno_exit(const char * s){

        fprintf (stderr, "%s error %d, %s\n",
                 s, errno, strerror (errno));

        exit (EXIT_FAILURE);

}


int CameraQuickCam::xioctl(int fd, int request, void * arg){

        int r;

        do r = ioctl (fd, request, arg);
        while (-1 == r && EINTR == errno);

        return r;


}


int CameraQuickCam::read_frame(){

        struct v4l2_buffer buf;

	CLEAR (buf);

	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.memory = V4L2_MEMORY_MMAP;

	if (-1 == xioctl (fd, VIDIOC_DQBUF, &buf)) {
		switch (errno) {
		case EAGAIN:
			return 0;

		case EIO:
			/* Could ignore EIO, see spec. */

			/* fall through */

		default:
			errno_exit ("VIDIOC_DQBUF");
		}
	}

	assert (buf.index < n_buffers);

	captureBuffer = (char *) buffers[buf.index].start;

	//process_image (buffers[buf.index].start);

	if (-1 == xioctl (fd, VIDIOC_QBUF, &buf))
		errno_exit ("VIDIOC_QBUF");

 
        return 1;

}



void CameraQuickCam::mainloop(){



}

char * CameraQuickCam::getCaptureBuffer(){

	return captureBuffer;

}



void CameraQuickCam::stop_capturing(){

        enum v4l2_buf_type type;

	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	if (-1 == xioctl (fd, VIDIOC_STREAMOFF, &type))
		errno_exit ("VIDIOC_STREAMOFF");


}



void CameraQuickCam::start_capturing(){

        unsigned int i;
        enum v4l2_buf_type type;

	for (i = 0; i < n_buffers; ++i) {
		struct v4l2_buffer buf;

		CLEAR (buf);

		buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory      = V4L2_MEMORY_MMAP;
		buf.index       = i;

		if (-1 == xioctl (fd, VIDIOC_QBUF, &buf))
			errno_exit ("VIDIOC_QBUF");
	}

	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	if (-1 == xioctl (fd, VIDIOC_STREAMON, &type))
		errno_exit ("VIDIOC_STREAMON");


}


void CameraQuickCam::uninit_device(){

        unsigned int i;

	for (i = 0; i < n_buffers; ++i)
		if (-1 == munmap (buffers[i].start, buffers[i].length))
			errno_exit ("munmap");

        free (buffers);


}



void CameraQuickCam::init_read(unsigned int buffer_size){

        buffers = (buffer *)calloc (1, sizeof (*buffers));

        if (!buffers) {
                fprintf (stderr, "Out of memory\n");
                exit (EXIT_FAILURE);
        }

        buffers[0].length = buffer_size;
        buffers[0].start = malloc (buffer_size);

        if (!buffers[0].start) {
                fprintf (stderr, "Out of memory\n");
                exit (EXIT_FAILURE);
        }

}


void CameraQuickCam::init_mmap(){

        struct v4l2_requestbuffers req;

        CLEAR (req);

        req.count               = 4;
        req.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory              = V4L2_MEMORY_MMAP;

        if (-1 == xioctl (fd, VIDIOC_REQBUFS, &req)) {
                if (EINVAL == errno) {
                        fprintf (stderr, "%s does not support "
                                 "memory mapping\n", dev_name);
                        exit (EXIT_FAILURE);
                } else {
                        errno_exit ("VIDIOC_REQBUFS");
                }
        }

        if (req.count < 2) {
                fprintf (stderr, "Insufficient buffer memory on %s\n",
                         dev_name);
                exit (EXIT_FAILURE);
        }

        buffers = (buffer *)calloc (req.count, sizeof (*buffers));

        if (!buffers) {
                fprintf (stderr, "Out of memory\n");
                exit (EXIT_FAILURE);
        }

        for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
                struct v4l2_buffer buf;

                CLEAR (buf);

                buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory      = V4L2_MEMORY_MMAP;
                buf.index       = n_buffers;

                if (-1 == xioctl (fd, VIDIOC_QUERYBUF, &buf))
                        errno_exit ("VIDIOC_QUERYBUF");

                buffers[n_buffers].length = buf.length;
                buffers[n_buffers].start =
                        mmap (NULL /* start anywhere */,
                              buf.length,
                              PROT_READ | PROT_WRITE /* required */,
                              MAP_SHARED /* recommended */,
                              fd, buf.m.offset);

                if (MAP_FAILED == buffers[n_buffers].start)
                        errno_exit ("mmap");
        }


}

int CameraQuickCam::set_gamma(__s32 value) {
	return set_control(V4L2_CID_GAMMA, value);
}

int CameraQuickCam::set_gain(__s32 value) {
	return set_control(V4L2_CID_GAIN, value);
}

int CameraQuickCam::set_hue(__s32 value) {
	return set_control(V4L2_CID_HUE, value);
}

int CameraQuickCam::set_saturation(__s32 value) {
	return set_control(V4L2_CID_SATURATION, value);
}

int CameraQuickCam::set_contrast(__s32 value) {
	return set_control(V4L2_CID_CONTRAST, value);
}

int CameraQuickCam::set_brightness(__s32 value) {
	return set_control(V4L2_CID_BRIGHTNESS, value);
}

int CameraQuickCam::set_sharpness(__s32 value) {
	return set_control(V4L2_CID_SHARPNESS, value);
}

int CameraQuickCam::set_auto_white_balance_on() {
	 return set_control((V4L2_CID_PRIVATE_BASE+12), 1);
}

int CameraQuickCam::set_auto_white_balance_off() {
	 return set_control((V4L2_CID_PRIVATE_BASE+12), 0);
}

int CameraQuickCam::set_white_balance(__s32 value) {
	return set_control((V4L2_CID_PRIVATE_BASE+13), value);
}

//bug on / off was exchanged, not my fault :) see luvcview
int CameraQuickCam::set_auto_exposure_off() {
	 return set_control((V4L2_CID_PRIVATE_BASE+10), 1);
}

int CameraQuickCam::set_auto_exposure_on() {
	 return set_control((V4L2_CID_PRIVATE_BASE+10), 8);
}

int CameraQuickCam::set_exposure(__s32 value) {
	return set_control((V4L2_CID_PRIVATE_BASE+11), value);
}


int CameraQuickCam::set_control(__u32 id, __s32 value) {
	struct v4l2_control ctrl; 
	int err;
	
	ctrl.id = id;
	ctrl.value = value;
	if ((err = ioctl(fd, VIDIOC_S_CTRL, &ctrl)) < 0) {
		printf("ioctl control error while set %d to %d\n", ctrl.id, ctrl.value);
	}
	return err;
}

void CameraQuickCam::init_device(){
 
        struct v4l2_capability cap;
        struct v4l2_cropcap cropcap;
        struct v4l2_crop crop;
        struct v4l2_format fmt;
        
        unsigned int min;

        if (-1 == xioctl (fd, VIDIOC_QUERYCAP, &cap)) {
                if (EINVAL == errno) {
                        fprintf (stderr, "%s is no V4L2 device\n",
                                 dev_name);
                        exit (EXIT_FAILURE);
                } else {
                        errno_exit ("VIDIOC_QUERYCAP");
                }
        }

        if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
                fprintf (stderr, "%s is no video capture device\n",
                         dev_name);
                exit (EXIT_FAILURE);
        }


	if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
		fprintf (stderr, "%s does not support streaming i/o\n",
				dev_name);
		exit (EXIT_FAILURE);
	}




        /* Select video input, video standard and tune here. */


        CLEAR (cropcap);

        cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

        if (0 == xioctl (fd, VIDIOC_CROPCAP, &cropcap)) {
                crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                crop.c = cropcap.defrect; /* reset to default */

                if (-1 == xioctl (fd, VIDIOC_S_CROP, &crop)) {
                        switch (errno) {
                        case EINVAL:
                                /* Cropping not supported. */
                                break;
                        default:
                                /* Errors ignored. */
                                break;
                        }
                }
        } else {        
                /* Errors ignored. */
        }


        CLEAR (fmt);

        fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width       = 640; 
        fmt.fmt.pix.height      = 480;
//        fmt.fmt.pix.width       = 320; 
//        fmt.fmt.pix.height      = 240;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
        fmt.fmt.pix.field       = V4L2_FIELD_ANY;

        if (-1 == xioctl (fd, VIDIOC_S_FMT, &fmt))
                errno_exit ("VIDIOC_S_FMT");

	struct v4l2_streamparm* setfps;
	setfps = (struct v4l2_streamparm *) calloc(1, sizeof(struct v4l2_streamparm));
	memset(setfps, 0, sizeof(struct v4l2_streamparm));
	setfps->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	setfps->parm.capture.timeperframe.numerator = 1;
	setfps->parm.capture.timeperframe.denominator = 30;

	if(-1 == xioctl(fd, VIDIOC_S_PARM, setfps))
		errno_exit("VIDIOC_S_PARM");



        /* Note VIDIOC_S_FMT may change width and height. */

        /* Buggy driver paranoia. */
        min = fmt.fmt.pix.width * 2;
        if (fmt.fmt.pix.bytesperline < min)
                fmt.fmt.pix.bytesperline = min;
        min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
        if (fmt.fmt.pix.sizeimage < min)
                fmt.fmt.pix.sizeimage = min;


	


	init_mmap ();


}


void CameraQuickCam::close_device(){

        if (-1 == close (fd))
                errno_exit ("close");

        fd = -1;

}

void CameraQuickCam::open_device()
{
        struct stat st; 

        if (-1 == stat (dev_name, &st)) {
                fprintf (stderr, "Cannot identify '%s': %d, %s\n",
                         dev_name, errno, strerror (errno));
                exit (EXIT_FAILURE);
        }

        if (!S_ISCHR (st.st_mode)) {
                fprintf (stderr, "%s is no device\n", dev_name);
                exit (EXIT_FAILURE);
        }

        fd = open (dev_name, O_RDWR /* required */ | O_NONBLOCK, 0);

        if (-1 == fd) {
                fprintf (stderr, "Cannot open '%s': %d, %s\n",
                         dev_name, errno, strerror (errno));
                exit (EXIT_FAILURE);
        }
}

void CameraQuickCam::captureBegin(){


	for(;;){
		fd_set fds;
		struct timeval tv;
		int r;
		
		FD_ZERO (&fds);
		FD_SET (fd, &fds);
		
		/* Timeout. */
		tv.tv_sec = 5;
		tv.tv_usec = 0;
		
		r = select (fd + 1, &fds, NULL, NULL, &tv);
		
		if (-1 == r) {
			if (EINTR == errno)
				continue;
		
			errno_exit ("select");
		}
		
		if (0 == r) {
			fprintf (stderr, "select timeout\n");
			exit (EXIT_FAILURE);
		}
		
		if (read_frame ())
			break;
	}

}


void CameraQuickCam::captureEnd(){



}

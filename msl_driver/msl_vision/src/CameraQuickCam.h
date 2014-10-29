/*
 * $Id: CameraQuickCam.h 1531 2006-08-01 21:36:57Z phbaer $
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
#ifndef CameraQuickCam_H
#define CameraQuickCam_H


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

#include <SystemConfig.h>

#define CLEAR(x) memset (&(x), 0, sizeof (x))

#define V4L2_CID_SHARPNESS			(V4L2_CID_PRIVATE_BASE+2)

using namespace castor;

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


class CameraQuickCam
{
	public:

		CameraQuickCam();
		~CameraQuickCam();
		char * getCaptureBuffer();
		void captureBegin();
		void captureEnd();
		int set_control(__u32 id, __s32 value);
		int set_gamma(__s32 value); 
		int set_gain(__s32 value);
		int set_hue(__s32 value);
		int set_exposure(__s32 value);
		int set_saturation(__s32 value);

		int set_contrast(__s32 value);
		int set_brightness(__s32 value);
		int set_sharpness(__s32 value);

		int set_auto_white_balance_on(); 
		int set_auto_white_balance_off(); 
		int set_white_balance(__s32 value);
		int set_auto_exposure_on();
		int set_auto_exposure_off();


	protected:

		SystemConfigPtr sc;

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

		char * dev_name;
		io_method io;
		int fd;
		struct buffer * buffers;
		unsigned int n_buffers;
		int id;

		char * captureBuffer;
		
};



#endif


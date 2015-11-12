/*
 * StatusCode.h
 *
 *  Created on: Nov 11, 2015
 *      Author: Stephan Opfer
 */

#ifndef INCLUDE_STATUSCODE_H_
#define INCLUDE_STATUSCODE_H_
namespace msl_driver
{
	enum StatusCode
		: uint
		{
			Undefined = 0x00000000,

			Ok = 0x00000001, Error = 0x80000001,

			Initialize = 0x00000002, ErrorInitialize = 0x80000002,

			Open = 0x00000003, ErrorOpen = 0x8000003,

			Shutdown = 0x00000004, ErrorShutdown = 0x80000004,

			ExecutingRequest = 0x00000005, ErrorExecutingRequest = 0x80000005,

			ExecutingCheck = 0x00000006, ErrorExecutingCheck = 0x80000006,
	};

}

#endif /* INCLUDE_STATUSCODE_H_ */

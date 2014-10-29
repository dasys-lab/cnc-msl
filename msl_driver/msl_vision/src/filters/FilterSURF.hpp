#ifndef FILTERSURF_H
#define FILTERSURF_H

#include <stdio.h>
#include <stdint.h>
#include <iostream>

#include "../global/Types.h"
#include "../helpers/ScanLineHelper3D.hpp"

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

class FilterSURF
{
	public:
		FilterSURF(ImageSize size, ScanLineHelper3D &helper);
		~FilterSURF();
		
		void process(unsigned char * inner_, unsigned char * outer_, ImageSize innerSize, ImageSize outerSize, ScanLineHelper3D &helper);
	
	protected:
		uint16_t width;
		uint16_t height;
		uint16_t mx;
		uint16_t my;
		
		Mat innerMask;
		Mat outerMask;
		Mat target;
		Mat mapx;
		Mat mapy;
};

#endif // FILTERSURF_H

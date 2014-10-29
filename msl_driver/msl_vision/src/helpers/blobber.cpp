/*
 * blobber.cpp
 * 
 * This is searching for matching blobs in the inner 
 * and outer panoramic image of the 3D Vision-System
 * 
 * Author: Tim Schlüter
 * under: University Kassel - FB16 - Verteilte Systeme
 * 
 */

using namespace std;

#include "blobber.hpp"
#include <boost/config/no_tr1/complex.hpp>

Blobber::Blobber(ImageSize innerSize, ImageSize outerSize)
{
	width		= innerSize.width;
	height		= innerSize.height+outerSize.height;
	innerHeight	= innerSize.height;
	outerHeight	= outerSize.height;

	blobSize		= 1;
	trash		= 20;
	trashBorder	= 3;

	blob		= new unsigned char [(2*blobSize+1)*(2*blobSize+1)];
	matches	= new uint16_t [2*height];

	image3D	= new unsigned char [width*outerHeight];

	// Setup display
	pannoDisplay	= new XVDisplay(width, innerHeight+outerHeight, XV_UYVY);
	display3D		= new XVDisplay(width, outerHeight, XV_UYVY);

	// Setup Gnuplot
// 	plot.set_title("3D-PointCloud");
// 	plot.set_xlabel("Rotation");
// 	plot.set_ylabel("Entfernung");
// 	plot.set_zlabel("Höhe"); 
// 	plot.set_yrange(0, 5000);
// 	plot.set_zrange(-5000, 5000);
}


void Blobber::process(unsigned char * panno, Distance3DHelper distance3DHelper)
{
	X.clear();
	Y.clear();
	Z.clear();
	// Over each ScanLine
	for(uint16_t x=blobSize; x<width-blobSize; x++)
	{
		// Over each Point in the outer.
		//for(uint16_t yI=blobSize; yI<innerHeight-blobSize; yI++)
		for(uint16_t yO=blobSize+innerHeight; yO<height-blobSize; yO++)
		{
			// Rest blob pointer.
			unsigned char * blobPtr = blob;
			
			// Store "colors" of the outer in the blob.
			for(int8_t u=-blobSize; u<=blobSize; u++)
			{
				for(int8_t v=-blobSize; v<=blobSize; v++)
				{
					*(blobPtr++) = panno[x+u+(yO+v)*width];
				}
			}
			
			/// Check for the blob in the outer.
			uint16_t matchCounter	= 0;
			uint16_t * matchPointer	= matches;
			// Over each point in the inner.
			//for(uint16_t yO=blobSize+innerHeight; yO<height-blobSize; yO++)
			for(uint16_t yI=blobSize; yI<innerHeight-blobSize; yI++)
			{
				uint16_t trashCounter = 0;
				
				// Reset pointer to the start of the blob.
				blobPtr = blob;
				
				// Check for differences to the blob
				for(int8_t u=-blobSize; u<=blobSize; u++)
				{
					for(int8_t v=-blobSize; v<=blobSize; v++)
					{
						int16_t diff = *(blobPtr++) - panno[x+u+(yI+v)*width];
						if(diff>trash || diff<-trash)
							trashCounter++;  
					}
				}
				
				// Check for good match
				if(trashCounter<trashBorder)
					matchPointer[matchCounter++] = yI;
			}
			// Workout
			if(matchCounter==1)
			{
				// Matching points: Inner -> y; Outer -> x,*matchPointer
				//image3D[x+yI*width]	= round(distance3DHelper.getDistance(yI, *matchPointer).x*255/2000);
				image3D[x+(yO-innerHeight)*width] = round(distance3DHelper.getDistance(*matchPointer, yO).x*255/2000);
				
				X.push_back(x);
				Y.push_back(distance3DHelper.getDistance(*matchPointer, yO).x);
				Z.push_back(distance3DHelper.getDistance(*matchPointer, yO).y);
			}
			else if(matchCounter>1)
			{
				//image3D[x+y*width]	= 200;
			}
		}
	}
	
	pannoDisplay->displayFrameGRAY(panno);
	display3D->displayFrameGRAY(image3D);
	
// 	plot.plot_xyz(X, Y, Z, "Cloud");
// 	plot.reset_plot();
// 	 plot.remove_tmpfiles();
}


Blobber::~Blobber()
{
	if(blob != NULL)	delete blob;
	if(matches != NULL)	delete matches;
	if(image3D != NULL)	delete image3D;
	if(pannoDisplay != NULL)	delete pannoDisplay;
	if(display3D != NULL)	delete display3D;
}


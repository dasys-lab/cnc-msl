/*
 * $Id: VisionPlayer3D.cpp 2214 2013-05-07 13:00:39Z cn $
 *
 *
 * Copyright 2013 Carpe Noctem, Distributed Systems Group,
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

#include "VisionPlayer3D.hpp"


using std::cout;
using std::endl;
using std::string;

/*
**  Variables
*/
bool color		= false;
bool datalog		= false;
bool debug		= false;
bool drawBorders	= false;
bool field		= false;
bool fixed_image	= false;
bool gray			= false;
bool help			= false;
bool loadRef		= false;
bool logFlag		= false;
bool online		= true;
bool panno		= false;
bool plot2d		= false;
bool plot3d		= false;
bool printLines	= false;
bool printScanLines = false;
bool rgb			= false;
bool simulation	= false;
bool ssh			= false;

uint16_t dir = 400;
uint16_t fixed_number = 1;
uint16_t frameCounter = 0;
uint16_t writeCounter = 0;

// Offline image
FILE * logfile;

// Initialize the display.
XVDisplay * xvDisplayGray	= NULL;
// XVDisplay * xvDisplayRGB		= NULL;
XVDisplay * xvDisplayYUV		= NULL;
XVDisplay * xvDisplayPanno	= NULL;

// Image variables
uint16_t width			= 0;
uint16_t height		= 0;
int16_t imageOffsetX	= 0;
int16_t imageOffsetY	= 0;
uint16_t imageCenterX	= 0;
uint16_t imageCenterY	= 0;
uint16_t innerRadiusStart	= 0;
uint16_t innerRadiusEnd	= 0;
uint16_t outerRadiusEnd	= 0;
uint16_t outerRadiusStart	= 0;
unsigned char * img_		= NULL;
unsigned char * imageRGB		= NULL;
unsigned char * imageGray	= NULL;
unsigned char * imageUV		= NULL;
unsigned char * imageLine	= NULL;
unsigned char * offlineImg	= NULL;
unsigned char * imagePanno		= NULL;
unsigned char * innerPanno	= NULL;
unsigned char * outerPanno	= NULL;
unsigned char * pannoLine	= NULL;

struct ImageSize
	innerSize,
	outerSize,
	pannoSize;

// Time variables
struct timeval tvBefore;
struct timeval tvAfter;
long timediff = 0;
long tvAverage = 0;
long tvCounter = 0;

ros::Subscriber sub;

FilterExtractImages		*extractImages;
FilterExtractPanno		*extractPanno;
FilterExtractLineImage	*extractLineImage;
FilterExtractLineImage	*extractPannoLineImage;
FilterLinePoints3D		*linePoints3D;
Draw 				*draw;


/**
*** Function declarations
**/
void	checkArg		(int &argc, char **&argv);	// Check arguments, set flags and print help.
void	vision		(const sensor_msgs::Image::ConstPtr &img);
void	initial		(void);	// Initial
void	initial_camera	(void);	// Initial camera
void	readImage		(unsigned char *&output);	// Read a new frame.
void	logImage		(unsigned char *&image);	// Log images for offline mode
void	algo			(unsigned char *&image_);
void release		(void);


/**
*** Functions
**/
int main(int argc,char **argv)
{
	// Check the arguments, set the flags and print the help.
	checkArg(argc, argv);

	// Initialize classes
	if(panno)
	{
		extractPanno			= new FilterExtractPanno(innerSize, outerSize, pannoSize);
		extractPannoLineImage	= new FilterExtractLineImage(pannoSize);
	}
	else
	{
		extractImages		= new FilterExtractImages;
		extractLineImage	= new FilterExtractLineImage;
	}
	linePoints3D	= new FilterLinePoints3D;
	draw			= new Draw;

// 	ros::init(argc, argv, "CNVision3D");
	SpicaHelper::initialize();

	FILE *launch;

	initial();

	if(online)
	{
		if(color)
			launch = popen("roslaunch CNVision driver_color.launch", "w");
		else
			launch = popen("roslaunch CNVision driver.launch", "w");
		// Subscribe to scan topic.
		if(color)
			sub = SpicaHelper::visionNode->subscribe("camera/image_rect_color", 10, vision);
		else
			sub = SpicaHelper::visionNode->subscribe("camera/image_rect", 10, vision);

		// Start the loop.
		while(ros::ok())
		{
			ros::spinOnce();
		}
	}
	else
	{
		// Allocate Memory for offline image.
		if(color)
			offlineImg = new unsigned char [width*height*2];
		else
			offlineImg = new unsigned char [width*height];

		// Start loop.
		while(ros::ok())
		{
			// Read image.
			readImage(offlineImg);

			// Start algorithm.
			algo(offlineImg);
		}

		// Deallocate Memory.
		if(offlineImg != NULL)	delete[] offlineImg;
	}

	printf("\n\nRos is not OK!!!\n\n");

	if(launch != NULL)	pclose(launch);

	release();
}


void vision(const sensor_msgs::Image::ConstPtr &img)
{
	printf("Encoding format: %s\n\n", img->encoding.c_str());
	uint32_t counter = 0;

	if(color)
	{
		// Convert RGB to YUV422
		for(uint16_t i=0; i<img->width; i++)
		{
			for(uint16_t j=0; j<img->height; j++)
			{
				uint8_t R, G, B;
				uint32_t index = counter*3;

				B = img->data.data()[index];
				G = img->data.data()[index+1];
				R = img->data.data()[index+2];

				index = counter*2;
				// Y-Value
				img_[index+1] = ((66*R +129*G +25*B +128) >> 8) +16;
				// UV-Value
				if(counter%2) // U-Value
					img_[index] = ((-38*R -74*G +112*B +128) >> 8) +128;
				else	// V-Value
					img_[index] = ((112*R -94*G -18*B +128) >> 9) +128;
				counter++;
			}
		}
	}
	else
	{
		for(uint16_t i=0; i<img->width; i++)
		{
			for(uint16_t j=0; j<img->height; j++)
			{
				img_[counter] = img->data.data()[counter];
				counter++;
			}
		}
	}

	algo(img_);
}


void algo(unsigned char * &rawFrame)
{
	/// Rise FrameCoutner.
	frameCounter++;


	/// Get time
	gettimeofday(&tvBefore, NULL);


	/// Extrat UV and gray image
	cout << "DEBUG filterExtractImages" << endl;
	if(color)
		extractImages->process(rawFrame, imageGray, imageUV);
	else
		imageGray = rawFrame;

	if(panno)
	{
		/// Extract gray Panno image
		extractPanno->process(imageGray, innerPanno, outerPanno, imagePanno);
		/// Extract Panno Line images
		bool temp = color;	// HACK
		color = false;		// HACK
		extractPannoLineImage->process(imagePanno, pannoLine);
		color = temp;		// HACK

		linePoints3D->panno(imagePanno, pannoLine, innerSize, outerSize);
		xvDisplayPanno->displayFrameGRAY(imagePanno);
	}

	/// Log images for offline mode
	if(logFlag)
	{
// 		logImage(imageGray);
		if( (frameCounter >= 200) && !(frameCounter%10) )
			logImage(rawFrame);
		return;
	}


	/// Draw Scan Line
	if(printScanLines)
	{
		cout << "DEBUG draw.Line" << endl;
		draw->ScanLine(imageGray);
	}


	/// Extract lines out of YUV image
	cout << "DEBUG filterExtractLineImage" << endl;
	extractLineImage->process(rawFrame, imageLine);


	/// Filter 3D Line Points
//	if(!ssh || (ssh && !(frameCounter%100)))
	if(!panno)
	{
		linePoints3D->process(imageGray, imageLine);
	}


	/// Draw mirror borders
	if(drawBorders)
	{
		struct Circle c1, c2, c3, c4;
// 		c1.radius = innerRadiusStart;
		c1.radius = 519;	// Upper mirror radius.
		c1.x = imageOffsetX;
		c1.y = imageOffsetY;
		c2.radius = innerRadiusEnd;
		c2.x = imageOffsetX;
		c2.y = imageOffsetY;
		c3.radius = outerRadiusEnd;
		c3.x = imageOffsetX;
		c3.y = imageOffsetY;
		c4.radius = outerRadiusStart;
		c4.x = imageOffsetX;
		c4.y = imageOffsetY;
		cout << "DEBUG draw.circle" << endl;
		draw->Circle(imageGray, c1);
		draw->Circle(imageGray, c2);
		draw->Circle(imageGray, c3);
		draw->Circle(imageGray, c4);
	}

	/// Center Marker
	cout << "DEBUG Center marker" << endl;
	imageGray[imageCenterX+imageCenterY*width] = 128;
	imageGray[imageCenterX+imageCenterY*width-1] = 128;
	imageGray[imageCenterX+imageCenterY*width-width] = 128;
	imageGray[imageCenterX+imageCenterY*width-width-1] = 128;


	/// Get titan braces
// 	filterTitanBraces.process(imageGray);


	/// Timemeasure
	gettimeofday(&tvAfter, NULL);
	timediff = tvAfter.tv_usec - tvBefore.tv_usec;
	if(timediff < 0)	timediff += 1000000;
	printf("\n\nTime for FilterChain: %ld\n", timediff);
	tvAverage += timediff;
	tvCounter++;
	cout << "TimeAverage " << tvAverage << "\n";
	cout << "TimeCounter " << tvCounter << "\n";
	timediff = 1000000/30-timediff;
	if(timediff>0 && !online)
	{
		printf("Sleep for %ld microseconds\n\n", timediff);
		usleep(timediff);
	}

	/// Display Images
	printf("Stage 14: Show Images\n");
	if((xvDisplayGray != NULL) && (!ssh || (ssh && frameCounter%33==0)))// <-- for ssh
	{
		if( printLines )
			xvDisplayGray->displayFrameGRAY(imageLine);
		else if (gray)
		{
// 			if(loadRef)
// 			{
// 				for(uint32_t i=0; i<(width*height); i++)
// 					imageGray[i] = (imageGray[i] - imageRef[i] + 255) / 2;
// 			}
			xvDisplayGray->displayFrameGRAY(imageGray);
		}
	}
	if( rgb && (xvDisplayYUV != NULL) && (!ssh || (ssh && frameCounter%33==0)))
	{
 		xvDisplayYUV->displayFrameYUV(rawFrame);
	}

	// Pause offline player
// 	if(!online)
// 		usleep(15000);

	if( fixed_image )
	{
		usleep(100);
	}
}


/**
*** Check arguments, set flags and print help.
**/
void checkArg (int &argc, char **&argv)
{
	if(argc > 1)
	{
		for(int i = 1; i < argc; i++)
		{
			if(string(argv[i]) == "--color")
			{	color = true;	continue;	}
			if(string(argv[i]) == "--rgb" && color)
			{	rgb = true;	continue;	}
			if(string(argv[i]) == "--datalog")
			{	datalog = true;	continue;	}
			if(string(argv[i]) == "--drawBorders")
			{	drawBorders = true;	continue;	}
			if(string(argv[i]) == "--offline")
			{	online = false;	continue;	}
			if(string(argv[i]) == "--help")
			{	help = true;	continue;	}
			if(string(argv[i]) == "--field")
			{	field = true;	continue;	}
			if(std::string(argv[i]) == "--log")
			{	logFlag = true;	continue;	}
			if(string(argv[i]) == "--fixed")
			{
				if(i<argc)	fixed_number = atoi(argv[++i]);
				else 		continue;
				fixed_image = true;
				continue;
			}
			if(string(argv[i]) == "--begin")
			{
				if(i<argc)	fixed_number = atoi(argv[++i]);
				continue;
			}
			if(string(argv[i]) == "--dir")
			{	if(i<argc)	dir = atoi(argv[++i]);	continue;	}
			if(string(argv[i]) == "--panno")
			{	panno = true;	continue;	}
			if(string(argv[i]) == "--plot2d")
			{	plot2d = true;	continue;	}
			else if(string(argv[i]) == "--plot3d")
			{	plot3d = true;	continue;	}
			if(string(argv[i]) == "--lineImage")
			{	printLines = true;	continue;	}
			if(string(argv[i]) == "--ref")
			{	loadRef = true;	continue;	}
			if(string(argv[i]) == "--scanlines")
			{	printScanLines = true;	continue;	}
			if(string(argv[i]) == "--sim")
			{	simulation = true;	continue;	}
			if(string(argv[i]) == "--ssh")
			{	ssh = true;	continue;	}
			if(string(argv[i]) == "--gray")
			{	gray = true;	continue;	}
			cout << "Something wrong with your argument " << argv[i] << endl;
		}
	}

	if(help)
	{
		printf("\nOptions for VisionPlayer3D:\n\n");
		printf("--begin [imageNumber]:\t Start the [imageNumber] image from the $VISION_LOG.\n");
		printf("--color:\t Use color image instead of gray images as input.\n");
		printf("--fixed [imageNumber]:\t only the image [imageNumber] from the $VISION_LOG is used.\n");
		printf("--gray:\t Print the gray image.\n");
		printf("--help:\t Shows this help text.\n");
		printf("--lineImage:\t Plot line Image, only with --gray\n");
		printf("--log:\t Log image to $VISION_LOG.\n");
		printf("--plot2d:\t Plot 2D line points.\n");
		printf("--plot3d:\t Plot 3D line points.\n");
		printf("--offline:\t offline mode using images in $VISION_LOG.\n");
		printf("--rgb:\t Print the rgb image; Only works with \"--color\", which have to be decleared before.\n");
		printf("--scanlines:\t Print the scan lines.\n");
		printf("--ssh:\t\t Less image publishing.\n");

		exit(EXIT_SUCCESS);
	}
}


/*
** Initialisation
*/
void initial (void)
{
	// Initialize the vision config files.
	supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
	supplementary::Configuration * vision3D = (*sc)["Vision3D"];

	cout << "Image";
	cout << "\tWidth_";
	width	= vision3D->get<uint16_t>("Image", "Width", NULL);
	cout << "\tHeight_";
	height	= vision3D->get<uint16_t>("Image", "Height", NULL);
	cout << "\tOffsetX_";
	imageOffsetX = vision3D->get<int16_t>("Image", "Offset_X", NULL);
	cout << "\tOffsetY_";
	imageOffsetY = vision3D->get<int16_t>("Image", "Offset_Y", NULL);

	imageCenterX = width/2 + imageOffsetX;
	imageCenterY = height/2 + imageOffsetY;

	cout << "ScanLines" << endl;
	cout << "\tInnerRadiusStart_";
	innerRadiusStart	= vision3D->get<uint16_t>("ScanLines", "InnerRadiusStart", NULL);
	cout << "\tInnerRadiusEnd_";
	innerRadiusEnd		= vision3D->get<uint16_t>("ScanLines", "InnerRadiusEnd", NULL);
	cout << "\tOuterRadiusEnd_";
	outerRadiusEnd		= vision3D->get<uint16_t>("ScanLines", "OuterRadiusEnd", NULL);
	cout << "\tOuterRadiusStart_";
	outerRadiusStart	= vision3D->get<uint16_t>("ScanLines", "OuterRadiusStart", NULL);

	// Allocate memory for images.
	if(color)
		img_	= new unsigned char [width*height*2];
	else
		img_	= new unsigned char [width*height];

	// Allocate memory for displays.
	xvDisplayGray	= new XVDisplay(width, height, XV_UYVY);
//	xvDisplayRGB	= new XVDisplay(width, height, XV_UYVY);
 	xvDisplayYUV	= new XVDisplay(width, height, XV_UYVY);
	xvDisplayPanno = new XVDisplay(pannoSize.width, pannoSize.height, XV_UYVY);
}


void release(void )
{
	if(extractLineImage != NULL)		delete extractLineImage;
	if(extractPannoLineImage != NULL)	delete extractPannoLineImage;
	if(linePoints3D != NULL)			delete linePoints3D;
	if(xvDisplayGray != NULL)		delete xvDisplayGray;
	if(xvDisplayYUV != NULL)			delete xvDisplayYUV;
	if(xvDisplayPanno != NULL)		delete xvDisplayPanno;
	if(img_ != NULL)				delete[] img_;
}




/*
** Read a new frame for the process.
*/
void readImage( unsigned char *&output )
{
	static int16_t imCounter = fixed_number;

	/* Get filename and path */
	char filename[256];
	sprintf(filename, "/log-image-%04d.raw", imCounter);
	char path[256];
	strcpy(path, getenv("VISION_LOG"));
	char * path_filename = strcat(path, filename);


	printf("Processing LogFile: %s\n", path_filename);

	logfile = fopen(path_filename, "r");

	if(logfile != NULL)
	{
		if(color)
			fread(output, sizeof(char),(uint32_t)(width * height * 2), logfile);
		else
			fread(output, sizeof(char),(uint32_t)(width * height), logfile);
		fclose(logfile);
	}
	else
	{
		printf("Log file not found ... Restarting\n");

		/* Reset counter */
		imCounter = 1;

		/* Get filename and path */
		char filename[256];
		sprintf(filename, "/log-image-%04d.raw", imCounter);
		char path[256];
		strcpy(path, getenv("VISION_LOG"));
		char * path_filename = strcat(path, filename);

		/* Open logfile */
		logfile = fopen(path_filename, "r");

		/* Check logfile */
		if(logfile != NULL)
		{
			if(color)
				fread(output, sizeof(char), (uint32_t)(width * height * 2), logfile);
			else
				fread(output, sizeof(char), (uint32_t)(width * height), logfile);
			fclose(logfile);
		}
		else
		{
			printf("Log file not found: ERRNO: %d\n", errno);
			perror("Test");
			exit(EXIT_FAILURE);
		}
	}
	/* Rise counter */
	if(!fixed_image)
		imCounter++;
}


/// Logging image for offline
void logImage( unsigned char * &image )
{
	char filename[256];
	char path[256];

	strcpy(path, getenv("VISION_LOG"));
	sprintf(filename, "/log-image-%04d.raw", writeCounter);
	char * path_filename = strcat(path, filename);

	FILE * logfile = fopen(path_filename, "w");

	if(color)
		fwrite(image, sizeof(char), width*height*2, logfile);
	else
		fwrite(image, sizeof(char), width*height, logfile);

	fclose(logfile);

	writeCounter++;
}



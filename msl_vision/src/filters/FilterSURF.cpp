/*
 * $Id: BallHelper.cpp 1531 2006-08-01 21:36:57Z phbaer $
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
 * This source is searching for corelating features in to images
 */


#include "FilterSURF.hpp"

FilterSURF::FilterSURF(ImageSize size, ScanLineHelper3D &helper)
{
	uint16_t	iRadiusStart	= helper.getInnerRadiusStart();
	uint16_t	iRadiusEnd	= helper.getInnerRadiusEnd();
	uint16_t	oRadiusStart	= helper.getOuterRadiusStart();
	uint16_t	oRadiusEnd	= helper.getOuterRadiusEnd();
	
	width	= size.width;
	height	= size.height;
	mx	= width/2;
	my	= height/2;
	
	innerMask.create(height, width, CV_8UC1);
	outerMask.create(height, width, CV_8UC1);
	
	uint32_t	temp;
	
	for(int16_t i=-width/2; i<width/2; i++)
	{
	  for(int16_t j=-height/2; j<height/2; j++)
	  {
	    temp 	= i*i+j*j; 
	    
	    // Inner Mask
	    if( temp < iRadiusEnd*iRadiusEnd && 
		temp > iRadiusStart*iRadiusStart )
	    {
		innerMask.at<uchar>(j+height/2,i+width/2) = 255;
	    }
	    else
	    {
		innerMask.at<uchar>(j+height/2,i+width/2) = 0;
	    }
	    
	    // Outer mask
	    if( temp > oRadiusEnd*oRadiusEnd &&
		temp < oRadiusStart*oRadiusStart )
	    {
		outerMask.at<uchar>(j+height/2,i+width/2) = 255;
	    }
	    else
	    {
		outerMask.at<uchar>(j+height/2,i+width/2) = 0;
	    }
	  }
	}
}



void FilterSURF::process(unsigned char * inner_, unsigned char * outer_, ImageSize innerSize, ImageSize outerSize, ScanLineHelper3D &helper)
{
	Mat inner;
	Mat outer;
	
	inner.create(innerSize.height, innerSize.width, CV_8UC1);
	outer.create(outerSize.height, outerSize.width, CV_8UC1);
	inner.data = inner_;
	outer.data = outer_;
	inner.copyTo(target);
	target.push_back(outer);
	
	
	//-- Step 1: Detect the keypoints using SURF Detector
	int minHessian = 400;
	std::vector<KeyPoint> keypoints_inner, keypoints_outer;
	SurfFeatureDetector detector(minHessian);
	detector.detect(inner, keypoints_inner);
	detector.detect(outer, keypoints_outer);
	
	//-- Step 2: Calculate descriptors (feature vectors)
	SurfDescriptorExtractor extractor;
	Mat descriptors_inner, descriptors_outer;
	extractor.compute( inner, keypoints_inner, descriptors_inner );
	extractor.compute( outer, keypoints_outer, descriptors_outer );
	
	//-- Step 3: Matching descriptor vectors using FLANN matcher
	FlannBasedMatcher matcher;
	std::vector<DMatch> matches;
	matcher.match( descriptors_inner, descriptors_outer, matches );
	
	
	//-- Quick calculation of max and min distances between keypoints
	double max_dist = 0; double min_dist = 100;
	for( int i = 0; i < matches.size(); i++ )
	{ 
		double dist = matches[i].distance;
		if( dist < min_dist ) min_dist = dist;
		if( dist > max_dist ) max_dist = dist;
	}
	printf("-- Max dist : %f \n", max_dist );
	printf("-- Min dist : %f \n", min_dist );
/*	
	//-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
	std::vector< DMatch > good_matches;
	for( int i = 0; i < matches.size(); i++ )
	{
	  if( matches[i].distance < 1.5 *min_dist )
	  {
		good_matches.push_back( matches[i]);
	  }
	}
*/	 
	for( int i=0; i<matches.size(); i++ )
	{
	  //keypoints_scene[(matches[i].trainIdx)].pt.y+=innerSize.height*outerSize.width;
//	  line(target, keypoints_inner[(matches[i].queryIdx)].pt, keypoints_outer[(matches[i].trainIdx)].pt, 255, 1, 8, 0);
	}
	
	//for(int i=0; i<100; i++)
	  //printf("-------%d\n", keypoints_object[i].pt);
	
//	Mat img_matches;
	drawKeypoints( inner, keypoints_inner, inner, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
	drawKeypoints( outer, keypoints_outer, outer, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
	
//	imshow("Mixed", target);
	imshow("Inner", inner);
	imshow("Outer", outer);
	
	cvWaitKey(10);
	
// 	drawMatches( inner, keypoints_object, outer, keypoints_scene,
// 		     matches, img_matches, Scalar::all(-1), Scalar::all(-1),
// 		     vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
// 	imshow("Matches", img_matches);
/*	
	//-- Localize the object
	std::vector<Point2f> obj;
	std::vector<Point2f> scene;
	
	for( int i = 0; i < good_matches.size(); i++ )
	{
		//-- Get the keypoints from the good matches
		obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
		scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
	}
	
	Mat H = findHomography( obj, scene, CV_RANSAC );
	
	//-- Get the corners from the image_1 ( the object to be "detected" )
	std::vector<Point2f> obj_corners(4);
	obj_corners[0] = cvPoint(0,0);
	obj_corners[1] = cvPoint( image.cols, 0 );
	obj_corners[2] = cvPoint( image.cols, image.rows );
	obj_corners[3] = cvPoint( 0, image.rows );
	std::vector<Point2f> scene_corners(4);
	
	perspectiveTransform( obj_corners, scene_corners, H);
*/	//-- Draw lines between the corners (the mapped object in the scene - image_2 )
	//-- Show detected matches
}


FilterSURF::~FilterSURF()
{
	destroyWindow("Good Matches & Object detection");
}




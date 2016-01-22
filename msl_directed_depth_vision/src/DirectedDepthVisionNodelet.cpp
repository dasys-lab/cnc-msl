/*
 * DirectedDepthVisionNodelet.cpp
 *
 *  Created on: 01.09.2015
 *      Author: tobi
 */

#include "msl_directed_depth_vision/DirectedDepthVisionNodelet.h"

//rosrun nodelet nodelet load msl_directed_depth_vision/DirectedDepthVisit /camera/camera_nodelet_manager __name:=directedDepthVision

namespace msl_vision
{
	DirectedDepthVisionNodelet::DirectedDepthVisionNodelet() {
		sc = supplementary::SystemConfig::getInstance();
		depthVision = (*sc)["DirectedDepthVision"];

		voxelSize = depthVision->get<double>("DepthVision", "ObstacleDetection", "voxelSize", NULL);
		maxFloorDist = depthVision->get<double>("DepthVision", "ObstacleDetection", "maxFloorDist", NULL);
		floorPlaneFilename = depthVision->get<string>("DepthVision", "FloorPlaneFile", "filename", NULL);

		vg =  new pcl_filters::FilterVoxelGrid();

		viewer = new pcl::visualization::CloudViewer("DirectedDepth PCL");
		cloud_in = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >();
		cloud_in->points.resize(640 * 480);
		cloud_voxel = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >();
		cloud_voxel->points.resize(640 * 480);
		cloud_temp = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >();
		cloud_temp->points.resize(640 * 480);
		cloud_cluster = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >();
		cloud_ground = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >();

	}
	DirectedDepthVisionNodelet::~DirectedDepthVisionNodelet() {
		delete (viewer);
	}
	void DirectedDepthVisionNodelet::onInit() {
		nh = getNodeHandle();

		std::vector<std::string> argv = getMyArgv();

		for (int i = 0; i < argv.size(); i++) {
			if (string(argv[i]) == "-calibrate") {
				calibrateFloor = true;
			}
			if (string(argv[i]) == "-debug") {
				debugDisplay = true;
			}
		}
		if(!calibrateFloor) {
			loadPlane();
		}
		pclSub = nh.subscribe("/camera/depth_registered/points", 10, &DirectedDepthVisionNodelet::PointCloudCallback, this);
	}
	void DirectedDepthVisionNodelet::PointCloudCallback(sensor_msgs::PointCloud2Ptr msg) {
		if (calibrateFloor) {
			calibrateFloor = false;
			floorCalib.calibrate(msg);
			loadPlane();
		}


		//downsample cloud_in into voxel grid and store it in cloud_voxel
		//cerr << "", tt.tic();
		pcl::fromROSMsg(*msg, *cloud_in);

		vg->processVoxelGridFilter(cloud_in, cloud_voxel, voxelSize, false);
		//cerr << " >> " <<tt.toc() << "ms" << endl;

		//color the segmented ground green and the rest red
		int clusterPixel = 0;
		int planePixel = 0;
		for(int i = 0; i < cloud_voxel->points.size(); i++) {
			double cx = cloud_voxel->points[i].x, cy = cloud_voxel->points[i].y, cz = cloud_voxel->points[i].z;
			double e = (a*cx+b*cy+c*cz+d) / (sqrt(a*a+b*b+c*c));

			if(e > maxFloorDist) {
				//every voxel cell except ground so the cluster cells
				clusterPixel++; // size of the cluster cloud
				cloud_voxel->points[i].r = 255; //
				cloud_voxel->points[i].g = 0; //
				cloud_voxel->points[i].b = 0; //
			} else {
				//every voxel cell in the ground
				planePixel++;
				cloud_voxel->points[i].r = 0; //
				cloud_voxel->points[i].g = 255; //
				cloud_voxel->points[i].b = 0; //
			}
		}

		//resize the cluster cloud to the exact size
		cloud_cluster->points.resize(clusterPixel);
		cloud_ground->points.resize(planePixel);

		//iterate over the voxel cloud and store the clusters segmented from ground to the cluster cloud
		for(int i = 0, j = 0, k = 0; i < cloud_voxel->points.size(); i++) {
			double cx = cloud_voxel->points[i].x, cy = cloud_voxel->points[i].y, cz = cloud_voxel->points[i].z;
			double e = (a*cx+b*cy+c*cz+d) / (sqrt(a*a+b*b+c*c));
			if(e > maxFloorDist) {
			//write every voxel cell segented from ground to cluster cloud (white color)
				cloud_cluster->points[j].r = 255;
				cloud_cluster->points[j].g = 0;
				cloud_cluster->points[j].b = 0;
				cloud_cluster->points[j].x = cloud_voxel->points[i].x;
				cloud_cluster->points[j].y = cloud_voxel->points[i].y;
				cloud_cluster->points[j++].z = cloud_voxel->points[i].z;
			} else {
				cloud_ground->points[k].r = 0;
				cloud_ground->points[k].g = 255;
				cloud_ground->points[k].b = 0;
				cloud_ground->points[k].x = cloud_voxel->points[i].x;
				cloud_ground->points[k].y = cloud_voxel->points[i].y;
				cloud_ground->points[k++].z =cloud_voxel->points[i].z;
			}
		}

		cout << "cloud_voxel,cloud_cluster,cloud_ground: " << cloud_voxel->size() << " " << cloud_cluster->size() << " " << cloud_ground->size() << endl;
		cerr << "Segmenting to clusters...\n", tt.tic();
		cerr << "relevant points: " << cloud_cluster->points.size() << endl;
		pcl::IndicesClustersPtr clusters (new pcl::IndicesClusters);
		pcl::ConditionalEuclideanClustering<pcl::PointXYZRGB> cec(true);
		cec.setInputCloud(cloud_cluster);
		cec.setConditionFunction(&DirectedDepthVisionNodelet::customRegionGrowing);
		cec.setClusterTolerance(0.1);
		cec.setMinClusterSize(30);
		cec.setMaxClusterSize(cloud_cluster->points.size() / 2);
		cec.segment(*clusters);
		cerr << " >> " << tt.toc() << "ms " << "clusters: " << clusters->size() << endl;

		cerr << "TRACKING CLUSTER...\n", tt.tic();
		clusterTracker.process(cloud_cluster, clusters);
		clusterTracker.includeCluster(cloud_voxel, cloud_cluster); // for visualizing
		cerr << " >> " << tt.toc() << "ms " << endl;

//		viewer->showCloud(cloud_ground, "ground"); //
		viewer->showCloud(cloud_in, "ground"); //
		double max = INT_MIN,avg=0,min = INT_MAX, count=1;
		for(int i = 0; i < cloud_in->size(); i++) {
			double z = cloud_in->points[i].z;
			if(z >= max) {
				max = z;
			}
			if(z <= min && z >=0.05) {
				min = z;
			}
			if(z >= 0.05 && z<= 7) {
				count++;
			avg+=z;
			}
		}
		avg = avg/count;
		cout << "max,avg,min: " << max << " " << avg << " " << min << endl;
//		viewer->showCloud(cloud_cluster, "cluster"); //
//		viewer->showCloud(cloud_voxel, "voxel cloud"); //
	}
	bool DirectedDepthVisionNodelet::loadPlane() {
		string dir = sc->getConfigPath() + floorPlaneFilename;
		ifstream ifs(dir);
		if(ifs.is_open()) {
			ifs >> a;
			ifs >> b;
			ifs >> c;
			ifs >> d;
			ROS_INFO("plane loaded: %f*x + %f*y + %f*z + %f = 0", a, b ,c ,d);
			return true;
		} else {
			ROS_ERROR("Couldn't load plane function!");
			return false;
		}
	}

	bool DirectedDepthVisionNodelet::customRegionGrowing(const pcl::PointXYZRGB& point_a, const pcl::PointXYZRGB& point_b, float squared_distance) {
		if((float)sqrt(squared_distance) < 0.1) {
			return true;
		} else {
			return false;
		}
	}
	void DirectedDepthVisionNodelet::projectPixelOnCenter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl) {
//		for(int i = 0; i < pcl->points.size(); i++) {
//			double x = pcl->points[i].x;
			double x = 300;
			double z = 2;
			double absToMidX = (double)abs(x - (double)width/2);
			double alpha = (double)(52 / width) * absToMidX;
			double projDist = asin(alpha*M_PI/180) * z;
//			cout << "-------------------- " << absToMidX << " " << alpha << endl;
//			cout << "-------------------- " << projDist << endl;

//		}
	}

} /* namespace msl_vision */

PLUGINLIB_EXPORT_CLASS(msl_vision::DirectedDepthVisionNodelet, nodelet::Nodelet)

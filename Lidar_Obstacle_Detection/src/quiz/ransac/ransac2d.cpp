/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// For max iterations 
	// Randomly sample subset and fit line

	while (maxIterations-- > 0) {
		std::unordered_set<int> inliersIdx; 
		while (inliersIdx.size() < 2) {
			inliersIdx.insert(rand() % (cloud->points.size())); 
		}
		// Fit the line of two points
		float x1, x2, y1, y2; 
		auto itr = inliersIdx.begin(); 
		x1 = cloud->points[*itr].x; 
		y1 = cloud->points[*itr].y; 
		itr++; 
		x2 = cloud->points[*itr].x; 
		y2 = cloud->points[*itr].y; 

		float a = (y1 - y2); 
		float b = x2 - x1;
		float c = x1*y2 - x2*y1; 
		// Measure distance between every point and fitted line
		// If distance is smaller than threshold count it as inlier
		for (size_t i = 0; i < cloud->points.size(); i++) {
			if (inliersIdx.find(i) != inliersIdx.end()) 
				continue; 
			float x3 = cloud->points[i].x; 
			float y3 = cloud->points[i].y; 
			float distance = fabs(a*x3 + b*y3 + c) / sqrt(a*a + b*b); 
			if (distance <= distanceTol) {
				inliersIdx.insert(i); 
			}
		}

		if (inliersIdx.size() > inliersResult.size()) {
			inliersResult = inliersIdx; 
		}

	}

	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;
}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// For max iterations 
	// Randomly sample subset and fit plane

	while (maxIterations-- > 0) {
		std::unordered_set<int> inliersIdx; 
		while (inliersIdx.size() < 3) {
			inliersIdx.insert(rand() % (cloud->points.size())); 
		}
		// Fit the plane of 3 points 
		float x1, x2, x3, y1, y2, y3, z1, z2, z3; 
		auto itr = inliersIdx.begin(); 
		x1 = cloud->points[*itr].x; 
		y1 = cloud->points[*itr].y; 
		z1 = cloud->points[*itr].z; 
		itr++;
		x2 = cloud->points[*itr].x; 
		y2 = cloud->points[*itr].y; 
		z2 = cloud->points[*itr].z; 
		itr++;
		x3 = cloud->points[*itr].x; 
		y3 = cloud->points[*itr].y; 
		z3 = cloud->points[*itr].z; 

		float a = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1); 
		float b = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3- z1); 
		float c = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
		float d = -(a * x1 + b * y1 + c * z1); 

		// Measure distance between every point and fitted plane
		// If distance is smaller than threshold count it as inlier
		for (size_t i = 0; i < cloud->points.size(); i++) {
			if (inliersIdx.find(i) != inliersIdx.end()) 
				continue; 
			float x4 = cloud->points[i].x; 
			float y4 = cloud->points[i].y; 
			float z4 = cloud->points[i].z; 
			float distance = fabs(a*x4 + b*y4 + c*z4 + d) / sqrt(a*a + b*b + c*c); 
			if (distance <= distanceTol) {
				inliersIdx.insert(i); 
			}
		}

		if (inliersIdx.size() > inliersResult.size()) {
			inliersResult = inliersIdx; 
		}

	}

	// Return indicies of inliers from fitted line with most inliers
	return inliersResult;
}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	// std::unordered_set<int> inliers = Ransac(cloud, 10, 1.0);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	std::unordered_set<int> inliers = RansacPlane(cloud, 50, .2);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}

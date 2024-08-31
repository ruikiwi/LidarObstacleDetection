/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    double groundSlope = 0; 

    // Create lidar sensor 
    Lidar* lidar = new Lidar(cars, groundSlope); 
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudPtr = lidar->scan(); 
    bool renderays = false, renderPoints = false, renderBoxes = true; 
    if (renderays)
        renderRays(viewer, lidar->position, pointCloudPtr); 
    if (renderPoints)
        renderPointCloud(viewer, pointCloudPtr, "pointcloud"); 

    // Create point processor
    ProcessPointClouds<pcl::PointXYZ> pointCloudProcessor; 
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentedClouds = pointCloudProcessor.SegmentPlaneRansac(pointCloudPtr, 100, 0.2); 
    renderPointCloud(viewer, segmentedClouds.first, "obstCloud", Color(1, 0, 0)); 
    renderPointCloud(viewer, segmentedClouds.second, "planeCloud", Color(0, 1, 0)); 

    int clusterId = 0; 
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointCloudProcessor.Clustering(segmentedClouds.first, 1.0, 3, 30); 
    std::vector<Color> colors = {Color(1, 0, 0), Color(1, 1, 0), Color(0, 0, 1)};
    
    for(auto& cluster : cloudClusters) {
        std::cout << "Cluster size : "; 
        pointCloudProcessor.numPoints(cluster); 
        renderPointCloud(viewer, cluster, "obstCloud " + std::to_string(clusterId), colors[clusterId % colors.size()]); 
        if(renderBoxes) {     
            Box box = pointCloudProcessor.BoundingBox(cluster); 
            renderBox(viewer, box, clusterId); 
        }
        clusterId++; 
    }

}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    ProcessPointClouds<pcl::PointXYZI> pointCloudProcessor; 
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudPtr = pointCloudProcessor.loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd"); 
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloudPtr = pointCloudProcessor.FilterCloud(pointCloudPtr, 0.2,
     Eigen::Vector4f(-14, -6, -2.3, 1),  Eigen::Vector4f(30, 6, 5, 1)); 
    renderPointCloud(viewer, filteredCloudPtr, "pointcloud"); 

    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentedClouds = pointCloudProcessor.SegmentPlaneRansac(filteredCloudPtr, 100, 0.2); 
    renderPointCloud(viewer, segmentedClouds.first, "obstCloud", Color(1, 0, 0)); 
    renderPointCloud(viewer, segmentedClouds.second, "planeCloud", Color(0, 1, 0)); 

    int clusterId = 0;
    bool renderBoxes = true;  
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointCloudProcessor.Clustering(segmentedClouds.first, .5, 10, 500); 

    std::vector<Color> colors = {Color(1, 0, 0), Color(1, 1, 0), Color(0, 0, 1)};
    
    for(auto& cluster : cloudClusters) {
        std::cout << "Cluster size : "; 
        pointCloudProcessor.numPoints(cluster); 
        renderPointCloud(viewer, cluster, "obstCloud " + std::to_string(clusterId), colors[clusterId % colors.size()]); 
        if(renderBoxes) {     
            Box box = pointCloudProcessor.BoundingBox(cluster); 
            renderBox(viewer, box, clusterId); 
        }
        clusterId++; 
    }
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessor, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloudPtr) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloudPtr = pointProcessor->FilterCloud(inputCloudPtr, 0.2,
     Eigen::Vector4f(-14, -6, -2.3, 1),  Eigen::Vector4f(30, 6, 5, 1)); 

    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentedClouds = pointProcessor->SegmentPlaneRansac(filteredCloudPtr, 100, 0.2); 
    renderPointCloud(viewer, segmentedClouds.first, "obstCloud", Color(1, 0, 0)); 
    renderPointCloud(viewer, segmentedClouds.second, "planeCloud", Color(0, 1, 0)); 

    int clusterId = 0;
    bool renderBoxes = true;  
    // std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessor->Clustering(segmentedClouds.first, 1.0, 15, 1000); 
      std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessor->ClusteringKdTree(segmentedClouds.first, .7, 10, 1000);

    std::vector<Color> colors = {Color(1, 0, 0), Color(1, 1, 0), Color(0, 0, 1)};
    
    for(auto& cluster : cloudClusters) {
        std::cout << "Cluster size : "; 
        pointProcessor->numPoints(cluster); 
        renderPointCloud(viewer, cluster, "obstCloud " + std::to_string(clusterId), colors[clusterId % colors.size()]); 
        if(renderBoxes) {     
            Box box = pointProcessor->BoundingBox(cluster); 
            renderBox(viewer, box, clusterId); 
        }
        clusterId++; 
    }
}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    bool viewSimpleHighway = false;  

    if (viewSimpleHighway) {
        simpleHighway(viewer);
    } else {
        cityBlock(viewer); 
    }

    ProcessPointClouds<pcl::PointXYZI>* pointProcesserPtr = new ProcessPointClouds<pcl::PointXYZI>(); 
    std::vector<boost::filesystem::path> stream = pointProcesserPtr->streamPcd("../src/sensors/data/pcd/data_1"); 
    auto streamIterator = stream.begin(); 
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudPtr; 

    while (!viewer->wasStopped ())
    {
        viewer->removeAllPointClouds(); 
        viewer->removeAllShapes(); 

        // Load pcd and run obstacle detection process 
        inputCloudPtr = pointProcesserPtr->loadPcd((*streamIterator).string()); 
        cityBlock(viewer, pointProcesserPtr, inputCloudPtr); 
        streamIterator++; 
        if(streamIterator == stream.end())
            streamIterator = stream.begin();
        viewer->spinOnce ();
    } 
}
// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <unordered_set>


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    std::cout << "Start filtering Pointcloud size " << cloud->points.size() << std::endl;

    typename pcl::PointCloud<PointT>::Ptr downSampledCloudPtr{new pcl::PointCloud<PointT>()}; 
    pcl::VoxelGrid<PointT> sor; 
    sor.setInputCloud(cloud); 
    sor.setLeafSize(filterRes, filterRes, filterRes); 
    sor.filter(*downSampledCloudPtr);
    std::cout << "Downsample Pointcloud size " << downSampledCloudPtr->points.size() << std::endl;

    typename pcl::PointCloud<PointT>::Ptr ROICloudPtr{new pcl::PointCloud<PointT>()}; 
    pcl::CropBox<PointT> cropBoxFilter(true); 
    cropBoxFilter.setMin(minPoint); 
    cropBoxFilter.setMax(maxPoint); 
    cropBoxFilter.setInputCloud(downSampledCloudPtr); 
    cropBoxFilter.filter(*ROICloudPtr); 

    std::vector<int> indices; 
    pcl::CropBox<PointT> roof(true); 
    roof.setMin(Eigen::Vector4f (-1.5, -1.7, -1, 1)); 
    roof.setMax(Eigen::Vector4f (2.6, 1.7, -.4, 1)); 
    roof.setInputCloud(ROICloudPtr); 
    roof.filter(indices); 

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices}; 
    for(int p : indices)
        inliers->indices.push_back(p); 

    pcl::ExtractIndices<PointT> extract; 
    extract.setInputCloud(ROICloudPtr); 
    extract.setIndices(inliers); 
    extract.setNegative(true); 
    extract.filter(*ROICloudPtr); 

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return ROICloudPtr;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstacleCloudPtr{new pcl::PointCloud<PointT>()}; 
    typename pcl::PointCloud<PointT>::Ptr planeCloudPtr{new pcl::PointCloud<PointT>()}; 
    for (int i : inliers->indices) {
        planeCloudPtr->points.push_back(cloud->points[i]); 
    }

    pcl::ExtractIndices<PointT> extract; 
    extract.setInputCloud(cloud); 
    extract.setIndices(inliers); 
    extract.setNegative(true); 
    extract.filter(*obstacleCloudPtr); 

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacleCloudPtr, planeCloudPtr);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    //Find inliers for the cloud.
    pcl::ModelCoefficients::Ptr coeffcients (new pcl::ModelCoefficients); 
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices()); 
    pcl::SACSegmentation<PointT> seg; 
    seg.setOptimizeCoefficients(true); 
    seg.setModelType(pcl::SACMODEL_PLANE); 
    seg.setModelType(pcl::SAC_RANSAC); 
    seg.setMaxIterations(maxIterations); 
    seg.setDistanceThreshold(distanceThreshold); 

    seg.setInputCloud(cloud); 
    seg.segment(*inliers, *coeffcients); 
    
    if(inliers->indices.size() == 0) {
        std::cout << "Could not estimate a planar model for given dataset." << std::endl; 
    }
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlaneRansac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold){
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
			if (distance <= distanceThreshold) {
				inliersIdx.insert(i); 
			}
		}

		if (inliersIdx.size() > inliersResult.size()) {
			inliersResult = inliersIdx; 
		}
	}  // end of iteration
    
    pcl::PointIndices::Ptr inlierResultPtr (new pcl::PointIndices()); 
    for(int idx : inliersResult) {
        inlierResultPtr->indices.push_back(idx); 
    }

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inlierResultPtr, cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud); 
    std::vector<pcl::PointIndices> clusterIndices; 
    typename pcl::EuclideanClusterExtraction<PointT> ec; 
    ec.setClusterTolerance(clusterTolerance); 
    ec.setMinClusterSize(minSize); 
    ec.setMaxClusterSize(maxSize); 
    ec.setSearchMethod(tree); 
    ec.setInputCloud(cloud); 
    ec.extract(clusterIndices); 

    int count = 0; 
    for(const auto& cluster : clusterIndices) {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>); 
        for (const auto& idx : cluster.indices) {
            cloudCluster->push_back(cloud->points[idx]); 
        }
        cloudCluster->width = cloudCluster->size(); 
        cloudCluster->height = 1; 
        cloudCluster->is_dense = true; 
        count++; 
        clusters.push_back(cloudCluster); 
    }  // end of cluster iteration 

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
void ProcessPointClouds<PointT>::Proximity(typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<bool>& isProcessed, int idx, typename pcl::PointCloud<PointT>::Ptr cluster, KdTree* tree, float clusterTolerance)
{
    auto& points = cloud->points; 
    isProcessed[idx] = true;
    cluster->push_back(points[idx]);

    PointT point = points[idx];
    std::vector<int> nearbyPoints = tree->search({point.x, point.y, point.z}, clusterTolerance);
    for (int i : nearbyPoints)
    {
        if (!isProcessed[i])
            Proximity(cloud, isProcessed, i, cluster, tree, clusterTolerance);
    }
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::ClusteringKdTree(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    auto& points = cloud->points; 
    KdTree* tree = new KdTree();
    for (int i = 0; i < points.size(); i++)
    {
        PointT point = points[i];
        tree->insert({point.x, point.y, point.y}, i);
    }

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    std::vector<bool> isProcessed(points.size(), false);   
    
    for (int i = 0; i < points.size(); ++i)
    {
        if (isProcessed[i]) continue;

        typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
        Proximity(cloud, isProcessed, i, cluster, tree, clusterTolerance);
        int clusterSize = cluster->size(); 
        if ((clusterSize >= minSize) && (clusterSize <= maxSize))
        {
            cluster->width = clusterSize;
            cluster->height = 1;
            cluster->is_dense = true;
            clusters.push_back(cluster);
        }
    }
    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{
    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}

template<typename PointT>
BoxQ ProcessPointClouds<PointT>::BoundingBoxQ(typename pcl::PointCloud<PointT>::Ptr cluster)
{
    Eigen::Vector4f pcaCentroid; 
    pcl::compute3DCentroid(cluster, pcaCentroid); 
    Eigen::Matrix3f covariance; 
    pcl::computeCovarianceMatrixNormalized(cluster, pcaCentroid, covariance); 
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigenSolver(covariance, Eigen::ComputeEigenvectors); 
    Eigen::Matrix3f eigenVectorsPCA = eigenSolver.eigenvectors(); 
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1)); 

    BoxQ box; 
    return box; 
}

template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}

template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}
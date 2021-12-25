// PCL lib Functions for processing point clouds 
#include <unordered_set>
#include "processPointClouds.h"

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

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
    // Create the filtering object
    pcl::VoxelGrid<PointT> sor;
    
    sor.setInputCloud (cloud);
    sor.setLeafSize (filterRes,filterRes,filterRes);
    sor.filter (*cloud_filtered);

    typename pcl::PointCloud<PointT>::Ptr region_filtered (new pcl::PointCloud<PointT>);
    
    pcl::CropBox<PointT> cb(true);
    cb.setMin(minPoint);
    cb.setMax(maxPoint);
    cb.setInputCloud(cloud_filtered);
    cb.filter(*region_filtered);  

    // filtering out the roof , optional 
    std::vector<int> to_remove;
    // identifying the roof point indices from the region filtered
  	pcl::CropBox<PointT> roof(true);
  	roof.setMin(Eigen::Vector4f (-3, -2, -1, 1));
  	roof.setMax(Eigen::Vector4f(3, 2, 1, 1));
  	roof.setInputCloud(region_filtered);
  	roof.filter(to_remove);
    

  	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  	for (int point : to_remove)
    	inliers->indices.push_back(point);
  
  	pcl::ExtractIndices<PointT> extract;
  	extract.setInputCloud(region_filtered);
  	extract.setIndices(inliers);
  	extract.setNegative(true);
  	extract.filter(*region_filtered);
  
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return region_filtered;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr plane (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr obstacle (new pcl::PointCloud<PointT>());

    for(int index : inliers->indices){
        plane->points.push_back(cloud->points[index]);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*obstacle);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacle, plane);
    return segResult;
}

template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Custom RANSAC 3D

    // For max iterations
    while(maxIterations--){
        std::unordered_set<int> inliers_set;
        
        // Randomly sample subset and fit line
        while(inliers_set.size()<3)
            inliers_set.insert(rand()%(cloud->points.size())  );

        
        auto itr = inliers_set.begin();

        float x_1 = cloud->points[*itr].x, y_1 = cloud->points[*itr].y, z_1 = cloud->points[*itr].z; itr++;
        float x_2 = cloud->points[*itr].x, y_2 = cloud->points[*itr].y, z_2 = cloud->points[*itr].z; itr++;
        float x_3 = cloud->points[*itr].x, y_3 = cloud->points[*itr].y, z_3 = cloud->points[*itr].z;
                       
        float v_1_x = x_2 - x_1, v_1_y = y_2 - y_1, v_1_z = z_2- z_1, 
              v_2_x = x_3 - x_1, v_2_y = y_3- y_1, v_2_z = z_3- z_1;

        float i = v_1_y * v_2_z - v_1_z * v_2_y ;
        float j = v_1_z * v_2_x - v_1_x * v_2_z ;
        float k = v_1_x * v_2_y - v_1_y * v_2_x ;

        float a = i,  b = j , c = k, d = -(i  * x_1 + j * y_1 + k * z_1);
        float radius = sqrt(a*a + b*b + c*c);
       
        // Measure distance between every point and fitted line
        for(int index = 0; index < cloud->points.size() ; index++){

            if(inliers_set.count(index)>0)
            continue;

            float sample_x = cloud->points[index].x, sample_y = cloud->points[index].y, sample_z = cloud->points[index].z;
            
            float distance = fabs(a * sample_x + b * sample_y + c * sample_z+ d )/ radius;

            // If distance is smaller than threshold count it as inlier
            if(distance <= distanceTol)
                inliers_set.insert(index);
            
        }
        // accounts for the max result
        if (inliers_set.size() > inliersResult.size())
            inliersResult = inliers_set;

    }
    // Return indicies of inliers from fitted line with most inliers
    return inliersResult;

}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	
    // TODO:: Fill in this function to find inliers for the cloud.
    // pcl::SACSegmentation<PointT> seg;
    // pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    std::unordered_set<int> _inliers = Ransac(cloud,maxIterations,distanceThreshold);
    for (const int index : _inliers) {
      inliers->indices.push_back(index);
    }
    /*
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold (distanceThreshold);
  
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    */
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }
    std::cout << "running... " << std::endl;
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}
template<typename PointT>
void ProcessPointClouds<PointT>::clusterHelper(int index, const std::vector<std::vector<float>> points, std::vector<int>& cluster, std::vector<bool>& processed,KdTree* tree, float distanceTol){

  processed[index] = true;
  cluster.push_back(index);

  std::vector<int> nearest = tree->search(points[index],distanceTol);

  for(int id: nearest){
    if(!(processed[id]))
      clusterHelper(id,points,cluster,processed,tree,distanceTol);
  }

}

template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol,int minSize, int maxSize)
{

	// TODO: Fill out this function to return list of indices for each cluster

    std::vector<std::vector<int>> clusters; 
  
    int _points_size = points.size();
    std::vector<bool> processed(_points_size,false);

	for(int i = 0; i< _points_size ; i++){
		if(processed[i]) continue;
		std::vector<int> _cluster;

		clusterHelper(i, points, _cluster, processed, tree, distanceTol);

		if (_cluster.size() >= minSize && _cluster.size() <= maxSize) 
			clusters.push_back(_cluster);

	}
    return clusters;

}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    // Creating the KdTree object for the search method of the extraction

    /* typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);

    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;

    ec.setClusterTolerance (clusterTolerance); // 2cm
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);
    */
    
    std::vector<std::vector<float>> points ;
	for (int i=0; i<cloud->points.size(); i++) 
		points.push_back(std::vector<float>{cloud->points[i].x, cloud->points[i].y,cloud->points[i].z});

    KdTree* tree = new KdTree;
    for (int i=0; i< points.size(); i++) 
    	tree->insert(points[i],i); 

    std::vector<std::vector<int>> _clusters = euclideanCluster(points, tree, clusterTolerance, minSize, maxSize);
  	
    for(std::vector<int> _cluster : _clusters)
  	{
  		typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
  		for(auto point: _cluster)
            cloud_cluster->points.push_back(cloud->points[point]);
        cloud_cluster->width = cloud_cluster->size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;    

  		clusters.push_back(cloud_cluster);
  	}
    
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

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
#include "lib.h"
#include <unordered_set>

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int maxIterations, float distanceTol)
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

        pcl::PointXYZ p_1, p_2, p_3;
                        p_1 = cloud->points[*(itr++)]; 
                        p_2 = cloud->points[*(itr++)]; 
                        p_3 = cloud->points[*itr];

        pcl::PointXYZ v_1(p_2.x - p_1.x, p_2.y - p_1.y, p_2.z - p_1.z), 
                        v_2(p_3.x - p_1.x, p_3.y - p_1.y, p_3.z - p_1.z);

        float i = v_1.y * v_2.z - v_1.z * v_2.y ;
        float j = v_1.z * v_2.x - v_1.x * v_2.z ;
        float k = v_1.x * v_2.y - v_1.y * v_2.x ;

        float a = i,  b = j , c = k, d = -(i  * p_1.x + j * p_1.y + k * p_1.z);
        float radius = sqrt(a*a + b*b + c*c);

        // Measure distance between every point and fitted line
        for(int index = 0; index < cloud->points.size() ; index++){

            if(inliers_set.count(index)>0)
            continue;

            pcl::PointXYZ sample = cloud->points[index];
            
            float d = fabs(a * sample.x + b * sample.y + c * sample.z+ d )/ radius;

            // If distance is smaller than threshold count it as inlier
            if(d <= distanceTol)
                inliers_set.insert(index);
            
        }
        // accounts for the max result
        if (inliers_set.size() > inliersResult.size())
            inliersResult = inliers_set;

    }
    // Return indicies of inliers from fitted line with most inliers
    return inliersResult;

}
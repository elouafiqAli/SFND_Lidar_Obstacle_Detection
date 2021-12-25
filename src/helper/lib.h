#ifndef LIB_H
#define LIB_H
#include <unordered_set>

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int maxIterations, float distanceTol);

#endif

#ifndef UTILS_H
#define UTILS_H
#include <pcl/io/io.h>
#include <iostream>
#include <pcl/search/kdtree.h>


double
computeCloudResolution(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud);

#endif
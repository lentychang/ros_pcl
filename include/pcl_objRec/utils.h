#ifndef UTILS_H
#define UTILS_H
#include <pcl/io/io.h>
#include <iostream>
#include <pcl/search/kdtree.h>
#include <pcl/surface/mls.h>

#include <vector>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>

#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/console/parse.h>
#include <pcl/segmentation/extract_clusters.h>



double
computeCloudResolution(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud);

void
mls_upsampling(pcl::PointCloud<pcl::PointXYZ>::Ptr& inCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &outCloud, float resolution ,float stepRatio=0.5);

void 
euclideanSeg (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int minClusterSize=30, int maxClusterSize=1500, float cluster_tol=0.01f, bool enableViewer=false);

#endif
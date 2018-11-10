#ifndef UTILS_H
#define UTILS_H
#include <pcl/io/io.h>
#include <iostream>
#include <unistd.h>
#include <stdio.h>
#include <limits.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/mls.h>

#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/console/parse.h>
#include <pcl/segmentation/extract_clusters.h>



double
computeCloudResolution(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud);

void
mls_upsampling(pcl::PointCloud<pcl::PointXYZ>::Ptr& inCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &outCloud, float resolution ,float stepRatio=0.5);

std::vector <pcl::PointIndices> 
euclideanSeg (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud, int minClusterSize, int maxClusterSize, float cluster_tol, bool enableViewer);
void extractPCDfromCluster(std::vector <pcl::PointIndices>& clusters, int nth_cluster, const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& outCloud);
void cropPcd(const Eigen::Vector4f& min, const Eigen::Vector4f& max, const Eigen::Vector3f &translation, const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& outCloud, bool setNegative=false);

#endif
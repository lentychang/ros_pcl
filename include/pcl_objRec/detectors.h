#ifndef DETECTORS_H
#define DETECTORS_H

#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/io.h>
#include "utils.h"
#include "../config/pointType.hpp"

#include <boost/thread/thread.hpp>
#include <iostream>
#include <string>

void update_detector_fileName(std::string filename, std::string source);
void getIssKeypoints(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& inputCloudPoints,
                     const pcl::PointCloud<pcl::PointXYZ>::Ptr& issKeypointclouds);

void getSiftKeypoints(const pcl::PointCloud<pcl::PointNormal>::ConstPtr& inputPointNormals,
                      const pcl::PointCloud<pcl::PointXYZ>::Ptr& siftKeypoints);

void getUniformsamplekeypoints(	const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& inputPoints,
                      			const pcl::PointCloud<pcl::PointXYZ>::Ptr& uniKeypoints,
								float radiusSearchR);

#endif
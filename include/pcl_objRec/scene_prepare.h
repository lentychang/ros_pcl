#ifndef SCENE_PREPARE_H
#define SCENE_PREPARE_H

#include "detectors.h"
#include "descriptors.h"
#include "preprocessor.h"
#include <pcl/io/pcd_io.h>
#include <pcl/io/openni2_grabber.h>
#include <pcl/common/io.h>
#include <pcl/search/flann_search.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/board.h>
#include <pcl/correspondence.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <iostream>
#include <string>
#include <cstdio>
#include <ctime>
#include <boost/thread/thread.hpp>
#include <pcl_objRec/utils.h>

using namespace std;


//g++ -g ../src/localPipeline__.cpp ../src/detectors.cpp ../src/preprocessor.cpp ../src/descriptors.cpp -o bin/localpipeline -L /usr/lib/x86_64-linux-gnu/  -I /usr/include/openni2 -lboost_system -lpcl_common -lpcl_io -lpcl_features -lpcl_search -lpcl_filters -lboost_thread -lpcl_filters -lpcl_kdtree -lpcl_octree -lflann -std=c++11
void scenenPreprocess(const std::string& dataDir, const std::string& srcName, float pntLeftRatio=0.07);

#endif
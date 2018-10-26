#ifndef MODEL_SCENE_H
#define MODEL_SCENE_H
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/feature.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <iostream>
#include <string>
#include <boost/thread/thread.hpp>

#include <pcl/search/kdtree.h>
#include <pcl/common/io.h>
#include "detectors.h"
#include "descriptors.h"
#include "preprocessor.h"
#include <pcl_objRec/utils.h>
//#include <pcl/visualization/pcl_visualizer.h>

using namespace std;

//debug command:
//gdb -d ../src/ ./model_prepare

// gcc compile command:
//g++ -g ../src/model_prepare_iss.cpp -o bin/model_prepare_iss -lboost_system -lpcl_common -lpcl_io -lpcl_features -lpcl_search -lpcl_filters -lboost_thread -lpcl_filters
//g++ -g ../src/model_prepare_iss.cpp ../src/detectors.cpp ../src/descriptors.cpp -o bin/model_prepare_iss -I /root/src/pcl_cvfh/src -lboost_system -lpcl_common -lpcl_io -lpcl_features -lpcl_search -lpcl_filters -lboost_thread -lpcl_filters -std=c++11

void modelPreprocess(const std::string& dataDir ,const std::string& modelName);
#endif
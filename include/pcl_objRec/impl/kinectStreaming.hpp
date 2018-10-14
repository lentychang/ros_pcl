#include "../detectors.h"
#include "../descriptors.h"
#include "../preprocessor.h"

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
#include <pcl/console/parse.h>
#include <iostream>
#include <string>
#include <cstdio>
#include <ctime>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/cloud_viewer.h>

bool SavePcd4Debug = false;
bool DEBUG = true;

typedef pcl::PointXYZ pntType;
typedef pcl::PointNormal pntNType;

void cloud_viewer (const typename pcl::PointCloud<pntType>::ConstPtr& cloud) {
    pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
    viewer.showCloud (cloud);
}

using namespace std;


//g++ -g ../src/localPipeline__.cpp ../src/detectors.cpp ../src/preprocessor.cpp ../src/descriptors.cpp -o bin/localpipeline -L /usr/lib/x86_64-linux-gnu/  -I /usr/include/openni2 -lboost_system -lpcl_common -lpcl_io -lpcl_features -lpcl_search -lpcl_filters -lboost_thread -lpcl_filters -lpcl_kdtree -lpcl_octree -lflann -std=c++11

int main(int argc, char** argv)
{
    pcl::PointCloud<pntType>::Ptr __cloudPoints(new pcl::PointCloud<pntType>);
	pcl::PointCloud<pntType>::Ptr cloudPoints(new pcl::PointCloud<pntType>);
    pcl::PointCloud<pcl::Normal>::Ptr cloudNormals(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pntNType>::Ptr pointNormals(new pcl::PointCloud<pntNType>);

    // callback function that copy the input data to point cloud
    boost::function<void(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> function = [&__cloudPoints](const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud) {
        pcl::copyPointCloud(*cloud, *__cloudPoints);
    };

    // Create Kinect2Grabber
    pcl::Grabber* grabber = new pcl::io::OpenNI2Grabber();
    // Regist Callback Function
    grabber->registerCallback(function);
    // Start Retrieve Data
    grabber->start();
    boost::this_thread::sleep(boost::posix_time::seconds(2));
    string rawname = "../models/lf064-01";
    string appendName = "_scene";
    Preprocessor<pntType, pntNType> preprocess;
    preprocess.setFileName(rawname, appendName);

    pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
    // bool leaveLoop = false;
    // int key = 1;
    float voxelSize = 0.002f;
    // int flag_statistic = 0;
    // while (!leaveLoop){
        // cout << "enter voxel size:" << endl;
        // cin >> voxelSize;
        // cout << "enter flag size:" << endl;
        // cin >> flag_statistic;

    int doGather =  pcl::console::find_argument(argc,argv,"-g");
    
    int doStatistic =  pcl::console::find_argument(argc,argv,"-s");
    int doVoxel = pcl::console::find_argument(argc,argv,"-v");
    int doRansac = pcl::console::find_argument(argc,argv,"-r");
    int doMlsSmooth = pcl::console::find_argument(argc,argv,"-m");
    
    preprocess.setCallbackCloud(__cloudPoints);
    if (doGather >=1) {
        preprocess.getPointsFromCamera(atoi(argv[doGather+1]));
    }
    else { preprocess.getPointsFromCamera(1);}
    preprocess.doRmNaNPoints();

    if (doStatistic>=1) {preprocess.doStatisticFilter(atoi(argv[doStatistic+1]),1.0);}
    preprocess.doZFilter(0.5,1.3);
    if (doVoxel >=1) {preprocess.doVoxelFilter(atof(argv[doVoxel+1]));}
    if (doRansac >=1) {preprocess.doRansacPlane(atof(argv[doRansac+1]));}
    if (doMlsSmooth >=1) {preprocess.doMlsSmoothing(false,atoi(argv[doMlsSmooth+1]));}

    // if (flag_statistic==1) preprocess.doStatisticFilter();
    //preprocess.doNormalEstimation("radius", 0.02, "flannkdtree");
    preprocess.getPoints(cloudPoints);
    preprocess.savePoints();
        // while (!viewer.wasStopped ()) {
        //     preprocess.setCallbackCloud(__cloudPoints);
        //     preprocess.getPointsFromCamera(1);
        //     preprocess.doRmNaNPoints();
        //     //preprocess.doStatisticFilter();
        //     preprocess.doZFilter(0.5,1.3);
        //     preprocess.doVoxelFilter(voxelSize,voxelSize,voxelSize);
        //     // if (flag_statistic==1) preprocess.doStatisticFilter();
        //     //preprocess.doNormalEstimation("radius", 0.02, "flannkdtree");
        //     preprocess.getPoints(cloudPoints);
        //     viewer.showCloud (cloudPoints);
        // }

        // preprocess.savePoints();
        // cout << "enter 0 to leave loop" << endl;
        // cin >> key;
        // if (key == 0) leaveLoop = true;
    //}

	grabber->stop();
}

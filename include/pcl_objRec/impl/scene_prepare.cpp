#include "../scene_prepare.h"

//g++ -g ../src/localPipeline__.cpp ../src/detectors.cpp ../src/preprocessor.cpp ../src/descriptors.cpp -o bin/localpipeline -L /usr/lib/x86_64-linux-gnu/  -I /usr/include/openni2 -lboost_system -lpcl_common -lpcl_io -lpcl_features -lpcl_search -lpcl_filters -lboost_thread -lpcl_filters -lpcl_kdtree -lpcl_octree -lflann -std=c++11
void scenenPreprocess(const std::string& dataDir)
{
	string inputName = dataDir + "/srcPCD/" + "scene.pcd";

	cout << "Read scene.pcd from:" << inputName << endl;
	// Object for storing the point cloud.
	pcl::PointCloud<pntType>::Ptr scenePoints(new pcl::PointCloud<pntType>());
	pcl::PointCloud<pcl::Normal>::Ptr sceneNormals(new pcl::PointCloud<pcl::Normal>());
	pcl::PointCloud<pntNType>::Ptr scenePointNormals(new pcl::PointCloud<pntNType>);	
	pcl::PointCloud<pcl::SHOT352>::Ptr sceneShotDescriptors(new pcl::PointCloud<pcl::SHOT352>());

	if (pcl::io::loadPCDFile<pntType>(inputName, *scenePoints) != 0) {
        std::cout << "[Error] Loading PCD file failed" << std::endl;
    }


    Preprocessor<pntType,pntNType> preprocess;
    preprocess.addPoints(scenePoints);
    string rawName = dataDir + "/filtered/" + "scene";
    string appendName = "";
    preprocess.setFileName(rawName, appendName);
 
    preprocess.doRmNaNPoints();
    preprocess.doZFilter(0.5,1.5);
	// preprocess.doStatisticFilter(20, 1.0f);
	preprocess.getPoints(scenePoints);
	computeCloudResolution(scenePoints);
	preprocess.doVoxelFilter(0.001);
	preprocess.segRansacPlane(0.08,0.001);
	preprocess.getPoints(scenePoints);
	// appendName = "_after Ransac";
	// preprocess.setFileName(rawName, appendName);
	// preprocess.savePoints();
	// appendName = "";
	// preprocess.setFileName(rawName, appendName);

	cout << "cloud resolution:" << computeCloudResolution(scenePoints) << endl;

    //preprocess.doVoxelFilter(0.01);
    preprocess.doNormalEstimation("radius", 0.005, "flannkdtree");
    preprocess.getData(scenePoints, sceneNormals, scenePointNormals);
    preprocess.saveData();

	// filename = srcPcdPath + "/../detector/scene.pcd";
	// lastindex = filename.find_last_of(".");
    // rawName = filename.substr(0, lastindex);
	// appendName = "";
	// update_detector_fileName(rawName, appendName);

	// pcl::PointCloud<pcl::PointXYZ>::Ptr issKeypoints(new pcl::PointCloud<pcl::PointXYZ>);
	// getIssKeypoints(scenePoints, issKeypoints);
	// pcl::PointCloud<pcl::PointXYZ>::Ptr siftKeypoints(new pcl::PointCloud<pcl::PointXYZ>);
	// getSiftKeypoints(scenePointNormals, siftKeypoints);

	// pcl::PointCloud<pntType>::Ptr uniKeypoints(new pcl::PointCloud<pntType>);
	// getUniformsamplekeypoints(scenePoints, uniKeypoints, 6.0);

	// std::cout << "[Info] Scene preprocess finished" << std::endl;
}

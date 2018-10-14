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
#include <ros_pcl/utils.h>

using namespace std;

//g++ -g ../src/localPipeline__.cpp ../src/detectors.cpp ../src/preprocessor.cpp ../src/descriptors.cpp -o bin/localpipeline -L /usr/lib/x86_64-linux-gnu/  -I /usr/include/openni2 -lboost_system -lpcl_common -lpcl_io -lpcl_features -lpcl_search -lpcl_filters -lboost_thread -lpcl_filters -lpcl_kdtree -lpcl_octree -lflann -std=c++11

int main(int argc, char **argv)
{

	cout << "command hint: [Input pcd file]" << endl;
	// Object for storing the point cloud.

	// Read a PCD file from disk.
	// pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhDescriptorsModel(new pcl::PointCloud<pcl::FPFHSignature33>());
	// if (pcl::io::loadPCDFile<pcl::FPFHSignature33>(argv[1], *fpfhDescriptorsModel) != 0) {
	//     return -1;
	// }
	string modelName = argv[1];
	size_t lastindex = modelName.find_last_of(".");
	string rawname = modelName.substr(0, lastindex);
	string modelPointsName = rawname + "_model_points_.pcd";
	string modelNormalsName = rawname + "_model_normals_.pcd";
	string modelKeypointsName = rawname + "_model_sift_.pcd";
	string modelDescriptorName = rawname + "_model_sift_shot_.pcd";

	pcl::PointCloud<pcl::PointXYZ>::Ptr modelPoints(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::io::loadPCDFile<pcl::PointXYZ>(modelPointsName, *modelPoints);
	pcl::PointCloud<pcl::Normal>::Ptr modelNormals(new pcl::PointCloud<pcl::Normal>());
	pcl::io::loadPCDFile<pcl::Normal>(modelNormalsName, *modelNormals);
	pcl::PointCloud<pcl::PointXYZ>::Ptr modelKeypoints(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::io::loadPCDFile<pcl::PointXYZ>(modelKeypointsName, *modelKeypoints);
	pcl::PointCloud<pcl::SHOT352>::Ptr modelShotDescriptors(new pcl::PointCloud<pcl::SHOT352>());
	pcl::io::loadPCDFile<pcl::SHOT352>(modelDescriptorName, *modelShotDescriptors);

	string sceneName = argv[2];
	lastindex = sceneName.find_last_of(".");
	string sceneRawname = sceneName.substr(0, lastindex);
	string scenePointsName = sceneRawname + "_points_.pcd";
	string sceneNormalsName = sceneRawname + "_normals_.pcd";
	string scenePointNormalsName = sceneRawname + "_pointNormals_.pcd";
	string sceneKeypointsName = sceneRawname + "_sift_.pcd";
	string sceneDescriptorName = sceneRawname + "_sift_shot_.pcd";

	string filename = argv[1];
	lastindex = filename.find_last_of(".");
	string rawName = filename.substr(0, lastindex);
	string appendName = "";

	pcl::PointCloud<pcl::PointXYZ>::Ptr scenePoints(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::io::loadPCDFile<pcl::PointXYZ>(scenePointsName, *scenePoints);
	pcl::PointCloud<pcl::Normal>::Ptr sceneNormals(new pcl::PointCloud<pcl::Normal>());
	pcl::io::loadPCDFile<pcl::Normal>(sceneNormalsName, *sceneNormals);
	pcl::PointCloud<pcl::PointXYZ>::Ptr sceneKeypoints(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::io::loadPCDFile<pcl::PointXYZ>(sceneKeypointsName, *sceneKeypoints);
	pcl::PointCloud<pcl::PointNormal>::Ptr scenePointNormals(new pcl::PointCloud<pcl::PointNormal>());
	pcl::io::loadPCDFile<pcl::PointNormal>(scenePointNormalsName, *scenePointNormals);

	Descriptors<pntType,pntNType> descriptor;
	// // ISS keypoint detector object.
	// appendName = "_scene_iss";
	// update_detector_fileName(rawName, appendName);
	// pcl::PointCloud<pcl::PointXYZ>::Ptr issKeypoints(new pcl::PointCloud<pcl::PointXYZ>);
	// getIssKeypoints(cloudPoints, issKeypoints);
	// // FPFH with issKeypoints
	// descriptor.setFilename(rawName, appendName);
	// descriptor.setData(cloudPoints, cloudNormals, issKeypoints, pointNormals);
	// pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhDescriptorSceneIss(new pcl::PointCloud<pcl::FPFHSignature33>());
	// descriptor.getFpfh(fpfhDescriptorSceneIss);

	// // Estimate the sift interest points using normals values from xyz as the Intensity variants
	// appendName = "_scene_sift";
	// update_detector_fileName(rawName, appendName);
	// pcl::PointCloud<pcl::PointXYZ>::Ptr siftKeypoints(new pcl::PointCloud<pcl::PointXYZ>);
	// getSiftKeypoints(pointNormals, siftKeypoints);
	// // FPFH with siftKeypoints
	// descriptor.setFilename(rawName, appendName);
	// descriptor.setData(cloudPoints, cloudNormals, siftKeypoints, pointNormals);
	// pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhDescriptorSceneSift(new pcl::PointCloud<pcl::FPFHSignature33>());
	// descriptor.getFpfh(fpfhDescriptorSceneSift);

	// // FPFH with siftKeypoints
	descriptor.setFilename(rawName, appendName);
	descriptor.setData(scenePoints, sceneNormals, sceneKeypoints, scenePointNormals);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr sceneSiftFpfhDescriptor(new pcl::PointCloud<pcl::FPFHSignature33>());
	descriptor.getFpfh(sceneSiftFpfhDescriptor);

	descriptor.setData(scenePoints, sceneNormals, sceneKeypoints, scenePointNormals);
	pcl::PointCloud<pcl::SHOT352>::Ptr sceneUniShotDescriptor(new pcl::PointCloud<pcl::SHOT352>());
	descriptor.getShot(sceneUniShotDescriptor);

	// A kd-tree object that uses the FLANN library for fast search of nearest neighbors.
	std::clock_t __start;
	double __duration;
	__start = std::clock();
	pcl::KdTreeFLANN<pcl::SHOT352> matching;
	matching.setInputCloud(modelShotDescriptors);
	// A Correspondence object stores the indices of the query and the match,
	// and the distance/weight.
	pcl::CorrespondencesPtr model_scene_corrs(new pcl::Correspondences());

	// Check every descriptor computed for the scene.
	for (size_t i = 0; i < sceneUniShotDescriptor->size(); ++i)
	{
		std::vector<int> neighbors(1);
		std::vector<float> squaredDistances(1);
		// Ignore NaNs.
		if (!pcl_isfinite(sceneUniShotDescriptor->at(i).descriptor[0]))
		{
			continue;
		}
		// Find the nearest neighbor (in descriptor space)...
		int neighborCount = matching.nearestKSearch(sceneUniShotDescriptor->at(i), 1, neighbors, squaredDistances);
		// ...and add a new correspondence if the distance is less than a threshold
		// (SHOT distances are between 0 and 1, other descriptors use different metrics).
		if (neighborCount == 1 && squaredDistances[0] < 0.25f)
		{
			pcl::Correspondence correspondence(neighbors[0], static_cast<int>(i), squaredDistances[0]);
			model_scene_corrs->push_back(correspondence);
		}
	}
	std::cout << "Found " << model_scene_corrs->size() << " correspondences." << std::endl;

	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> rototranslations;
	std::vector<pcl::Correspondences> clustered_corrs;
	//  Compute (Keypoints) Reference Frames only for Hough
	//
	pcl::PointCloud<pcl::ReferenceFrame>::Ptr model_rf(new pcl::PointCloud<pcl::ReferenceFrame>());
	pcl::PointCloud<pcl::ReferenceFrame>::Ptr scene_rf(new pcl::PointCloud<pcl::ReferenceFrame>());

	pcl::BOARDLocalReferenceFrameEstimation<pcl::PointXYZ, pcl::Normal, pcl::ReferenceFrame> rf_est;
	rf_est.setFindHoles(true);
	rf_est.setRadiusSearch(0.015f);

	rf_est.setInputCloud(modelKeypoints);
	rf_est.setInputNormals(modelNormals);
	rf_est.setSearchSurface(modelPoints);
	rf_est.compute(*model_rf);

	rf_est.setInputCloud(sceneKeypoints);
	rf_est.setInputNormals(sceneNormals);
	rf_est.setSearchSurface(scenePoints);
	rf_est.compute(*scene_rf);

	//  Clustering
	pcl::Hough3DGrouping<pcl::PointXYZ, pcl::PointXYZ, pcl::ReferenceFrame, pcl::ReferenceFrame> clusterer;
	clusterer.setHoughBinSize(0.01f);
	clusterer.setHoughThreshold(5.0f);
	clusterer.setUseInterpolation(true);
	clusterer.setUseDistanceWeight(false);

	clusterer.setInputCloud(modelKeypoints);
	clusterer.setInputRf(model_rf);
	clusterer.setSceneCloud(sceneKeypoints);
	clusterer.setSceneRf(scene_rf);
	clusterer.setModelSceneCorrespondences(model_scene_corrs);

	//clusterer.cluster (clustered_corrs);
	clusterer.recognize(rototranslations, clustered_corrs);

	//  Output results
	//
	std::cout << "Model instances found: " << rototranslations.size() << std::endl;
	for (size_t i = 0; i < rototranslations.size(); ++i)
	{
		std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
		std::cout << "        Correspondences belonging to this instance: " << clustered_corrs[i].size() << std::endl;

		// Print the rotation matrix and translation vector
		Eigen::Matrix3f rotation = rototranslations[i].block<3, 3>(0, 0);
		Eigen::Vector3f translation = rototranslations[i].block<3, 1>(0, 3);

		printf("\n");
		printf("            | %6.3f %6.3f %6.3f | \n", rotation(0, 0), rotation(0, 1), rotation(0, 2));
		printf("        R = | %6.3f %6.3f %6.3f | \n", rotation(1, 0), rotation(1, 1), rotation(1, 2));
		printf("            | %6.3f %6.3f %6.3f | \n", rotation(2, 0), rotation(2, 1), rotation(2, 2));
		printf("\n");
		printf("        t = < %0.3f, %0.3f, %0.3f >\n", translation(0), translation(1), translation(2));
	}

	//  Visualization
	//
	pcl::visualization::PCLVisualizer viewer("Correspondence Grouping");
	viewer.addPointCloud(scenePoints, "scene_cloud");

	pcl::PointCloud<pcl::PointXYZ>::Ptr off_scene_model(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr off_scene_model_keypoints(new pcl::PointCloud<pcl::PointXYZ>());
	//  We are translating the model so that it doesn't end in the middle of the scene representation
	pcl::transformPointCloud(*modelPoints, *off_scene_model, Eigen::Vector3f(-1, 0, 0), Eigen::Quaternionf(1, 0, 0, 0));
	pcl::transformPointCloud(*modelKeypoints, *off_scene_model_keypoints, Eigen::Vector3f(-1, 0, 0), Eigen::Quaternionf(1, 0, 0, 0));

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> off_scene_model_color_handler(off_scene_model, 255, 255, 128);
	viewer.addPointCloud(off_scene_model, off_scene_model_color_handler, "off_scene_model");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> scene_keypoints_color_handler(sceneKeypoints, 0, 0, 255);
	viewer.addPointCloud(sceneKeypoints, scene_keypoints_color_handler, "scene_keypoints");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "scene_keypoints");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> off_scene_model_keypoints_color_handler(off_scene_model_keypoints, 0, 0, 255);
	viewer.addPointCloud(off_scene_model_keypoints, off_scene_model_keypoints_color_handler, "off_scene_model_keypoints");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "off_scene_model_keypoints");

	for (size_t i = 0; i < rototranslations.size(); ++i)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr rotated_model(new pcl::PointCloud<pcl::PointXYZ>());
		pcl::transformPointCloud(*modelPoints, *rotated_model, rototranslations[i]);

		std::stringstream ss_cloud;
		ss_cloud << "instance" << i;

		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rotated_model_color_handler(rotated_model, 255, 0, 0);
		viewer.addPointCloud(rotated_model, rotated_model_color_handler, ss_cloud.str());

		for (size_t j = 0; j < clustered_corrs[i].size(); ++j)
		{
			std::stringstream ss_line;
			ss_line << "correspondence_line" << i << "_" << j;
			pcl::PointXYZ &model_point = off_scene_model_keypoints->at(clustered_corrs[i][j].index_query);
			pcl::PointXYZ &scene_point = sceneKeypoints->at(clustered_corrs[i][j].index_match);

			//  We are drawing a line for each pair of clustered correspondences found between the model and the scene
			viewer.addLine<pcl::PointXYZ, pcl::PointXYZ>(model_point, scene_point, 0, 255, 0, ss_line.str());
		}
	}

	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}

	return 0;
}

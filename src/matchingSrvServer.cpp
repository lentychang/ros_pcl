#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/features/board.h>
#include <pcl/correspondence.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "../config/pointType.hpp"
#include <ros_pcl_msgs/srv_matching.h>
#include <pcl_objRec/utils.h>
#include <pcl_objRec/preprocessor.h>

template<typename TpntType, typename TpntNType, typename TdescriptorType>
void matching_descriptor(std::string& dataDir, std::string& modelName, std::string& detector, std::string& descriptor) {

    // read filtered scene Clouds
    typename pcl::PointCloud<TpntType>::Ptr scenePnts(new pcl::PointCloud<TpntType>());
    pcl::PointCloud<pcl::Normal>::Ptr sceneNs(new pcl::PointCloud<pcl::Normal>());
	typename pcl::PointCloud<TpntNType>::Ptr scenePntNs(new pcl::PointCloud<TpntNType>());

    std::string sceneCloudPath = dataDir + "/filtered/scene_points_.pcd";
    if (pcl::io::loadPCDFile<TpntType>(sceneCloudPath, *scenePnts) != 0) {
        std::cout << "[Error] Loading PCD file failed" << std::endl;
    }
    sceneCloudPath = dataDir + "/filtered/scene_pointNormals_.pcd";
    if (pcl::io::loadPCDFile<TpntNType>(sceneCloudPath, *scenePntNs) != 0) {
        std::cout << "[Error] Loading PCD file failed" << std::endl;
    }
    sceneCloudPath = dataDir + "/filtered/scene_normals_.pcd";
    if (pcl::io::loadPCDFile<pcl::Normal>(sceneCloudPath, *sceneNs) != 0) {
        std::cout << "[Error] Loading PCD file failed" << std::endl;
    }

    // read filtered model Clouds
    typename pcl::PointCloud<TpntType>::Ptr modelPnts(new pcl::PointCloud<TpntType>());
    pcl::PointCloud<pcl::Normal>::Ptr modelNs(new pcl::PointCloud<pcl::Normal>());
	typename pcl::PointCloud<TpntNType>::Ptr modelPntNs(new pcl::PointCloud<TpntNType>());

    std::string modelCloudPath = dataDir + "/filtered/" + modelName + "_points_.pcd";
    if (pcl::io::loadPCDFile<TpntType>(modelCloudPath, *modelPnts) != 0) {
        std::cout << "[Error] Loading PCD file failed" << std::endl;
    }
    modelCloudPath = dataDir + "/filtered/" + modelName + "_pointNormals_.pcd";
    if (pcl::io::loadPCDFile<TpntNType>(modelCloudPath, *modelPntNs) != 0) {
        std::cout << "[Error] Loading PCD file failed" << std::endl;
    }
    modelCloudPath = dataDir + "/filtered/" + modelName + "_normals_.pcd";
    if (pcl::io::loadPCDFile<pcl::Normal>(modelCloudPath, *modelNs) != 0) {
        std::cout << "[Error] Loading PCD file failed" << std::endl;
    }
	double resolution = computeCloudResolution(modelPnts);
	Preprocessor<TpntType, TpntNType> preprocess;
    // assign path to read keypoints and descriptor
    typename pcl::PointCloud<TpntType>::Ptr sceneKeypnts(new  pcl::PointCloud<TpntType>);
    typename pcl::PointCloud<TpntType>::Ptr modelKeypnts(new  pcl::PointCloud<TpntType>);
    typename pcl::PointCloud<TdescriptorType>::Ptr sceneDescriptor(new pcl::PointCloud<TdescriptorType>());
    typename pcl::PointCloud<TdescriptorType>::Ptr modelDescriptor(new pcl::PointCloud<TdescriptorType>());   

    std::string sceneKeypntsFilePath;
    std::string modelKeypntsFilePath;
    std::string sceneDescriptorFilePath;
    std::string modelDescriptorFilePath;

    if(detector=="iss" || detector=="sift" || detector=="uni") {
        sceneKeypntsFilePath = dataDir + "/detector/scene_" + detector + "_.pcd";
        modelKeypntsFilePath = dataDir + "/detector/" + modelName + "_" + detector + "_.pcd";
        if(descriptor=="pfh" || descriptor=="fpfh" || descriptor=="vfh" || descriptor=="rsd" ||
           descriptor=="sc3d" || descriptor=="usc" || descriptor=="shot") {
        sceneDescriptorFilePath = dataDir + "/descriptor/" + detector + "/scene_" + descriptor +"_.pcd";
        modelDescriptorFilePath = dataDir + "/descriptor/" + detector + "/" + modelName + "_" + descriptor +"_.pcd";        
        }
        else { ROS_FATAL("unknown descriptor name, please enter only [pfh|fpfh|vfh|rsd|sc3d|usc|shot]");}
    }
    else { ROS_FATAL("unknown detector name, please enter only [iss|sift|uni]");}

    if (pcl::io::loadPCDFile<TpntType>(sceneKeypntsFilePath, *sceneKeypnts) != 0) {
        std::cout << "[Error] Loading PCD file failed" << std::endl;
    }
    if (pcl::io::loadPCDFile<TpntType>(modelKeypntsFilePath, *modelKeypnts) != 0) {
        std::cout << "[Error] Loading PCD file failed" << std::endl;    }

    if (pcl::io::loadPCDFile<TdescriptorType>(sceneDescriptorFilePath, *sceneDescriptor) != 0) {
        std::cout << "[Error] Loading PCD file failed" << std::endl;
    }
    if (pcl::io::loadPCDFile<TdescriptorType>(modelDescriptorFilePath, *modelDescriptor) != 0) {
        std::cout << "[Error] Loading PCD file failed" << std::endl;
    }

    // A kd-tree object that uses the FLANN library for fast search of nearest neighbors.
	pcl::KdTreeFLANN<TdescriptorType> matching;
	matching.setInputCloud(modelDescriptor);
	// A Correspondence object stores the indices of the query and the match,
	// and the distance/weight.
	pcl::CorrespondencesPtr model_scene_corrs(new pcl::Correspondences());

	// Check every descriptor computed for the scene.
	for (size_t i = 0; i < sceneDescriptor->size(); ++i)
	{
		std::vector<int> neighbors(1);
		std::vector<float> squaredDistances(1);
		// Ignore NaNs.
		if (!pcl_isfinite(sceneDescriptor->at(i).descriptor[0]))
		    {continue;}
		// Find the nearest neighbor (in descriptor space)...
		int neighborCount = matching.nearestKSearch(sceneDescriptor->at(i), 1, neighbors, squaredDistances);
		// ...and add a new correspondence if the distance is less than a threshold
		// (SHOT distances are between 0 and 1, other descriptors use different metrics).
		if (neighborCount == 1 && squaredDistances[0] < 1.0f) {
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
	rf_est.setRadiusSearch(3.0*resolution);

	rf_est.setInputCloud(modelKeypnts);
	rf_est.setInputNormals(modelNs);
	rf_est.setSearchSurface(modelPnts);
	rf_est.compute(*model_rf);

	rf_est.setInputCloud(sceneKeypnts);
	rf_est.setInputNormals(sceneNs);
	rf_est.setSearchSurface(scenePnts);
	rf_est.compute(*scene_rf);

	//  Clustering
	pcl::Hough3DGrouping<pcl::PointXYZ, pcl::PointXYZ, pcl::ReferenceFrame, pcl::ReferenceFrame> clusterer;
	clusterer.setHoughBinSize(0.01f);
	clusterer.setHoughThreshold(5.0f);
	clusterer.setUseInterpolation(true);
	clusterer.setUseDistanceWeight(false);

	clusterer.setInputCloud(modelKeypnts);
	clusterer.setInputRf(model_rf);
	clusterer.setSceneCloud(sceneKeypnts);
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
	viewer.addPointCloud(scenePnts, "scene_cloud");

	pcl::PointCloud<pcl::PointXYZ>::Ptr off_scene_model(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr off_scene_model_keypoints(new pcl::PointCloud<pcl::PointXYZ>());
	//  We are translating the model so that it doesn't end in the middle of the scene representation
	pcl::transformPointCloud(*modelPnts, *off_scene_model, Eigen::Vector3f(-1, 0, 0), Eigen::Quaternionf(1, 0, 0, 0));
	pcl::transformPointCloud(*modelKeypnts, *off_scene_model_keypoints, Eigen::Vector3f(-1, 0, 0), Eigen::Quaternionf(1, 0, 0, 0));

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> off_scene_model_color_handler(off_scene_model, 255, 255, 128);
	viewer.addPointCloud(off_scene_model, off_scene_model_color_handler, "off_scene_model");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> scene_keypoints_color_handler(sceneKeypnts, 0, 0, 255);
	viewer.addPointCloud(sceneKeypnts, scene_keypoints_color_handler, "scene_keypoints");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "scene_keypoints");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> off_scene_model_keypoints_color_handler(off_scene_model_keypoints, 0, 0, 255);
	viewer.addPointCloud(off_scene_model_keypoints, off_scene_model_keypoints_color_handler, "off_scene_model_keypoints");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "off_scene_model_keypoints");

	for (size_t i = 0; i < rototranslations.size(); ++i)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr rotated_model(new pcl::PointCloud<pcl::PointXYZ>());
		pcl::transformPointCloud(*modelPnts, *rotated_model, rototranslations[i]);

		std::stringstream ss_cloud;
		ss_cloud << "instance" << i;

		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rotated_model_color_handler(rotated_model, 255, 0, 0);
		viewer.addPointCloud(rotated_model, rotated_model_color_handler, ss_cloud.str());

		for (size_t j = 0; j < clustered_corrs[i].size(); ++j)
		{
			std::stringstream ss_line;
			ss_line << "correspondence_line" << i << "_" << j;
			pcl::PointXYZ &model_point = off_scene_model_keypoints->at(clustered_corrs[i][j].index_query);
			pcl::PointXYZ &scene_point = sceneKeypnts->at(clustered_corrs[i][j].index_match);

			//  We are drawing a line for each pair of clustered correspondences found between the model and the scene
			viewer.addLine<pcl::PointXYZ, pcl::PointXYZ>(model_point, scene_point, 0, 255, 0, ss_line.str());
		}
	}

	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}


}

template<typename TpntType, typename TpntNType, typename TdescriptorType>
void matching_histogram(std::string& dataDir, std::string& modelName, std::string& detector, std::string& descriptor) {

    // read filtered scene Clouds
    typename pcl::PointCloud<TpntType>::Ptr scenePnts(new pcl::PointCloud<TpntType>());
    pcl::PointCloud<pcl::Normal>::Ptr sceneNs(new pcl::PointCloud<pcl::Normal>());
	typename pcl::PointCloud<TpntNType>::Ptr scenePntNs(new pcl::PointCloud<TpntNType>());

    std::string sceneCloudPath = dataDir + "/filtered/scene_points_.pcd";
    if (pcl::io::loadPCDFile<TpntType>(sceneCloudPath, *scenePnts) != 0) {
        std::cout << "[Error] Loading PCD file failed" << std::endl;
    }
    sceneCloudPath = dataDir + "/filtered/scene_pointNormals_.pcd";
    if (pcl::io::loadPCDFile<TpntNType>(sceneCloudPath, *scenePntNs) != 0) {
        std::cout << "[Error] Loading PCD file failed" << std::endl;
    }
    sceneCloudPath = dataDir + "/filtered/scene_normals_.pcd";
    if (pcl::io::loadPCDFile<pcl::Normal>(sceneCloudPath, *sceneNs) != 0) {
        std::cout << "[Error] Loading PCD file failed" << std::endl;
    }

    // read filtered model Clouds
    typename pcl::PointCloud<TpntType>::Ptr modelPnts(new pcl::PointCloud<TpntType>());
    pcl::PointCloud<pcl::Normal>::Ptr modelNs(new pcl::PointCloud<pcl::Normal>());
	typename pcl::PointCloud<TpntNType>::Ptr modelPntNs(new pcl::PointCloud<TpntNType>());

    std::string modelCloudPath = dataDir + "/filtered/" + modelName + "_points_.pcd";
    if (pcl::io::loadPCDFile<TpntType>(modelCloudPath, *modelPnts) != 0) {
        std::cout << "[Error] Loading PCD file failed" << std::endl;
    }
    modelCloudPath = dataDir + "/filtered/" + modelName + "_pointNormals_.pcd";
    if (pcl::io::loadPCDFile<TpntNType>(modelCloudPath, *modelPntNs) != 0) {
        std::cout << "[Error] Loading PCD file failed" << std::endl;
    }
    modelCloudPath = dataDir + "/filtered/" + modelName + "_normals_.pcd";
    if (pcl::io::loadPCDFile<pcl::Normal>(modelCloudPath, *modelNs) != 0) {
        std::cout << "[Error] Loading PCD file failed" << std::endl;
    }

    // assign path to read keypoints and descriptor
    typename pcl::PointCloud<TpntType>::Ptr sceneKeypnts(new  pcl::PointCloud<TpntType>);
    typename pcl::PointCloud<TpntType>::Ptr modelKeypnts(new  pcl::PointCloud<TpntType>);
    typename pcl::PointCloud<TdescriptorType>::Ptr sceneDescriptor(new pcl::PointCloud<TdescriptorType>());
    typename pcl::PointCloud<TdescriptorType>::Ptr modelDescriptor(new pcl::PointCloud<TdescriptorType>());   

    std::string sceneKeypntsFilePath;
    std::string modelKeypntsFilePath;
    std::string sceneDescriptorFilePath;
    std::string modelDescriptorFilePath;

    if(detector=="iss" || detector=="sift" || detector=="uni") {
        sceneKeypntsFilePath = dataDir + "/detector/scene_" + detector + "_.pcd";
        modelKeypntsFilePath = dataDir + "/detector/" + modelName + "_" + detector + "_.pcd";
        if(descriptor=="pfh" || descriptor=="fpfh" || descriptor=="vfh" || descriptor=="rsd" ||
           descriptor=="sc3d" || descriptor=="usc" || descriptor=="shot") {
        sceneDescriptorFilePath = dataDir + "/descriptor/" + detector + "/scene_" + descriptor +"_.pcd";
        modelDescriptorFilePath = dataDir + "/descriptor/" + detector + "/" + modelName + "_" + descriptor +"_.pcd";        
        }
        else { ROS_FATAL("unknown descriptor name, please enter only [pfh|fpfh|vfh|rsd|sc3d|usc|shot]");}
    }
    else { ROS_FATAL("unknown detector name, please enter only [iss|sift|uni]");}

    if (pcl::io::loadPCDFile<TpntType>(sceneKeypntsFilePath, *sceneKeypnts) != 0) {
        std::cout << "[Error] Loading PCD file failed" << std::endl;
    }
    if (pcl::io::loadPCDFile<TpntType>(modelKeypntsFilePath, *modelKeypnts) != 0) {
        std::cout << "[Error] Loading PCD file failed" << std::endl;    }

    if (pcl::io::loadPCDFile<TdescriptorType>(sceneDescriptorFilePath, *sceneDescriptor) != 0) {
        std::cout << "[Error] Loading PCD file failed" << std::endl;
    }
    if (pcl::io::loadPCDFile<TdescriptorType>(modelDescriptorFilePath, *modelDescriptor) != 0) {
        std::cout << "[Error] Loading PCD file failed" << std::endl;
    }

    // A kd-tree object that uses the FLANN library for fast search of nearest neighbors.
	pcl::KdTreeFLANN<TdescriptorType> matching;
	matching.setInputCloud(modelDescriptor);
	// A Correspondence object stores the indices of the query and the match,
	// and the distance/weight.
	pcl::CorrespondencesPtr model_scene_corrs(new pcl::Correspondences());

	// Check every descriptor computed for the scene.
	for (size_t i = 0; i < sceneDescriptor->size(); ++i)
	{
		std::vector<int> neighbors(1);
		std::vector<float> squaredDistances(1);
		// Ignore NaNs.
		if (!pcl_isfinite(sceneDescriptor->at(i).histogram[0]))
		    {continue;}
		// Find the nearest neighbor (in descriptor space)...
		int neighborCount = matching.nearestKSearch(sceneDescriptor->at(i), 1, neighbors, squaredDistances);
		// ...and add a new correspondence if the distance is less than a threshold
		// (SHOT distances are between 0 and 1, other descriptors use different metrics).
		if (neighborCount == 1 && squaredDistances[0] < 1.0f) {
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

	rf_est.setInputCloud(modelKeypnts);
	rf_est.setInputNormals(modelNs);
	rf_est.setSearchSurface(modelPnts);
	rf_est.compute(*model_rf);

	rf_est.setInputCloud(sceneKeypnts);
	rf_est.setInputNormals(sceneNs);
	rf_est.setSearchSurface(scenePnts);
	rf_est.compute(*scene_rf);

	//  Clustering
	pcl::Hough3DGrouping<pcl::PointXYZ, pcl::PointXYZ, pcl::ReferenceFrame, pcl::ReferenceFrame> clusterer;
	clusterer.setHoughBinSize(0.01f);
	clusterer.setHoughThreshold(5.0f);
	clusterer.setUseInterpolation(true);
	clusterer.setUseDistanceWeight(false);

	clusterer.setInputCloud(modelKeypnts);
	clusterer.setInputRf(model_rf);
	clusterer.setSceneCloud(sceneKeypnts);
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
	viewer.addPointCloud(scenePnts, "scene_cloud");

	pcl::PointCloud<pcl::PointXYZ>::Ptr off_scene_model(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr off_scene_model_keypoints(new pcl::PointCloud<pcl::PointXYZ>());
	//  We are translating the model so that it doesn't end in the middle of the scene representation
	pcl::transformPointCloud(*modelPnts, *off_scene_model, Eigen::Vector3f(-1, 0, 0), Eigen::Quaternionf(1, 0, 0, 0));
	pcl::transformPointCloud(*modelKeypnts, *off_scene_model_keypoints, Eigen::Vector3f(-1, 0, 0), Eigen::Quaternionf(1, 0, 0, 0));

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> off_scene_model_color_handler(off_scene_model, 255, 255, 128);
	viewer.addPointCloud(off_scene_model, off_scene_model_color_handler, "off_scene_model");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> scene_keypoints_color_handler(sceneKeypnts, 0, 0, 255);
	viewer.addPointCloud(sceneKeypnts, scene_keypoints_color_handler, "scene_keypoints");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "scene_keypoints");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> off_scene_model_keypoints_color_handler(off_scene_model_keypoints, 0, 0, 255);
	viewer.addPointCloud(off_scene_model_keypoints, off_scene_model_keypoints_color_handler, "off_scene_model_keypoints");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "off_scene_model_keypoints");


	for (size_t i = 0; i < rototranslations.size(); ++i)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr rotated_model(new pcl::PointCloud<pcl::PointXYZ>());
		pcl::transformPointCloud(*modelPnts, *rotated_model, rototranslations[i]);

		std::stringstream ss_cloud;
		ss_cloud << "instance" << i;

		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rotated_model_color_handler(rotated_model, 255, 0, 0);
		viewer.addPointCloud(rotated_model, rotated_model_color_handler, ss_cloud.str());

		for (size_t j = 0; j < clustered_corrs[i].size(); ++j)
		{
			std::stringstream ss_line;
			ss_line << "correspondence_line" << i << "_" << j;
			pcl::PointXYZ &model_point = off_scene_model_keypoints->at(clustered_corrs[i][j].index_query);
			pcl::PointXYZ &scene_point = sceneKeypnts->at(clustered_corrs[i][j].index_match);

			//  We are drawing a line for each pair of clustered correspondences found between the model and the scene
			viewer.addLine<pcl::PointXYZ, pcl::PointXYZ>(model_point, scene_point, 0, 255, 0, ss_line.str());
		}
	}

	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}


}


bool matchingSrvCb(ros_pcl_msgs::srv_matching::Request &req, ros_pcl_msgs::srv_matching::Response &res) {
    // initialize
	pcl::PointCloud<pcl::PFHSignature125>::Ptr scenePfhDescriptor(new pcl::PointCloud<pcl::PFHSignature125>());
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr sceneFpfhDescriptor(new pcl::PointCloud<pcl::FPFHSignature33>());
    pcl::PointCloud<pcl::VFHSignature308>::Ptr sceneVfhDescriptor(new pcl::PointCloud<pcl::VFHSignature308>());
    pcl::PointCloud<pcl::PrincipalRadiiRSD>::Ptr sceneRsdDescriptor(new pcl::PointCloud<pcl::PrincipalRadiiRSD>());
    pcl::PointCloud<pcl::ShapeContext1980>::Ptr sceneSc3dDescriptor(new pcl::PointCloud<pcl::ShapeContext1980>());
    pcl::PointCloud<pcl::UniqueShapeContext1960>::Ptr sceneUscDescriptor(new pcl::PointCloud<pcl::UniqueShapeContext1960>());
    pcl::PointCloud<pcl::SHOT352>::Ptr sceneShotDescriptor(new pcl::PointCloud<pcl::SHOT352>());

	pcl::PointCloud<pcl::PFHSignature125>::Ptr modelPfhDescriptor(new pcl::PointCloud<pcl::PFHSignature125>());
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr modelFpfhDescriptor(new pcl::PointCloud<pcl::FPFHSignature33>());
    pcl::PointCloud<pcl::VFHSignature308>::Ptr modelVfhDescriptor(new pcl::PointCloud<pcl::VFHSignature308>());
    pcl::PointCloud<pcl::PrincipalRadiiRSD>::Ptr modelRsdDescriptor(new pcl::PointCloud<pcl::PrincipalRadiiRSD>());
    pcl::PointCloud<pcl::ShapeContext1980>::Ptr modelSc3dDescriptor(new pcl::PointCloud<pcl::ShapeContext1980>());
    pcl::PointCloud<pcl::UniqueShapeContext1960>::Ptr modelUscDescriptor(new pcl::PointCloud<pcl::UniqueShapeContext1960>());
    pcl::PointCloud<pcl::SHOT352>::Ptr modelShotDescriptor(new pcl::PointCloud<pcl::SHOT352>());    

    if (req.descriptor=="pfh"){
        matching_histogram<pntType, pntNType, pcl::PFHSignature125>(req.dataDir, req.modelName, req.detector, req.descriptor);
    }
    else if (req.descriptor=="fpfh"){
        matching_histogram<pntType, pntNType, pcl::FPFHSignature33>(req.dataDir, req.modelName, req.detector, req.descriptor);
    }
    else if (req.descriptor=="vfh"){
        matching_histogram<pntType, pntNType, pcl::VFHSignature308>(req.dataDir, req.modelName, req.detector, req.descriptor);
    }
    else if (req.descriptor=="rsd"){
        //matching_histogram<pntType, pntNType, pcl::PrincipalRadiiRSD>(req.dataDir, req.modelName, req.detector, req.descriptor);
        std::cout << "Not implemented!!" << std::endl;
    }
    else if (req.descriptor=="sc3d"){
        matching_descriptor<pntType, pntNType, pcl::ShapeContext1980>(req.dataDir, req.modelName, req.detector, req.descriptor);
    }
    else if (req.descriptor=="usc"){
        matching_descriptor<pntType, pntNType, pcl::UniqueShapeContext1960>(req.dataDir, req.modelName, req.detector, req.descriptor);
    }
    else if (req.descriptor=="shot"){
        matching_descriptor<pntType, pntNType, pcl::SHOT352>(req.dataDir, req.modelName, req.detector, req.descriptor);
    }
    else {ROS_FATAL("only the descriptors below are implemented [pfh|fpfh|vfh|rsd|sc3d|usc|shot]");}


    return true;
}


int main(int argc, char** argv) {


    ros::init(argc, argv, "matchingSrvServer");

    ros::NodeHandle nh_srv;
    ros::ServiceServer service;
    service = nh_srv.advertiseService("matchingSrvServer", matchingSrvCb);
    ROS_INFO("Matching Service started");
    ROS_INFO("command [dataDir] [modelName] [detector] [descriptor]");
    ros::spin();

}
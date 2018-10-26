#include "../model_prepare.h"

void modelPreprocess(const std::string& dataDir ,const std::string& modelName)
{
	// Object for storing the point cloud.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPoints(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr cloudNormals(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr cloudAll(new pcl::PointCloud<pcl::PointNormal>);
	string descriptorName;
	string keypointsName;

	//-------- read pcd --------//
    string inputName = dataDir + "/srcPCD/" + modelName + ".pcd";

	// Read a PCD file from disk.
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(inputName, *cloudPoints) != 0)
	{
		std::cout << "[Error] Loading PCD file failed" << std::endl;
	}

    string rawname = dataDir + "/filtered/" + modelName;
	string appendName;

	//-------- PCD Preprocessing --------//
	Preprocessor<pntType, pntNType> preprocess;
	preprocess.addPoints(cloudPoints);
	preprocess.doRmNaNPoints();
	preprocess.doVoxelFilter(0.001);
	preprocess.getPoints(cloudPoints);
	double resolution = computeCloudResolution(cloudPoints);
	
	preprocess.doNormalEstimation(true, 2.0*resolution, false);
	preprocess.getData(cloudPoints, cloudNormals, cloudAll);
	appendName = "";
	preprocess.setFileName(rawname,appendName);
	preprocess.saveData();

	Descriptors<pntType, pntNType> descriptor;
	update_detector_fileName(rawname, "");

	// //-------- Keypoints Detector --------//
	// // Assign location to save result
	// filename = dataDir + "/detector/" + modelName +"_.pcd";
    // lastindex = filename.find_last_of(".");
    // rawname = filename.substr(0, lastindex);

	// // ISS keypoint detector object.
	// pcl::PointCloud<pcl::PointXYZ>::Ptr issKeypoints(new pcl::PointCloud<pcl::PointXYZ>);
	// getIssKeypoints(cloudPoints, issKeypoints);

	// // Sift keypoints interest points using normals values from xyz as the Intensity variants
	// pcl::PointCloud<pcl::PointXYZ>::Ptr siftKeypoints(new pcl::PointCloud<pcl::PointXYZ>);
	// getSiftKeypoints(cloudAll, siftKeypoints);

	// // uniSampling keypoints
	// pcl::PointCloud<pcl::PointXYZ>::Ptr uniKeypoints(new pcl::PointCloud<pcl::PointXYZ>);
	// getUniformsamplekeypoints(cloudPoints, uniKeypoints, 6.0);

	// //-------- Features from Keypoints --------//
	// // Assign location to save result
	// filename = srcPcdPath + "/../descriptor/lf064-0" + std::to_string(modelNo) +".pcd";
    // lastindex = filename.find_last_of(".");
    // rawname = filename.substr(0, lastindex);
	
	// // Features from Iss keypoints
	// // FPFH with issKeypoints
	// appendName = "_model_iss";
	// descriptor.setFilename(rawname, appendName);
	// descriptor.setData(cloudPoints, cloudNormals,issKeypoints, cloudAll);
	// pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhDescriptorsModelIss(new pcl::PointCloud<pcl::FPFHSignature33>());
	// descriptor.getFpfh(fpfhDescriptorsModelIss);
	// // Features from Sift Keypoints
	// // SHOT
	// appendName = "_model_sift";
	// descriptor.setFilename(rawname, appendName);
	// descriptor.setData(cloudPoints, cloudNormals, siftKeypoints, cloudAll);
	// pcl::PointCloud<pcl::SHOT352>::Ptr shotDescriptorsModelSift(new pcl::PointCloud<pcl::SHOT352>());
	// descriptor.getShot(shotDescriptorsModelSift);

	// // FPFH
	// descriptor.setData(cloudPoints, cloudNormals,siftKeypoints, cloudAll);
	// pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhDescriptorsModelSift(new pcl::PointCloud<pcl::FPFHSignature33>());
	// descriptor.getFpfh(fpfhDescriptorsModelSift);

	// // Features from unikeypoints
	// // SHOT
	// appendName = "_model_uni";	
	// descriptor.setFilename(rawname, appendName);
	// descriptor.setData(cloudPoints, cloudNormals, uniKeypoints, cloudAll);
	// pcl::PointCloud<pcl::SHOT352>::Ptr shotDescriptorsModelUni(new pcl::PointCloud<pcl::SHOT352>());
	// descriptor.getShot(shotDescriptorsModelUni);

	// // FPFH
	// descriptor.setData(cloudPoints, cloudNormals, uniKeypoints, cloudAll);
	// pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhDescriptorsModelUni(new pcl::PointCloud<pcl::FPFHSignature33>());
	// descriptor.getFpfh(fpfhDescriptorsModelUni);


	std::cout << "[Info] Model preprocess finished" << std::endl;
}

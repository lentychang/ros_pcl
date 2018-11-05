#include "../detectors.h"
using namespace std;

string keypointsName;
string rawName;
string sourceName;
const string subName = "_.pcd";

void update_detector_fileName(std::string filename, std::string source)
{
    rawName = filename;
    sourceName = source;
}

// ISS keypoint detector object.
void getIssKeypoints(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& inputCloudPoints,
                     const pcl::PointCloud<pcl::PointXYZ>::Ptr& issKeypointclouds)
{
    keypointsName = rawName + sourceName + "_iss" + subName;
	pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> detector;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);

    detector.setInputCloud(inputCloudPoints);
	detector.setSearchMethod(kdtree);
	double resolution = computeCloudResolution(inputCloudPoints);
	// Set the radius of the spherical neighborhood used to compute the scatter matrix.
	detector.setSalientRadius(6 * resolution);
	// Set the radius for the application of the non maxima supression algorithm.
	detector.setNonMaxRadius(4 * resolution);
	// Set the minimum number of neighbors that has to be found while applying the non maxima suppression algorithm.
	detector.setMinNeighbors(5);
	// Set the upper bound on the ratio between the second and the first eigenvalue.
	detector.setThreshold21(0.975);
	// Set the upper bound on the ratio between the third and the second eigenvalue.
	detector.setThreshold32(0.975);
	// Set the number of prpcessing threads to use. 0 sets it to automatic.
	detector.setNumberOfThreads(4);
	detector.compute(*issKeypointclouds);
	cout << "Number of non NaNpoints:" << inputCloudPoints->size() << endl;
	cout << "Number of iss keypoints:" << issKeypointclouds->size() << endl;
	pcl::io::savePCDFileASCII(keypointsName, *issKeypointclouds);
}

void getSiftKeypoints(const pcl::PointCloud<pcl::PointNormal>::ConstPtr& inputPointNormals,
                      const pcl::PointCloud<pcl::PointXYZ>::Ptr& siftKeypoints)
{
    // Parameters for sift computation
  	const float min_scale = 0.002f;
	const int n_octaves = 3;
	const int n_scales_per_octave = 4;
	const float min_contrast = 0.0005f;
    keypointsName = rawName + sourceName + "_sift" + subName;

    pcl::SIFTKeypoint<pcl::PointNormal, pcl::PointWithScale> sift;
    pcl::PointCloud<pcl::PointWithScale> keypoints_withRel;
  	pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal> ());
  	sift.setSearchMethod(tree);
  	sift.setScales(min_scale, n_octaves, n_scales_per_octave);
  	sift.setMinimumContrast(min_contrast);
  	sift.setInputCloud(inputPointNormals);
  	sift.compute(keypoints_withRel);
    pcl::io::savePCDFileASCII(keypointsName, keypoints_withRel);
	pcl::copyPointCloud (keypoints_withRel, *siftKeypoints);
	pcl::io::savePCDFileASCII(keypointsName, *siftKeypoints);
	cout << "Number of non NaNpoints:" << inputPointNormals->size() << endl;
	cout << "Number of sift keypoints:" << siftKeypoints->size() << endl;
}


void getSiftKeypoints(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& inputPoints,
                      const pcl::PointCloud<pcl::PointXYZ>::Ptr& siftKeypoints) {
	pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne;
	pcl::PointCloud<pcl::PointNormal>::Ptr cloudPntNormals (new pcl::PointCloud<pcl::PointNormal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_n(new pcl::search::KdTree<pcl::PointXYZ>());

	ne.setInputCloud(inputPoints);
	ne.setSearchMethod(tree_n);
	ne.setRadiusSearch(0.2);
	ne.compute(*cloudPntNormals);
	// Copy the xyz info from cloud_xyz and add it to cloudPntNormals as the xyz field in PointNormals estimation is zero
	for(size_t i = 0; i<cloudPntNormals->points.size(); ++i) {
		cloudPntNormals->points[i].x = inputPoints->points[i].x;
		cloudPntNormals->points[i].y = inputPoints->points[i].y;
		cloudPntNormals->points[i].z = inputPoints->points[i].z;
	}
	getSiftKeypoints(cloudPntNormals, siftKeypoints);
}


void getUniformsamplekeypoints(	const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& inputPoints,
                      			const pcl::PointCloud<pcl::PointXYZ>::Ptr& uniKeypoints,
								double searchRadius){
	pcl::UniformSampling<pcl::PointXYZ> uniform_sampling;
	uniform_sampling.setInputCloud (inputPoints);
	uniform_sampling.setRadiusSearch (searchRadius);
	uniform_sampling.filter (*uniKeypoints);
	cout << "Number of non NaNpoints:" << inputPoints->size() << endl;
	cout << "Number of uniformSampling keypoints:" << uniKeypoints->size() << endl;

	keypointsName = rawName + sourceName + "_uni" + subName;
	pcl::io::savePCDFileASCII(keypointsName, *uniKeypoints);
}
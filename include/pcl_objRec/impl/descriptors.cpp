#include "../descriptors.h"

using namespace std;


template<typename TPntType, typename TPntNType>
Descriptors<TPntType,TPntNType>::Descriptors() {
	__inputPoints = typename  pcl::PointCloud<TPntType>::Ptr(new pcl::PointCloud<TPntType>);
	__inputNormals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
	__keyPoints = typename pcl::PointCloud<TPntType>::Ptr(new pcl::PointCloud<TPntType>);
	__inputPointNormals = typename pcl::PointCloud<TPntNType>::Ptr(new pcl::PointCloud<TPntNType>);
	__kdtree = typename pcl::search::KdTree<TPntType>::Ptr(new pcl::search::KdTree<TPntType> ());
	__number_thread = 3;
	subName.append("_.pcd");
	enableOMP = false;
}

template<typename TPntType, typename TPntNType> 
void Descriptors<TPntType,TPntNType>::setData(	const typename pcl::PointCloud<TPntType>::ConstPtr& inputPoints,
							const pcl::PointCloud<pcl::Normal>::ConstPtr& inputNormals,
							const typename pcl::PointCloud<TPntType>::ConstPtr& keyPoints,
							const typename pcl::PointCloud<TPntNType>::ConstPtr& inputPointNormals ) {
	pcl::copyPointCloud(*inputPoints, *__inputPoints);
	pcl::copyPointCloud(*inputNormals, *__inputNormals);
	pcl::copyPointCloud(*keyPoints, *__keyPoints);
	pcl::copyPointCloud(*inputPointNormals, *__inputPointNormals);
}

template<typename TPntType, typename TPntNType> 
void Descriptors<TPntType,TPntNType>::setFilename(std::string& filename, std::string& source) {
	rawName = filename;
	sourceName = source;
}

// PFH
template<typename TPntType, typename TPntNType> 
void Descriptors<TPntType,TPntNType>::getPfh(const pcl::PointCloud<pcl::PFHSignature125>::Ptr& pfhDescriptor) {
		descriptorName = rawName + sourceName + "_pfh" + subName ;
		pcl::PFHEstimation<TPntType, pcl::Normal, pcl::PFHSignature125> pfh;

		pfh.setSearchSurface(__inputPoints);
		pfh.setInputCloud(__keyPoints);
		pfh.setInputNormals(__inputNormals);
		pfh.setSearchMethod(__kdtree);
		// Search radius, to look for neighbors. Note: the value given here has to be
		// larger than the radius used to estimate the normals.
		pfh.setRadiusSearch(0.04);
		pfh.compute(*pfhDescriptor);
		pcl::io::savePCDFileASCII(descriptorName, *pfhDescriptor);
}

// FPFH
template<typename TPntType, typename TPntNType> 
void Descriptors<TPntType,TPntNType>::getFpfh(const pcl::PointCloud<pcl::FPFHSignature33>::Ptr& fpfhDescriptor) {
	__timeStart();
	descriptorName = rawName + sourceName + "_fpfh" + subName;
	pcl::FPFHEstimation<TPntType, pcl::Normal, pcl::FPFHSignature33> fpfh;
	// if (enableOMP) {
	//  	fpfh.setNumberOfThreads(3);}
	// else {
	// 	pcl::FPFHEstimation<TPntType, pcl::Normal, pcl::FPFHSignature33> fpfh;}
	
	fpfh.setSearchSurface(__inputPoints);
	fpfh.setInputNormals(__inputNormals);
	fpfh.setInputCloud(__keyPoints);
	fpfh.setSearchMethod(__kdtree);
	// Search radius, to look for neighbors. Note: the value given here has to be
	// larger than the radius used to estimate the normals.
	fpfh.setRadiusSearch(0.02);
	fpfh.compute(*fpfhDescriptor);
	pcl::io::savePCDFileASCII(descriptorName, *fpfhDescriptor);
	__printDuration("FPFH Descriptor");
}

template<typename TPntType, typename TPntNType> 
void Descriptors<TPntType,TPntNType>::getVfh(const pcl::PointCloud<pcl::VFHSignature308>::Ptr& vfhDescriptor) {
	pcl::OURCVFHEstimation<TPntType, pcl::Normal, pcl::VFHSignature308> vfh;
	vfh.setSearchSurface(__inputPoints);
	vfh.setInputNormals(__inputNormals);
	vfh.setInputCloud(__keyPoints);
	vfh.setSearchMethod(__kdtree);
	vfh.setEPSAngleThreshold(5.0 / 180.0 * M_PI); // 5 degrees.
	vfh.setCurvatureThreshold(1.0);
	vfh.setNormalizeBins(false);
	// Set the minimum axis ratio between the SGURF axes. At the disambiguation phase,
	// this will decide if additional Reference Frames need to be created, if ambiguous.
	vfh.setAxisRatio(0.8);
	vfh.compute(*vfhDescriptor);
	descriptorName = rawName + sourceName + "_vfh" + subName;
	pcl::io::savePCDFileASCII(descriptorName, *vfhDescriptor);
}

template<typename TPntType, typename TPntNType> 
void Descriptors<TPntType,TPntNType>::getCvfh(const pcl::PointCloud<pcl::VFHSignature308>::Ptr& cvfhDescriptor) {
	pcl::OURCVFHEstimation<TPntType, pcl::Normal, pcl::VFHSignature308> cvfh;
	cvfh.setSearchSurface(__inputPoints);
	cvfh.setInputNormals(__inputNormals);
	cvfh.setInputCloud(__keyPoints);
	cvfh.setSearchMethod(__kdtree);
	cvfh.setEPSAngleThreshold(5.0 / 180.0 * M_PI); // 5 degrees.
	cvfh.setCurvatureThreshold(1.0);
	cvfh.setNormalizeBins(false);
	// Set the minimum axis ratio between the SGURF axes. At the disambiguation phase,
	// this will decide if additional Reference Frames need to be created, if ambiguous.
	cvfh.setAxisRatio(0.8);
	cvfh.compute(*cvfhDescriptor);
	descriptorName = rawName + sourceName + "_cvfh" + subName;
	pcl::io::savePCDFileASCII(descriptorName, *cvfhDescriptor);
}

template<typename TPntType, typename TPntNType> 
void Descriptors<TPntType,TPntNType>::getOurcvfh(const pcl::PointCloud<pcl::VFHSignature308>::Ptr& ourcvfhDescriptor) {
    // OUR-CVFH estimation object.

	pcl::OURCVFHEstimation<TPntType, pcl::Normal, pcl::VFHSignature308> ourcvfh;
	ourcvfh.setSearchSurface(__inputPoints);
	ourcvfh.setInputCloud(__keyPoints);
	ourcvfh.setInputNormals(__inputNormals);
	ourcvfh.setSearchMethod(__kdtree);
	ourcvfh.setEPSAngleThreshold(5.0 / 180.0 * M_PI); // 5 degrees.
	ourcvfh.setCurvatureThreshold(1.0);
	ourcvfh.setNormalizeBins(false);
	// Set the minimum axis ratio between the SGURF axes. At the disambiguation phase,
	// this will decide if additional Reference Frames need to be created, if ambiguous.
	ourcvfh.setAxisRatio(0.8);
	ourcvfh.compute(*ourcvfhDescriptor);
    // Write it back to disk under a different name.
	// Another possibility would be "savePCDFileBinary()".
	descriptorName = rawName + sourceName + "_ourcvfh" + subName;
    pcl::io::savePCDFileASCII(descriptorName, *ourcvfhDescriptor);
}

// RSD estimation object - A large descriptor!!
template<typename TPntType, typename TPntNType> 
void Descriptors<TPntType,TPntNType>::getRsd(const pcl::PointCloud<pcl::PrincipalRadiiRSD>::Ptr& rsdDescriptor) {
	pcl::RSDEstimation<TPntType, pcl::Normal, pcl::PrincipalRadiiRSD> rsd;
	rsd.setSearchSurface(__inputPoints);
	rsd.setInputCloud(__keyPoints);
	rsd.setInputNormals(__inputNormals);
	rsd.setSearchMethod(__kdtree);
	// Search radius, to look for neighbors. Note: the value given here has to be
	// larger than the radius used to estimate the normals.
	rsd.setRadiusSearch(0.05);
	// Plane radius. Any radius larger than this is considered infinite (a plane).
	rsd.setPlaneRadius(0.1);
	// Do we want to save the full distance-angle histograms?
	rsd.setSaveHistograms(false);
	rsd.compute(*rsdDescriptor);

	descriptorName = rawName + sourceName + "_rsd" + subName;
	pcl::io::savePCDFileASCII(descriptorName, *rsdDescriptor);
}

template<typename TPntType, typename TPntNType> 
void Descriptors<TPntType,TPntNType>::getSc3d(const pcl::PointCloud<pcl::ShapeContext1980>::Ptr& sc3dDescriptor){
	pcl::ShapeContext3DEstimation<TPntType, pcl::Normal, pcl::ShapeContext1980> sc3d;
	sc3d.setSearchSurface(__inputPoints);
	sc3d.setInputCloud(__keyPoints);
	sc3d.setInputNormals(__inputNormals);
	sc3d.setSearchMethod(__kdtree);
	// Search radius, to look for neighbors. It will also be the radius of the support sphere.
	sc3d.setRadiusSearch(0.05);
	// The minimal radius value for the search sphere, to avoid being too sensitive
	// in bins close to the center of the sphere.
	sc3d.setMinimalRadius(0.05 / 10.0);
	// Radius used to compute the local point density for the neighbors
	// (the density is the number of points within that radius).
	sc3d.setPointDensityRadius(0.05 / 5.0);

	sc3d.compute(*sc3dDescriptor);
	descriptorName = rawName + sourceName + "_sc3d" + subName;
	pcl::io::savePCDFileASCII(descriptorName, *sc3dDescriptor);
}
// USC estimation object.
template<typename TPntType, typename TPntNType> 
void Descriptors<TPntType,TPntNType>::getUsc(const pcl::PointCloud<pcl::UniqueShapeContext1960>::Ptr& uscDescriptor){
	pcl::UniqueShapeContext<TPntType, pcl::UniqueShapeContext1960, pcl::ReferenceFrame> usc;
	usc.setSearchSurface(__inputPoints);
	usc.setInputCloud(__keyPoints);
	// Search radius, to look for neighbors. It will also be the radius of the support sphere.
	usc.setRadiusSearch(0.05);
	// The minimal radius value for the search sphere, to avoid being too sensitive
	// in bins close to the center of the sphere.
	usc.setMinimalRadius(0.05 / 10.0);
	// Radius used to compute the local point density for the neighbors
	// (the density is the number of points within that radius).
	usc.setPointDensityRadius(0.05 / 5.0);
	// Set the radius to compute the Local Reference Frame.
	usc.setLocalRadius(0.05);
	usc.compute(*uscDescriptor);
	descriptorName = rawName + sourceName + "_usc" + subName;
	pcl::io::savePCDFileASCII(descriptorName, *uscDescriptor);
}

template<typename TPntType, typename TPntNType> 
void Descriptors<TPntType,TPntNType>::getShot(const pcl::PointCloud<pcl::SHOT352>::Ptr& shotDescriptor){
	__timeStart();
	// SHOT estimation object.
	pcl::SHOTEstimationOMP<TPntType, pcl::Normal, pcl::SHOT352> shot;
	shot.setNumberOfThreads(__number_thread);
	shot.setSearchSurface(__inputPoints);
	shot.setInputCloud(__keyPoints);
	shot.setInputNormals(__inputNormals);
	// The radius that defines which of the keypoint's neighbors are described.
	// If too large, there may be clutter, and if too small, not enough points may be found.
	shot.setRadiusSearch(0.02);
	shot.compute(*shotDescriptor);

	vector<int> mapping_;

	descriptorName = rawName + sourceName + "_shot" + subName;
	pcl::io::savePCDFileASCII(descriptorName, *shotDescriptor);
	__printDuration("Descriptor Shot");
}

template<typename TPntType, typename TPntNType> 
template<typename U> 
typename std::enable_if
        <(std::is_same<U, pcl::PointXYZRGB>::value), void>::type 
Descriptors<TPntType,TPntNType>::getColorShot(const pcl::PointCloud<pcl::SHOT1344>::Ptr& shotDescriptor){
	__timeStart();
	// SHOT estimation object.

	pcl::SHOTColorEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::SHOT1344, pcl::ReferenceFrame> shot;
	shot.setNumberOfThreads(__number_thread);
	shot.setSearchSurface(__inputPoints);
	shot.setInputCloud(__keyPoints);
	shot.setInputNormals(__inputNormals);
	// The radius that defines which of the keypoint's neighbors are described.
	// If too large, there may be clutter, and if too small, not enough points may be found.
	shot.setRadiusSearch(0.02);
	shot.compute(*shotDescriptor);

	vector<int> mapping_;

	descriptorName = rawName + sourceName + "_shot" + subName;
	pcl::io::savePCDFileASCII(descriptorName, *shotDescriptor);
	__printDuration("Descriptor Shot");

}

// template<typename TPntType, typename TPntNType> 
// void Descriptors<TPntType,TPntNType>::getSpin(const pcl::PointCloud<pcl::Histogram<153>>::Ptr& spinDescriptor) {
// 	__timeStart();	
// 	pcl::SpinImageEstimation<TPntType, pcl::Normal, pcl::Histogram<153> > spin_image_descriptor(8, 0.5, 16);
// 	spin_image_descriptor.setSearchSurface(__inputPoints);
// 	spin_image_descriptor.setInputCloud (__keyPoints);
// 	spin_image_descriptor.setInputNormals (__inputNormals);
// 	// Use the same KdTree from the normal estimation
// 	spin_image_descriptor.setSearchMethod (__kdtree);
// 	spin_image_descriptor.setRadiusSearch (0.2);

// 	// Actually compute the spin images
// 	spin_image_descriptor.compute (*spinDescriptor);
// 	descriptorName = rawName + sourceName + "_spin" + subName;
// 	pcl::io::savePCDFileASCII(descriptorName, *spinDescriptor);
// 	__printDuration("Descriptor Spin");
// }


template<typename TPntType, typename TPntNType> 
void Descriptors<TPntType,TPntNType>::__timeStart(){__start = std::clock();}
template<typename TPntType, typename TPntNType> 
void Descriptors<TPntType,TPntNType>::__printDuration(const char str[]){
    __duration = ( std::clock() - __start ) / (double) CLOCKS_PER_SEC;
    cout << "Time for " << str << ": " << __duration <<'\n';
  }
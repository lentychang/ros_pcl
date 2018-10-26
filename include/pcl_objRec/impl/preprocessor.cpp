#include "../preprocessor.h"

using namespace std;

template<typename TPointType>
void modelRescaling(const string& filename){
    typename pcl::PointCloud<TPointType>::Ptr cloudPnts(new pcl::PointCloud<TPointType>);
    pcl::io::loadPCDFile<TPointType>(filename, *cloudPnts);

    typename pcl::PointCloud<TPointType>::Ptr newPoints(new pcl::PointCloud<TPointType>);
    typename pcl::PointCloud<TPointType>::PointType pnt;
    for (int i=0; i< cloudPnts->size(); ++i) {
        //cout << cloudPnts->points[i].x << " " << cloudPnts->points[i].y << " " << cloudPnts->points[i].z << endl;
        pnt.x = cloudPnts->points[i].x / 1000.0f;
        pnt.y = cloudPnts->points[i].y / 1000.0f;
        pnt.z = cloudPnts->points[i].z / 1000.0f;
        newPoints->push_back(pnt);
        //cout << pnt.x << " " << pnt.y << " " << pnt.z << endl;
        //cout << newPoints->points[i].x << " " << newPoints->points[i].y << " " << newPoints->points[i].z << endl;
    }
    size_t lastindex = filename.find_last_of("_");
    string rawname = filename.substr(0, lastindex);
    string newFileName = rawname + ".pcd";
    pcl::io::savePCDFileASCII(newFileName ,*newPoints);
}

template<typename TPntType, typename TPntNType>
Preprocessor<TPntType,TPntNType>::Preprocessor() {
    pointsFromCallback = typename pcl::PointCloud<TPntType>::Ptr(new pcl::PointCloud<TPntType>);
    __points = typename pcl::PointCloud<TPntType>::Ptr(new pcl::PointCloud<TPntType>);
    __pointsPool = typename pcl::PointCloud<TPntType>::Ptr(new pcl::PointCloud<TPntType>);
    __cloudPoints = typename pcl::PointCloud<TPntType>::Ptr(new pcl::PointCloud<TPntType>);
    __cloudNormals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
	__pointNormals = typename pcl::PointCloud<TPntNType>::Ptr(new pcl::PointCloud<TPntNType>);

    __kdtree = typename pcl::search::KdTree<TPntType>::Ptr(new pcl::search::KdTree<TPntType>);
    __octree = typename pcl::search::Octree<TPntType>::Ptr(new pcl::search::Octree<TPntType>(0.005));
    __flannkdtree = typename pcl::KdTreeFLANN<TPntType>::Ptr(new pcl::KdTreeFLANN<TPntType>);
    subName = "_.pcd";
    __normalEstimation.setNumberOfThreads(3);
}

template<typename TPntType,typename TPntNType>
void Preprocessor<TPntType,TPntNType>::setCallbackCloud(const typename pcl::PointCloud<TPntType>::Ptr& points_from_callback) {
    pointsFromCallback = points_from_callback->makeShared();
}

template<typename TPntType,typename TPntNType>
void Preprocessor<TPntType,TPntNType>::__updatePoints() {pcl::copyPointCloud(*pointsFromCallback,*__points);}

template<typename TPntType,typename TPntNType>
void Preprocessor<TPntType,TPntNType>::getPointsFromCamera(int n) {
    __timeStart();
    __pointsPool->clear();
    for (int i=0; i< n; ++i){
        __updatePoints();
        *__pointsPool += *__points;
    }
    __printDuration("getPointsFromCamera");
}

template<typename TPntType,typename TPntNType>
void Preprocessor<TPntType,TPntNType>::clearPointsPool() {
    __pointsPool->clear();
}

template<typename TPntType,typename TPntNType>
void Preprocessor<TPntType,TPntNType>::addPoints(typename pcl::PointCloud<TPntType>::Ptr& addPoints) {
    *__pointsPool = (*__pointsPool) + (*addPoints);
}


template<typename TPntType,typename TPntNType>
void Preprocessor<TPntType,TPntNType>::doRmNaNPoints(){
    __timeStart();
    __mapping.clear();
    cout << "Number of points before remove NaN:" << __pointsPool->size() << endl;
    pcl::removeNaNFromPointCloud(*__pointsPool, *__cloudPoints, __mapping);
    cout << "Number of points after remove NaN: " << __cloudPoints->size() << endl;
    __printDuration("rmNaN");
}

template<typename TPntType,typename TPntNType>
void Preprocessor<TPntType,TPntNType>::doZFilter(float min, float max){
    // Filter out all points with Z values not in the [min-max] range.
    __timeStart();
    __zFilter.setInputCloud(__cloudPoints);
	__zFilter.setFilterFieldName("z");
	__zFilter.setFilterLimits(min, max);
	__zFilter.filter(*__cloudPoints);
	cout << "Number of points after Z_dist_filter: " << __cloudPoints->size() << endl;
    __printDuration("ZFilter");
}

template<typename TPntType,typename TPntNType>
void Preprocessor<TPntType,TPntNType>::doStatisticFilter(int meanK, float sigma) {
  __timeStart();
  pcl::StatisticalOutlierRemoval<TPntType> sor;
  sor.setInputCloud (__cloudPoints);
  sor.setMeanK (meanK);
  sor.setStddevMulThresh (sigma);
  sor.filter (*__cloudPoints);
  cout << "Number of points after Statistical Outlier filtering: " << __cloudPoints->size() << endl;
  __printDuration("StatisticalOutlierRemoval");

}

template<typename TPntType,typename TPntNType>
void Preprocessor<TPntType,TPntNType>::doVoxelFilter(float voxelSize) {
    // We set the size of every voxel to be 1x1x1cm
	// (only one point per every cubic centimeter will survive).
    __timeStart();
    __voxelFilter.setInputCloud(__cloudPoints);
	__voxelFilter.setLeafSize(voxelSize, voxelSize, voxelSize);
	__voxelFilter.filter(*__cloudPoints);
    cout << "Number of points after voxelFilter: " << __cloudPoints->size() << endl;
    __printDuration("voxelFilter");
}


template<typename TPntType,typename TPntNType>
void Preprocessor<TPntType,TPntNType>::segRansacPlane(float nonPlnPntsRatio, float distThres) {
    typename pcl::PointCloud<TPntType>::Ptr cloud_planeOutlier (new pcl::PointCloud<TPntType>);
    typename pcl::PointCloud<TPntType>::Ptr cloud_planeInlier (new pcl::PointCloud<TPntType>);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    // Create the segmentation object
    pcl::SACSegmentation<TPntType> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.001);
    pcl::ExtractIndices<TPntType> extract;

    int i = 0, nr_points = (int) __cloudPoints->points.size ();
    std::cout << "input cloud size:" << nr_points << std::endl;
    // While 30% of the original cloud is still there
    while (__cloudPoints->points.size () > nonPlnPntsRatio * nr_points) {
    // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (__cloudPoints);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
        break;
        }
    // Extract the inliers
    std::cout << coefficients->values[0] << "," << coefficients->values[1] << "," << coefficients->values[2] << "," << coefficients->values[3] << std::endl;
    extract.setInputCloud (__cloudPoints);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter(*cloud_planeInlier);

    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_planeOutlier);
    std::cout << "Points left after removing plane:" << cloud_planeOutlier->points.size() << std::endl;
    std::cout << "Points before swapping plane:" << __cloudPoints->points.size() << std::endl;
    __cloudPoints.swap (cloud_planeOutlier);
    std::cout << "Points after swapping plane:" << __cloudPoints->points.size() << std::endl;
    i++;
    }


    //seg.segment (*inliers, *coefficients);

    // Create the filtering object
    //pcl::ExtractIndices<pcl::PointXYZ> extract;



}


template<typename TPntType,typename TPntNType>
void Preprocessor<TPntType,TPntNType>::rmRansacPlane(float distThres) {
    __timeStart();
    // created RandomSampleConsensus object and compute the appropriated model
    typename pcl::SampleConsensusModelPlane<TPntType>::Ptr model_p (new pcl::SampleConsensusModelPlane<TPntType> (__cloudPoints));
    typename pcl::PointCloud<TPntType>::Ptr tempCloud(new pcl::PointCloud<TPntType>);
    typename pcl::PointCloud<TPntType>::Ptr cloud_outliers(new pcl::PointCloud<TPntType>);
    pcl::PointIndices::Ptr plane_inliers (new pcl::PointIndices());
    pcl::RandomSampleConsensus<TPntType> ransac (model_p);
    ransac.setDistanceThreshold (distThres);
    ransac.computeModel();
    ransac.getInliers(plane_inliers->indices);
	if (plane_inliers->indices.size() == 0) {
		PCL_ERROR ("Could not estimate a planar model for the given dataset.\n");
	}
	// Extract outliers
    std::cout << "number of original cloud:"<< __cloudPoints->size() << std::endl;
    std::cout << "number of inlier:"<< plane_inliers->indices.size() << std::endl;


    pcl::ExtractIndices<TPntType> extract;
	extract.setInputCloud (__cloudPoints);
	extract.setIndices (plane_inliers);
	extract.setNegative (true);				// Extract the outliers
	extract.filter (*cloud_outliers);		// cloud_outliers contains everything but the plane
    pcl::copyPointCloud<TPntType>(*cloud_outliers, *__cloudPoints);


    __printDuration("Ransac Plane");
}

template<typename TPntType,typename TPntNType>
void Preprocessor<TPntType,TPntNType>::doRansacPlane(float distThres) {
    __timeStart();
    std::vector<int> plane_inliers;
    // created RandomSampleConsensus object and compute the appropriated model
    typename pcl::SampleConsensusModelPlane<TPntType>::Ptr model_p (new pcl::SampleConsensusModelPlane<TPntType> (__cloudPoints));
    typename pcl::PointCloud<TPntType>::Ptr tempCloud(new pcl::PointCloud<TPntType>);
    pcl::RandomSampleConsensus<TPntType> ransac (model_p);
    ransac.setDistanceThreshold (distThres);
    ransac.computeModel();
    ransac.getInliers(plane_inliers);
	if (plane_inliers.size() == 0) {
		PCL_ERROR ("Could not estimate a planar model for the given dataset.\n");
	}
    __printDuration("Ransac Plane");
}


template<typename TPntType,typename TPntNType>
void Preprocessor<TPntType,TPntNType>::doNormalEstimation(bool useRadiusSearch, double searchParam, bool useOctreeSearch) {
	__timeStart();
    __cloudNormals->clear();
    __pointNormals->clear();
    __normalEstimation.setInputCloud(__cloudPoints);
    if (useRadiusSearch) {
        cout << "[Normal estimation] uses Radius search, radius: " << searchParam << endl;
        __normalEstimation.setRadiusSearch(searchParam);}
    else {
      cout << "[Normal estimation] uses Ksearch, K: " << (int)searchParam << endl;
      __normalEstimation.setKSearch(int(searchParam));}

    //if (strcmp(searchMethod,"flannkdtree")== 0) {__normalEstimation.setSearchMethod(__flannkdtree);}
    if (useOctreeSearch) {__normalEstimation.setSearchMethod(__octree);}
    else {__normalEstimation.setSearchMethod(__kdtree);}

	__normalEstimation.compute(*__cloudNormals);

	// Concatenate the fields (PointXYZ + Normal = PointNormal).
	pcl::concatenateFields(*__cloudPoints, *__cloudNormals, *__pointNormals);

    // remove Nan in Normals
    __mapping.clear();
    cout << "Before remove NaN in Normals:" << __cloudNormals->size() << endl;
    pcl::removeNaNNormalsFromPointCloud(*__cloudNormals, *__cloudNormals , __mapping);
    cout << "After remove NaN in Normals:" << __cloudNormals->size() << endl;
    __mapping.clear();
	pcl::removeNaNNormalsFromPointCloud(*__pointNormals, *__pointNormals, __mapping);

    pcl::PointIndices::Ptr pointIndices(new pcl::PointIndices);
    pointIndices->indices = __mapping;
	// Object for extracting points from a list of indices.
	pcl::ExtractIndices<TPntType> extract;
	extract.setInputCloud(__cloudPoints);
	extract.setIndices(pointIndices);
	// We will extract the points that are indexed.
	extract.setNegative(false);
	extract.filter(*__cloudPoints);
    __printDuration("Normal estimation");
}

template<typename TPntType,typename TPntNType>
void Preprocessor<TPntType,TPntNType>::doMlsSmoothing(const bool computeNormals, int polynomialOrder) {
    __timeStart();
    // Output has the PointNormal type in order to store the normals calculated by MLS
    pcl::PointCloud<TPntNType> mls_points;
    // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<TPntType, TPntNType> mls;
    
    mls.setComputeNormals (computeNormals);
    // Set parameters
    mls.setInputCloud (__cloudPoints);
    mls.setPolynomialOrder(polynomialOrder);
    //mls.setPolynomialFit (true);
    mls.setSearchMethod (__kdtree);
    mls.setSearchRadius (0.02);
    // Reconstruct
    mls.process (mls_points);
    __cloudPoints->clear();
    __cloudNormals->clear();
    __pointNormals->clear();
    pcl::copyPointCloud(mls_points, *__cloudPoints);
    pcl::copyPointCloud(mls_points, *__cloudNormals);
    pcl::copyPointCloud(mls_points, *__pointNormals);
    __printDuration("MLS Smoothing");
}

template<typename TPntType,typename TPntNType>
void Preprocessor<TPntType,TPntNType>::getData(const typename pcl::PointCloud<TPntType>::Ptr& cloudPnts,
                           const pcl::PointCloud<pcl::Normal>::Ptr& cloudNormals,
                           const typename pcl::PointCloud<TPntNType>::Ptr& pntNormals) {
    pcl::copyPointCloud(*__cloudPoints, *cloudPnts);
    pcl::copyPointCloud(*__cloudNormals, *cloudNormals);
    pcl::copyPointCloud(*__pointNormals, *pntNormals);
}

template<typename TPntType,typename TPntNType>
void Preprocessor<TPntType,TPntNType>::getPoints(const typename pcl::PointCloud<TPntType>::Ptr& cloudPnts) {
    pcl::copyPointCloud(*__cloudPoints, *cloudPnts);
}

template<typename TPntType,typename TPntNType>
void Preprocessor<TPntType,TPntNType>::setFileName(std::string& rawname, std::string& srcname) {
    rawName = rawname;  sourceName = srcname;
}

template<typename TPntType,typename TPntNType>
void Preprocessor<TPntType,TPntNType>::saveData() {
    fileName = rawName + sourceName + "_points" + subName;
    std::cout << fileName << std::endl;
    pcl::io::savePCDFileASCII(fileName ,*__cloudPoints);
    fileName = rawName + sourceName + "_normals" + subName;
    pcl::io::savePCDFileASCII(fileName ,*__cloudNormals);
    fileName = rawName + sourceName + "_pointNormals" + subName;
    pcl::io::savePCDFileASCII(fileName ,*__pointNormals);

}

template<typename TPntType,typename TPntNType>
void Preprocessor<TPntType,TPntNType>::savePoints() {
    fileName = rawName + sourceName + "_points" + subName;
    pcl::io::savePCDFileASCII(fileName ,*__cloudPoints);
}

template<typename TPntType,typename TPntNType>
void Preprocessor<TPntType,TPntNType>::__timeStart(){__start = std::clock();}

template<typename TPntType,typename TPntNType>
void Preprocessor<TPntType,TPntNType>::__printDuration(const char str[]){
    __duration = ( std::clock() - __start ) / (double) CLOCKS_PER_SEC;
    cout << "Time for " << str << ": " << __duration <<'\n';
  }


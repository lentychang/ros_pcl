#ifndef PREPROCESSOR_H
#define PREPROCESSOR_H

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/time.h>

#include <pcl/search/kdtree.h>
#include <pcl/search/octree.h>
#include <pcl/search/search.h>
#include <pcl/search/flann_search.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/mls.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/ModelCoefficients.h>
#include "../../config/pointType.hpp"

#include <boost/thread/thread.hpp>
#include <iostream>
#include <string>
#include <cstdio>
#include <ctime>

template<typename TPointType>
void modelRescaling(const std::string& filename);

template<typename TPntType, typename TPntNType>
class Preprocessor {
private:
    typename pcl::PointCloud<TPntType>::Ptr __pointsPool;
    typename pcl::PointCloud<TPntType>::Ptr __points;
    typename pcl::PointCloud<TPntType>::Ptr __cloudPoints;
    pcl::PointCloud<pcl::Normal>::Ptr __cloudNormals;
	typename pcl::PointCloud<TPntNType>::Ptr __pointNormals;
    std::clock_t __start;
    double __duration;

    std::vector<int> __mapping;
    typename pcl::PassThrough<TPntType> __zFilter;
    typename pcl::VoxelGrid<TPntType> __voxelFilter;
    typename pcl::NormalEstimationOMP<TPntType, pcl::Normal> __normalEstimation;
    typename pcl::search::KdTree<TPntType>::Ptr __kdtree;
    typename pcl::search::Octree<TPntType>::Ptr __octree;
    typename pcl::KdTreeFLANN<TPntType>::Ptr __flannkdtree;

    void __updatePoints();
    void __collectMorePoints();
    void __timeStart();
    void __printDuration(const char str[]);

    std::string fileName;
    std::string rawName;
    std::string sourceName;
    std::string subName;

public:
    Preprocessor();
    typename pcl::PointCloud<TPntType>::Ptr pointsFromCallback;
    void addPoints(typename pcl::PointCloud<TPntType>::Ptr& addPoints);
    void setCallbackCloud(const typename pcl::PointCloud<TPntType>::Ptr& points_from_callback);
    void clearPointsPool();
    void getPointsFromCamera(int n=5);
    void doRmNaNPoints();
    void doZFilter(float min=0.5, float max=2.0);
    void doStatisticFilter(int meanK=20, float sigma=1.0);
    void doVoxelFilter(float voxelSize=0.001);
    void segRansacPlane(float nonPlnPntsRatio=0.2, float distThre=0.001);
    void rmRansacPlane(float distThres=0.001);
    void doRansacPlane(float distThres = 0.01);
    void doNormalEstimation(bool useRadiusSearch=true, double searchParam=0.002, bool useOctreeSearch=false);
    void doMlsSmoothing(const bool computeNormals=false, int polynomialOrder=3);
    void getData(   const typename pcl::PointCloud<TPntType>::Ptr& cloudPnts,
                    const pcl::PointCloud<pcl::Normal>::Ptr& cloudNormals,
                    const typename pcl::PointCloud<TPntNType>::Ptr& pntNormals);
    void getPoints(const typename pcl::PointCloud<TPntType>::Ptr& cloudPnts);
    void setFileName(std::string& rawname, std::string& srcname);
    void saveData();
    void savePoints();

};


template class Preprocessor<pcl::PointXYZ,pcl::PointNormal>;
template class Preprocessor<pcl::PointXYZRGB,pcl::PointXYZRGBNormal>;
#endif

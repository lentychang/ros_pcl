#ifndef DESCRIPTORS_H
#define DESCRIPTORS_H

#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/our_cvfh.h>
#include <pcl/features/cvfh.h>
#include <pcl/features/vfh.h>
#include <pcl/features/pfh.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/rsd.h>
#include <pcl/features/3dsc.h>
#include <pcl/features/usc.h>
#include <pcl/features/shot.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/spin_image.h>
#include <pcl_objRec/utils.h>


#include <pcl/io/pcd_io.h>
#include <pcl/common/io.h>

#include <boost/thread/thread.hpp>
#include <iostream>
#include <string>
#include <cstdio>
#include <ctime>
#include <type_traits>

template<typename TPntType, typename TPntNType>
class Descriptors {
private:
    typename pcl::PointCloud<TPntType>::Ptr __inputPoints;
    pcl::PointCloud<pcl::Normal>::Ptr __inputNormals;
    typename pcl::PointCloud<TPntType>::Ptr __keyPoints;
	typename pcl::PointCloud<TPntNType>::Ptr __inputPointNormals;
	typename pcl::search::KdTree<TPntType>::Ptr __kdtree;
    std::string descriptorName;
    std::string rawName;
    std::string sourceName;
    bool enableOMP;
    std::string subName;
    std::clock_t __start;
    double __duration;
    void __timeStart();
    void __printDuration(const char str[]);
    int __number_thread;
public:
    Descriptors();
    void setData(   const typename pcl::PointCloud<TPntType>::ConstPtr& inputPoints,
			        const pcl::PointCloud<pcl::Normal>::ConstPtr& inputNormals,
				    const typename pcl::PointCloud<TPntType>::ConstPtr& keyPoints,
				    const typename pcl::PointCloud<TPntNType>::ConstPtr& inputPointNormals );
    void setFilename(std::string& filename, std::string& source);
    void getPfh(const pcl::PointCloud<pcl::PFHSignature125>::Ptr& pfhDescriptor);
    void getFpfh(const pcl::PointCloud<pcl::FPFHSignature33>::Ptr& fpfhDescriptor);
    void getVfh(const pcl::PointCloud<pcl::VFHSignature308>::Ptr& vfhDescriptor);
    void getCvfh(const pcl::PointCloud<pcl::VFHSignature308>::Ptr& cvfhDescriptor);
    void getOurcvfh(const pcl::PointCloud<pcl::VFHSignature308>::Ptr& ourcvfhDescriptor);
    void getRsd(const pcl::PointCloud<pcl::PrincipalRadiiRSD>::Ptr& rsdDescriptor);
    void getSc3d(const pcl::PointCloud<pcl::ShapeContext1980>::Ptr& sc3dDescriptor);
    void getUsc(const pcl::PointCloud<pcl::UniqueShapeContext1960>::Ptr& uscDescriptor);
    void getShot(const pcl::PointCloud<pcl::SHOT352>::Ptr& shotDescriptor);

    template<typename U = TPntType>
    typename std::enable_if
        <(std::is_same<U, pcl::PointXYZRGB>::value), void>::type
    getColorShot(const pcl::PointCloud<pcl::SHOT1344>::Ptr& shotDescriptor);
    //void getSpin(const pcl::PointCloud<pcl::Histogram<153>>::Ptr& spinDescriptor);

};


template class Descriptors<pcl::PointXYZ,pcl::PointNormal>;
template class Descriptors<pcl::PointXYZRGB,pcl::PointXYZRGBNormal>;


#endif
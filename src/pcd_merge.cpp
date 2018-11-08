#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include "../config/pointType.hpp"
#include <pcl/common/transforms.h>
#include <string>

Eigen::Matrix<double, 3, 1> getTranslation(const pcl::PointCloud<pntType>& cloud_in) {
    Eigen::Matrix<double, 3, 1> translation;
    translation.x() = cloud_in.sensor_origin_[0];
    translation.y() = cloud_in.sensor_origin_[1];
    translation.z() = cloud_in.sensor_origin_[2];
    return translation;
}

Eigen::Quaternion<double> getRotation(const pcl::PointCloud<pntType>& cloud_in) {
    Eigen::Quaternion<double> rotation;
    rotation.x() = cloud_in.sensor_orientation_.x();
    rotation.y() = cloud_in.sensor_orientation_.y();
    rotation.z() = cloud_in.sensor_orientation_.z();
    rotation.w() = cloud_in.sensor_orientation_.w();
    return rotation;
}

void setViewpoint(pcl::PointCloud<pntType>& cloud_in) {
    cloud_in.sensor_origin_[0] = 0;
    cloud_in.sensor_origin_[1] = 0;
    cloud_in.sensor_origin_[2] = 0;
    cloud_in.sensor_orientation_.x() = 0;
    cloud_in.sensor_orientation_.y() = 0;
    cloud_in.sensor_orientation_.z() = 0;
    cloud_in.sensor_orientation_.w() = 1.0;
}

int main(int argc, char** argv) {
    std::string fileDir = argv[1];
    std::string fileName1 = argv[2];
    std::string fileName2 = argv[3];
    pcl::PointCloud<pntType>::Ptr cloudPnts(new pcl::PointCloud<pntType>());
    pcl::PointCloud<pntType>::Ptr cloudPntsTrf(new pcl::PointCloud<pntType>());
    if (pcl::io::loadPCDFile<pntType>((fileDir + "/" + fileName1 + ".pcd"), *cloudPnts) != 0) {
        std::cout << "[Error] Loading PCD file failed" << std::endl;
    }

    pcl::PointCloud<pntType>::Ptr cloudPnts2(new pcl::PointCloud<pntType>());
    pcl::PointCloud<pntType>::Ptr cloudPntsTrf2(new pcl::PointCloud<pntType>());
    if (pcl::io::loadPCDFile<pntType>((fileDir + "/" + fileName2 + ".pcd"), *cloudPnts2) != 0) {
        std::cout << "[Error] Loading PCD file failed" << std::endl;
    }

    pcl::transformPointCloud(*cloudPnts, *cloudPntsTrf, getTranslation(*cloudPnts), getRotation(*cloudPnts));
    setViewpoint(*cloudPntsTrf);
    pcl::io::savePCDFileASCII(fileDir + "/" + fileName1 + "_trf.pcd", *cloudPntsTrf);

    pcl::transformPointCloud(*cloudPnts2, *cloudPntsTrf2, getTranslation(*cloudPnts2), getRotation(*cloudPnts2));
    setViewpoint(*cloudPntsTrf2);
    pcl::io::savePCDFileASCII(fileDir + "/" + fileName2 + "_trf.pcd", *cloudPntsTrf);

    *cloudPntsTrf += *cloudPntsTrf2;
    setViewpoint(*cloudPntsTrf);
    pcl::io::savePCDFileASCII(fileDir + "/" + fileName1 + "_merged.pcd", *cloudPntsTrf);
}
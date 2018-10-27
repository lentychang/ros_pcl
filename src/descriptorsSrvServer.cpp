#include <ros/ros.h>
#include <pcl_ros/io/pcd_io.h>
#include <pcl_objRec/descriptors.h>
#include <ros_pcl_msgs/srv_descriptors.h>
#include <pcl_objRec/utils.h>
#include "../config/pointType.hpp"

// rosservice call /descriptorSrvServer "{keypntType: 'iss', getPfh: true, getFpfh: true, getVfh: true, getRsd: true, getSc3d: true,   getUsc: true, getShot: true, filename: 'scene', inputCloudDir: '/root/exchange/tempData/filtered', inputKeypntsDir: '/root/exchange/tempData/detector',  outputDir: '/root/exchange/tempData/descriptor'}"
bool descriptorSrvCb(ros_pcl::srv_descriptors::Request &req, ros_pcl::srv_descriptors::Response &res) {
    ROS_INFO("Service descriptor is called!");
    res.success=true;

	pcl::PointCloud<pntType>::Ptr cloudPnts(new pcl::PointCloud<pntType>());
    pcl::PointCloud<pntType>::Ptr keyPnts(new pcl::PointCloud<pntType>());
    pcl::PointCloud<pcl::Normal>::Ptr cloudNs(new pcl::PointCloud<pcl::Normal>());
	pcl::PointCloud<pntNType>::Ptr cloudPntNs(new pcl::PointCloud<pntNType>());
    std::string rawName;

    std::string cloudPath = req.dataDir + "/filtered/" + req.filename + "_points_.pcd";
    if (pcl::io::loadPCDFile<pntType>(cloudPath, *cloudPnts) != 0) {
        std::cout << "[Error] Loading PCD file failed" << std::endl;
    }
    cloudPath = req.dataDir + "/filtered/" + req.filename + "_pointNormals_.pcd";
    if (pcl::io::loadPCDFile<pntNType>(cloudPath, *cloudPntNs) != 0) {
        std::cout << "[Error] Loading PCD file failed" << std::endl;
    }
    cloudPath = req.dataDir + "/filtered/" + req.filename + "_normals_.pcd";
    if (pcl::io::loadPCDFile<pcl::Normal>(cloudPath, *cloudNs) != 0) {
        std::cout << "[Error] Loading PCD file failed" << std::endl;
    }

    std::string keyPntsFile;
    if (req.keypntType=="iss" || req.keypntType=="ISS") {
        keyPntsFile = req.dataDir + "/detector/" + req.filename + "_iss_.pcd";
        rawName = req.dataDir + "/descriptor/iss/" + req.filename;
        }
    else if (req.keypntType=="sift" || req.keypntType=="sift") {
        keyPntsFile = req.dataDir + "/detector/" + req.filename + "_sift_.pcd";
        rawName = req.dataDir + "/descriptor/sift/" + req.filename;
        }
    else {
        keyPntsFile = req.dataDir + "/detector/" + req.filename + "_uni_.pcd";
        rawName = req.dataDir + "/descriptor/uni/" + req.filename;}

    if (pcl::io::loadPCDFile<pntType>(keyPntsFile, *keyPnts) != 0) {
        std::cout << "[Error] Loading PCD file failed" << std::endl;
    }
    float resolution = computeCloudResolution(cloudPnts);
    Descriptors<pntType,pntNType> descriptor;

    std::string appendName = "";
    std::cout << "rawName:" << rawName << std::endl;
    descriptor.setFilename(rawName, appendName);
	descriptor.setData(cloudPnts, cloudNs, keyPnts, cloudPntNs);

	pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhDescriptor(new pcl::PointCloud<pcl::PFHSignature125>());
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhDescriptor(new pcl::PointCloud<pcl::FPFHSignature33>());
    pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhDescriptor(new pcl::PointCloud<pcl::VFHSignature308>());
    pcl::PointCloud<pcl::PrincipalRadiiRSD>::Ptr rsdDescriptor(new pcl::PointCloud<pcl::PrincipalRadiiRSD>());
    pcl::PointCloud<pcl::ShapeContext1980>::Ptr sc3dDescriptor(new pcl::PointCloud<pcl::ShapeContext1980>());
    pcl::PointCloud<pcl::UniqueShapeContext1960>::Ptr uscDescriptor(new pcl::PointCloud<pcl::UniqueShapeContext1960>());
    pcl::PointCloud<pcl::SHOT352>::Ptr shotDescriptor(new pcl::PointCloud<pcl::SHOT352>());

    if (req.getPfh==true) descriptor.getPfh(pfhDescriptor,true, 3*resolution);
    if (req.getFpfh==true) descriptor.getFpfh(fpfhDescriptor, true, 3*resolution);
    if (req.getVfh==true) descriptor.getVfh(vfhDescriptor);
    if (req.getRsd==true) descriptor.getRsd(rsdDescriptor);
    if (req.getSc3d==true) descriptor.getSc3d(sc3dDescriptor);
    if (req.getUsc==true) descriptor.getUsc(uscDescriptor);
    if (req.getShot==true) descriptor.getShot(shotDescriptor, true, 3*resolution);

    return true;
}


int main(int argc, char** argv) {

    ros::init(argc, argv, "descriptorSrvServer");

    ros::NodeHandle nh_srv;
    ros::ServiceServer service;
    service = nh_srv.advertiseService("descriptorSrvServer", descriptorSrvCb);
    ROS_INFO("Descriptor Service started");
    ROS_INFO("Command instruction:");
    std::cout << "[keypntType='iss/sift/uni'] [getPfh=true] [getFpfh=true] [getVfh=true]" << std::endl;
    std::cout << "[getRsd=true] [getSc3d=true] [getUsc=true] [getShot=true]" << std::endl;
    std::cout << "[dataDir] [filename='lf064-0X | scene']" << std::endl;
    ros::spin();

}
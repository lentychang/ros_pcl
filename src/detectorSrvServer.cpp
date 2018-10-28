
#include <ros/ros.h>
#include <ros_pcl_msgs/srv_detectors.h>
#include <pcl_objRec/detectors.h>
#include <pcl/io/pcd_io.h>
#include <pcl_objRec/utils.h>
#include "../config/pointType.hpp"

//rosservice call /getDetectors "filename:'lf064-05' pntInputPath:'/root/exchange/tempData/filtered/lf064-05_model_points_.pcd' pntNormalInputPath:'/root/exchange/tempData/filtered/lf064-05_model_pointNormals_.pcd' outputFolderPath:'/root/exchange/tempData/detector'" 


//rosservice call /getDetectors "filename: 'scene'pntInputPath: '/root/exchange/tempData/filtered/scene_points_.pcd' pntNormalInputPath: '/root/exchange/tempData/filtered/scene_pointNormals_.pcd' outputFolderPath: '/root/exchange/tempData/detector'" 
bool detectorSrvCb(ros_pcl_msgs::srv_detectors::Request &req, ros_pcl_msgs::srv_detectors::Response &res) {
    res.success=true;
	pcl::PointCloud<pntType>::Ptr cloud_pnts(new pcl::PointCloud<pntType>());
	pcl::PointCloud<pntNType>::Ptr cloudpntNormals(new pcl::PointCloud<pntNType>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr issKeypointclouds(new  pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr siftKeypointclouds(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr uniKeypointclouds(new pcl::PointCloud<pcl::PointXYZ>);

    std::string filePath;
    filePath = req.dataDir + "/filtered/" + req.filename + "_points_.pcd";
    if (pcl::io::loadPCDFile<pntType>(filePath, *cloud_pnts) != 0) {
        std::cout << "[Error] Loading PCD file failed" << std::endl;
    }
    filePath = req.dataDir + "/filtered/" + req.filename + "_pointNormals_.pcd";
    if (pcl::io::loadPCDFile<pntNType>(filePath, *cloudpntNormals) != 0) {
        std::cout << "[Error] Loading PCD file failed" << std::endl;
    }

    filePath = req.dataDir + "/detector/" + req.filename;
    update_detector_fileName(filePath, "");
    getIssKeypoints(cloud_pnts, issKeypointclouds);
    getSiftKeypoints(cloudpntNormals, siftKeypointclouds);
    float resolution = computeCloudResolution(cloud_pnts);
    getUniformsamplekeypoints(cloud_pnts,uniKeypointclouds, 3.0*resolution);

    return true;

}


int main(int argc, char** argv) {


    ros::init(argc, argv, "detectorSrvServer");

    ros::NodeHandle nh_srv;
    ros::ServiceServer service;
    service = nh_srv.advertiseService("detectorSrvServer", detectorSrvCb);
    ROS_INFO("detector Service started");
    ROS_INFO("command [dataDir] [filename]");
    ros::spin();

}
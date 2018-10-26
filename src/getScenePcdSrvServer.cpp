#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <boost/thread/thread.hpp>
#include <boost/bind/bind.hpp>
#include <iostream>
#include "std_msgs/String.h"
#include <ros_pcl/srv_getScenePcd.h>
#include <tf2/transform_datatypes.h>
#include "../config/pointType.hpp"
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>


// ** tranform a point cloud2 **
// http://docs.ros.org/indigo/api/pcl_ros/html/namespacepcl__ros.html#aad1ce4ad90ab784aae6158419ad54d5f
// think about also save pcd file with view Point then the frame will become "world"
// pntType is defined in pointType.hpp
typedef pcl::PointCloud<pntType> PointCloud;


class PcdGrabber{
protected:
    ros::NodeHandle nh_sub;
    ros::NodeHandle nh_srv;
    ros::NodeHandle nh_tf2Sub;
    PointCloud::Ptr __cloudPoints;

private:
    geometry_msgs::TransformStamped __transformStamped;


public:
    ros::MultiThreadedSpinner spinner;
    ros::Subscriber subscriber;
    ros::Subscriber subscriber2;
    ros::ServiceServer service;
    tf2_ros::Buffer __tfBuffer;

    PcdGrabber();
    PointCloud::Ptr cloudPoints;
    void callback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void catchCallbackPcdOnce(ros_pcl::srv_getScenePcd::Request &req);
    void appendNewPointCloud();
    void savePCD(const char fileName[]);
    void sub();
    bool subPCDStart(ros_pcl::srv_getScenePcd::Request &req, ros_pcl::srv_getScenePcd::Response &res);
    void getTf2(ros_pcl::srv_getScenePcd::Request &req);
    void tf2callback(const geometry_msgs::TransformStamped &msg);
};


//bool getPCDSrv(ros_pcl::Request &req, ros_pcl::Response &res);

//void callback(const sensor_msgs::PointCloud2::ConstPtr& );
 
int main(int argc, char** argv)
{
    std::cout << "please enter the number of how many times you want to collect PointCloud" << std::endl;

    ros::init(argc, argv, "getScenePcdSrvServer");
    PcdGrabber grabber;
    // tf2_ros::Buffer tfBuffer__;
    // tf2_ros::TransformListener tfListener__(tfBuffer__);
    // geometry_msgs::TransformStamped transformStamped__;

    // for (int i=0;i<=20;++i){
    // transformStamped__ = tfBuffer__.lookupTransform("world", "kinect2_depth_frame",
    //                                 ros::Time(0), ros::Duration(3.0));
    // ROS_INFO("%s",tfBuffer__.allFramesAsString().c_str());
    // ROS_INFO("HIIIIII");
    // ros::Duration(0.25).sleep();
    // }
    // ROS_INFO("%s",tfBuffer__.allFramesAsString().c_str());

    std::cout << "Start to Spin" << std::endl;
    grabber.spinner.spin();
    return 0;

}

PcdGrabber::PcdGrabber() {
    // ros::MultiThreadedSpinner spinner(4);
    __cloudPoints = PointCloud::Ptr(new PointCloud); 
    cloudPoints = PointCloud::Ptr(new PointCloud);
    service = nh_srv.advertiseService("getScenePcdSrvServer", &PcdGrabber::subPCDStart, this);
    tf2_ros::TransformListener __tfListener(__tfBuffer);
    std::cout << "Instantiated" << std::endl;
}

void PcdGrabber::sub() {
    ROS_INFO("Start subscribe pointCloud");
    spinner.spin();
}

bool PcdGrabber::subPCDStart(ros_pcl::srv_getScenePcd::Request &req, ros_pcl::srv_getScenePcd::Response &res) {
    std::cout << "SrvCalled" << std::endl;
    subscriber = nh_sub.subscribe<sensor_msgs::PointCloud2>("/kinect2/sd/points", 1, &PcdGrabber::callback, this);
    subscriber2 = nh_tf2Sub.subscribe("/tf2_kinect2_world", 1, &PcdGrabber::tf2callback, this);

    boost::thread subThread(&PcdGrabber::sub, this);

    boost::this_thread::sleep(boost::posix_time::seconds(2));
    catchCallbackPcdOnce(req);
    // appendNewPointCloud();
    subscriber.shutdown();
    subscriber2.shutdown();
    ROS_INFO("Stop subscribing PointCloud");
    
    //  pcl::PassThrough<pcl::PointXYZRGB> zFilter;

    //  std::vector<int> mapping;
    //  pcl::removeNaNFromPointCloud(*cloudPoints, *cloudPoints, mapping);
    //  zFilter.setInputCloud(cloudPoints);
    //  zFilter.setFilterFieldName("z");
    //  zFilter.setFilterLimits(0.5, 1.5);
    //  zFilter.filter(*cloudPoints);

    ROS_INFO("Collected %d points from Camera", static_cast<int>(cloudPoints->size()));
    std::string outputPath = req.dataDir + "/srcPCD/" + req.pcdName +".pcd";
    savePCD(outputPath.c_str());
    ROS_INFO("scenePCD saved!");
    res.success = true;
    return true;
}

void PcdGrabber::callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
    // Container for original & filtered data
    pcl::fromROSMsg(*msg, *__cloudPoints);
    // boost::this_thread::interruption_point();
}

void PcdGrabber::tf2callback(const geometry_msgs::TransformStamped &msg) {
    __transformStamped = msg;
    ROS_INFO("translation.x %f",__transformStamped.transform.translation.x);
    ROS_INFO("rotation.x: %f",__transformStamped.transform.rotation.x);
    ROS_INFO("tf2 subscribing");
}



void PcdGrabber::catchCallbackPcdOnce(ros_pcl::srv_getScenePcd::Request &req) {
    printf("catchCallbackPcdOnce called !!");
    pcl::copyPointCloud(*__cloudPoints, *cloudPoints);
    //getTf2(req);
    cloudPoints->sensor_origin_[0] = __transformStamped.transform.translation.x;
    cloudPoints->sensor_origin_[1] = __transformStamped.transform.translation.y;
    cloudPoints->sensor_origin_[2] = __transformStamped.transform.translation.z;
    cloudPoints->sensor_orientation_.x () = __transformStamped.transform.rotation.x;
    cloudPoints->sensor_orientation_.y () = __transformStamped.transform.rotation.y;
    cloudPoints->sensor_orientation_.z () = __transformStamped.transform.rotation.z;
    cloudPoints->sensor_orientation_.w () = __transformStamped.transform.rotation.w;
}

void PcdGrabber::appendNewPointCloud() {
    *cloudPoints = (*__cloudPoints) +(*cloudPoints);
}

void PcdGrabber::savePCD(const char fileName[]){
    pcl::io::savePCDFileASCII(fileName ,*cloudPoints);
}


void PcdGrabber::getTf2(ros_pcl::srv_getScenePcd::Request &req){
   
        std::cout << "In getTf2" << std::endl;
        while (nh_sub.ok()) {
            __transformStamped = __tfBuffer.lookupTransform(req.targetFrame, req.sourceFrame,
                                            ros::Time(0), ros::Duration(3.0));
            ROS_INFO("%s",__tfBuffer.allFramesAsString().c_str());
            ROS_INFO("HIIIIII");

            try{
                __transformStamped = __tfBuffer.lookupTransform(req.targetFrame, req.sourceFrame,
                                            ros::Time(0), ros::Duration(3.0));
                                        
            }
            catch (tf2::TransformException &ex) {
                ROS_WARN("%s",ex.what());
                ros::Duration(0.5).sleep();
            }
            ros::Duration(0.5).sleep();

        }

    std::cout << "subscriber of point cloud is shut down !" << std::endl;
}
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
#include "../config/pointType.hpp"

// pntType is defined in pointType.hpp
typedef pcl::PointCloud<pntType> PointCloud;


class PcdGrabber{
protected:
    ros::NodeHandle nh_sub;
    ros::NodeHandle nh_srv;
    PointCloud::Ptr __cloudPoints;
public:
    ros::MultiThreadedSpinner spinner;
    ros::Subscriber subscriber;
    ros::ServiceServer service;

    PcdGrabber();
    PointCloud::Ptr cloudPoints;
    void callback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void catchCallbackPcdOnce();
    void appendNewPointCloud();
    void savePCD(const char fileName[]);
    void sub();
    bool subPCDStart(ros_pcl::srv_getScenePcd::Request &req, ros_pcl::srv_getScenePcd::Response &res);
};


//bool getPCDSrv(ros_pcl::Request &req, ros_pcl::Response &res);

//void callback(const sensor_msgs::PointCloud2::ConstPtr& );
 
int main(int argc, char** argv)
{
    std::cout << "please enter the number of how many times you want to collect PointCloud" << std::endl;

    ros::init(argc, argv, "getScenePcdSrvServer");
    PcdGrabber grabber;
    std::cout << "Start to Spin" << std::endl;
    grabber.spinner.spin();
    return 0;

}

PcdGrabber::PcdGrabber() {
    ros::MultiThreadedSpinner spinner(4);
    __cloudPoints = PointCloud::Ptr(new PointCloud); 
    cloudPoints = PointCloud::Ptr(new PointCloud);
    service = nh_srv.advertiseService("getScenePcdSrvServer", &PcdGrabber::subPCDStart, this);
    std::cout << "Instantiated" << std::endl;
}

void PcdGrabber::sub() {
    ROS_INFO("Start subscribe pointCloud");
    spinner.spin();
}

bool PcdGrabber::subPCDStart(ros_pcl::srv_getScenePcd::Request &req, ros_pcl::srv_getScenePcd::Response &res) {
    std::cout << "SrvCalled" << std::endl;
    subscriber = nh_sub.subscribe<sensor_msgs::PointCloud2>("/kinect2/sd/points", 1, &PcdGrabber::callback, this);
    boost::this_thread::sleep(boost::posix_time::seconds(2));
    boost::thread subThread(&PcdGrabber::sub, this);

    catchCallbackPcdOnce();
    appendNewPointCloud();
    boost::this_thread::sleep(boost::posix_time::seconds(2));
    subscriber.shutdown();
    ROS_INFO("Stop subscribing PointCloud");
    
    //  pcl::PassThrough<pcl::PointXYZRGB> zFilter;

    //  std::vector<int> mapping;
    //  pcl::removeNaNFromPointCloud(*cloudPoints, *cloudPoints, mapping);
    //  zFilter.setInputCloud(cloudPoints);
    //  zFilter.setFilterFieldName("z");
    //  zFilter.setFilterLimits(0.5, 1.5);
    //  zFilter.filter(*cloudPoints);


    ROS_INFO("Collected %d points from Camera", static_cast<int>(cloudPoints->size()));
    std::string outputPath = req.dataDir + "/srcPCD/scene.pcd";
    savePCD(outputPath.c_str());
    ROS_INFO("scenePCD saved!");
    res.success = true;
    return true;
}

void PcdGrabber::callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
    // Container for original & filtered data
    pcl::fromROSMsg(*msg, *__cloudPoints);
    boost::this_thread::interruption_point();
}


void PcdGrabber::catchCallbackPcdOnce() {
    printf("catchCallbackPcdOnce called !!");
    pcl::copyPointCloud(*__cloudPoints, *cloudPoints);
}

void PcdGrabber::appendNewPointCloud() {
    *cloudPoints = (*__cloudPoints) +(*cloudPoints);
}

void PcdGrabber::savePCD(const char fileName[]){
    pcl::io::savePCDFileASCII(fileName ,*cloudPoints);
}

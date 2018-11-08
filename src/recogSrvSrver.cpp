#include <iostream>
#include <ros/ros.h>
#include <ros_pcl_msgs/srv_recogSrv.h>
#include <sensor_msgs/PointCloud2.h>
//#include <pub_msgs/pubMsgType.h>             // [TODO] rename: pub_msgs,
#include <geometry_msgs/TransformStamped.h>
#include <ros_pcl_msgs/srv_getScenePcd.h>
#include <ros_pcl_msgs/srv_preprocess.h>

#include <boost/bind/bind.hpp>

#include "../config/pointType.hpp"
#include "std_msgs/String.h"
#include <boost/bind/bind.hpp>
#include <boost/foreach.hpp>
#include <boost/thread/thread.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <iostream>
#include <pcl/common/io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <ros_pcl_msgs/srv_getScenePcd.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>

#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

// define a class, including a constructor, member variables and member
// functions
class RecogSrv {
  public:
    RecogSrv(ros::NodeHandle* nodehandle); //"main" will need to instantiate a ROS
                                           // nodehandle, then pass it to the
                                           // constructor

    // may choose to define public methods or public variables, if desired
  private:
    // put private member data here;  "private" data will only be available to
    // member functions of this class;
    ros::NodeHandle nh_; // we will need this, to pass between "main" and constructor
    ros::Rate __rate;
    // some objects to support subscriber, service, and publisher
    ros::Subscriber minimal_subscriber_; // these will be set up within the class
                                         // constructor, hiding these ugly details
    ros::ServiceServer __service;
    ros::Publisher __publisher;
    geometry_msgs::TransformStamped __transform;

    double val_from_subscriber_; // example member variable: better than using
                                 // globals; convenient way to pass data from a
                                 // subscriber to other member functions
    double val_to_remember_;     // member variables will retain their values even as
                                 // callbacks come and go

    // member methods as well:
    void initializePublishers();
    void initializeServices();

    // prototype for callback for example service
    bool serviceCallback(ros_pcl_msgs::srv_recogSrv::Request& request, ros_pcl_msgs::srv_recogSrv::Response& response);
    void __preparePubMsg();
    void pub();
}; // note: a class definition requires a semicolon at the end of the definition

RecogSrv::RecogSrv(ros::NodeHandle* nodehandle) : nh_(*nodehandle), __rate(5) { // constructor
    ROS_INFO("in class constructor of RecogSrv");
    initializePublishers();
    initializeServices();

    // initialize variables here, as needed

    // can also do tests/waits to make sure all required services, topics, etc
    // are
    // alive
}

// member helper function to set up services:
// similar syntax to subscriber, required for setting up services outside of
// "main()"
void RecogSrv::initializeServices() {
    ROS_INFO("Initializing Services");
    __service = nh_.advertiseService("exampleMinimalService", &RecogSrv::serviceCallback, this);
    // add more services here, as needed
}

// member helper function to set up publishers;
void RecogSrv::initializePublishers() {
    ROS_INFO("Initializing Publishers");
    __publisher = nh_.advertise<geometry_msgs::TransformStamped>("exampleMinimalPubTopic", 1, true); // [TODO]
    // add more publishers, as needed
    // note: COULD make __publisher a public member function, if want to use it
    // within "main()"
}

// member function implementation for a service callback function
bool RecogSrv::serviceCallback(ros_pcl_msgs::srv_recogSrv::Request& request, ros_pcl_msgs::srv_recogSrv::Response& response) {
    ROS_INFO("service callback activated");
    __preparePubMsg();
    response.success = true;

    return true;
}

void RecogSrv::pub() {
    for (int i = 0; i < 5; ++i) {
        __publisher.publish(__transform);
        ros::spinOnce();
        __rate.sleep();
    }
}

void RecogSrv::__preparePubMsg() {
    // here call the algorithm function
    int x;
}

int main(int argc, char** argv) {
    // ROS set-ups:
    ros::init(argc, argv, "RecogSrv"); // node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor

    ROS_INFO("main: instantiating an object of type RecogSrv");
    RecogSrv RecogSrv(&nh); // instantiate an RecogSrv object and pass in pointer
                            // to nodehandle for constructor to use

    ROS_INFO("main: going into spin; let the callbacks do all the work");
    ros::spin();
    return 0;
}
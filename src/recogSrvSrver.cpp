#include <iostream>
#include <ros/ros.h>
#include <ros_pcl_msgs/srv_recogSrv.h>
#include <sensor_msgs/PointCloud2.h>
#include <thesis_visualization_msgs/objectLocalization.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/TransformStamped.h>
#include <pcl_objRec/impl/matchingLocalPipeline.cpp>
#include <tf2_eigen/tf2_eigen.h>
#include <string>
#include <vector>


// [TODO] must be compiled without optimization with -DMAKE_BUILD_TYPE=Debug which is specified with option -O0 in CMakeList.txt
//        Otherwise, __pubMsg cannot find register?

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
    ros::Publisher __publisher_accepted;
    ros::Publisher __publisher_rejected;
    std::vector<RecogResult> __accepted, __rejected;
    thesis_visualization_msgs::objectLocalization __pubMsg_accepted;
    thesis_visualization_msgs::objectLocalization __pubMsg_rejected;

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
}

// member helper function to set up services:
// similar syntax to subscriber, required for setting up services outside of
// "main()"
void RecogSrv::initializeServices() {
    ROS_INFO("Initializing Services");
    __service = nh_.advertiseService("/RecognizeSrv", &RecogSrv::serviceCallback, this);
    // add more services here, as needed
}

// member helper function to set up publishers;
void RecogSrv::initializePublishers() {
    ROS_INFO("Initializing Publishers");
    __publisher_accepted = nh_.advertise<thesis_visualization_msgs::objectLocalization>("/acceptedModel", 1, true); // [TODO]
    __publisher_rejected = nh_.advertise<thesis_visualization_msgs::objectLocalization>("/rejectedModel", 1, true); // [TODO]
}

// member function implementation for a service callback function
bool RecogSrv::serviceCallback(ros_pcl_msgs::srv_recogSrv::Request& request, ros_pcl_msgs::srv_recogSrv::Response& response) {
    ROS_INFO("service callback activated");
    __preparePubMsg();
    pub();
    response.success = true;

    ROS_INFO("Recog Service finished");
    return true;
}

void RecogSrv::pub() {
    for (int i = 0; i < 5; ++i) {
        __publisher_accepted.publish(__pubMsg_accepted);
        __publisher_rejected.publish(__pubMsg_rejected);
        ros::spinOnce();
        __rate.sleep();
    }
    ROS_INFO("publish finished");
}

void RecogSrv::__preparePubMsg() {
    // here call the algorithm function
    std::string configFile = "/root/catkin_ws/src/ros_pcl/config/matchinLocalPipline_enableRes.yaml";    
    recognize(configFile, __accepted, __rejected);
    // tranform matrix4f to affine3f
    for (auto &instance : __accepted){
        Eigen::Transform<float, 3, Eigen::Affine> affine(instance.tf_mat);
        __pubMsg_accepted.modelList.push_back(instance.modelName);

        std_msgs::Header header;
        header.stamp = ros::Time::now();
        header.frame_id = "/kinect2_depth_optical_frame";
        __pubMsg_accepted.headers.push_back(header);

        Eigen::Affine3d affine_3d = affine.cast<double>();
        geometry_msgs::Pose transformPose = Eigen::toMsg(affine_3d);
        __pubMsg_accepted.pose.push_back(transformPose);
    }
    for (auto &instance : __rejected){
        Eigen::Transform<float, 3, Eigen::Affine> affine(instance.tf_mat);
        __pubMsg_rejected.modelList.push_back(instance.modelName);

        std_msgs::Header header;
        header.stamp = ros::Time::now();
        header.frame_id = "/kinect2_depth_optical_frame";
        __pubMsg_rejected.headers.push_back(header);

        //check translation from affine
        Eigen::Affine3d affine_3d = affine.cast<double>();
        geometry_msgs::Pose transformPose = Eigen::toMsg(affine_3d);

        __pubMsg_rejected.pose.push_back(transformPose);
    }
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
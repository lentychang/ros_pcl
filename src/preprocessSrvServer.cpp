#include <ros/ros.h>
#include <pcl_objRec/preprocessor.h>
#include <pcl_objRec/scene_prepare.h>
#include <pcl_objRec/model_prepare.h>
#include <ros_pcl_msgs/srv_preprocess.h>

bool preprocessSrvCb(ros_pcl_msgs::srv_preprocess::Request &req, ros_pcl_msgs::srv_preprocess::Response &res) {
    res.success=true;
    if (req.filename == "scene") {
        scenenPreprocess(req.dataDir);

    }
    else if (req.filename=="lf064-01" || req.filename=="lf064-02" || 
             req.filename=="lf064-03" || req.filename=="lf064-04" || 
             req.filename=="lf064-05" || req.filename=="lf064-06" ) {
        modelPreprocess(req.dataDir, req.filename);

    }
    else  {
        ROS_ERROR("preprocess service... filename doesn't not exist");
        res.success=false;}
    
    return true;
}


int main(int argc, char** argv) {

    ros::init(argc, argv, "preprocessSrvServer");

    ros::NodeHandle nh_srv;
    ros::ServiceServer service;
    service = nh_srv.advertiseService("preprocessSrvServer", preprocessSrvCb);
    ROS_INFO("Preprocess Service started");
    ROS_INFO("command [dataDir] [filename]");
    ros::spin();

}
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "my_tf2_listener");

    ros::NodeHandle node;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Publisher pub = node.advertise<geometry_msgs::TransformStamped>(argv[1], 1);

    ros::Rate rate(5.0);
    geometry_msgs::TransformStamped transformStamped;
    while (node.ok()) {
        ROS_DEBUG("%s", tfBuffer.allFramesAsString().c_str());
        try {
            transformStamped = tfBuffer.lookupTransform(argv[2], argv[3], ros::Time(0), ros::Duration(3.0));
        }
        catch (tf2::TransformException& ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        pub.publish(transformStamped);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
};
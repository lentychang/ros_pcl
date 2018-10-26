#include <ros/ros.h>
#include <visualization/Marker.h>


visualization_msgs::Marker marker;
marker.header.frame_id = "world";
marker.header.stamp = ros::Time();
marker.ns = "/recognized_";
marker.id = 0;
marker.type = visualization_msgs::Marker::MESH_RESOURCE;
marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
marker.action = visualization_msgs::Marker::ADD;
marker.pose.position.x = 1;
marker.pose.position.y = 1;
marker.pose.position.z = 1;
marker.pose.orientation.x = 0.0;
marker.pose.orientation.y = 0.0;
marker.pose.orientation.z = 0.0;
marker.pose.orientation.w = 1.0;
marker.scale.x = 0.001;
marker.scale.y = 0.001;
marker.scale.z = 0.001;
marker.color.a = 0.7; // Don't forget to set the alpha!
marker.color.r = 0.0;
marker.color.g = 1.0;
marker.color.b = 0.0;
//only if using a MESH_RESOURCE marker type:
marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
vis_pub.publish( marker );
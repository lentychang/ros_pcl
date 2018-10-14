#ifndef CAMERA_H
#define CAMERA_H

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/io.h>
#include <pcl/io/openni2_grabber.h>

#include <iostream>
#include <string>
#include <cstdio>
#include <ctime>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>
#include "../../config/pointType.hpp"


class Camera{
private:
    pcl::Grabber* grabber;
public:
    Camera();
    pcl::PointCloud<pntType>::Ptr __cloudPoints;
    static void cb_kinect(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud);
    boost::function<void(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> function;
    void start();
    void stop();
    void updatePoints(const pcl::PointCloud<pntType>::Ptr& givenPointCloud);


};





#endif
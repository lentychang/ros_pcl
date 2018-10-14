#include "../Camera.h"


Camera::Camera() {
	// callback function that copy the input data to point cloud
    __cloudPoints = pcl::PointCloud<pntType>::Ptr(new pcl::PointCloud<pntType>);
    
    function = boost::bind(&(this->cb_kinect), this);
    // Create Kinect2Grabber
    grabber = new pcl::io::OpenNI2Grabber();
    // Regist Callback Function
    grabber->registerCallback(function);

}

void Camera::cb_kinect(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud) {
    pcl::copyPointCloud(*cloud, *__cloudPoints);
};

// Start Retrieve Data
void Camera::start() {
    grabber->start();
    boost::this_thread::sleep(boost::posix_time::seconds(2));
}

void Camera::stop() {
    grabber->stop();
}

void Camera::updatePoints(const pcl::PointCloud<pntType>::Ptr& givenPointCloud) {
    pcl::copyPointCloud(*__cloudPoints, *givenPointCloud);
}
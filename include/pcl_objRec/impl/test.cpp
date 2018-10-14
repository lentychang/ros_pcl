#include "preprocess.hpp"
using namespace std;

int main() {
	Camera camera;
	camera.start();

	pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZ>);

	camera.updatePoints(pointCloud);

	for (int i =0; i <20; ++i)
	{
	cout << pointCloud->points[i] << endl;
	}
	boost::this_thread::sleep(boost::posix_time::seconds(2));
	camera.updatePoints(pointCloud);
	for (int i =0; i <20; ++i)
	{
	cout << pointCloud->points[i] << endl;
	}


	camera.stop();

	return 0;
} 


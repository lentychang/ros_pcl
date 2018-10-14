#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <pcl/search/kdtree.h>

int main(int argc, char const *argv[])
{
	std::cout << "command hint: [Input pcd file] [scale]" << std::endl;
	// Object for storing the point cloud.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPoints(new pcl::PointCloud<pcl::PointXYZ>);
	// Read a PCD file from disk.
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloudPoints) != 0)
	{
		return -1;
	}

	for (int i=0; i < cloudPoints->size(); ++i) {
		cloudPoints->at(i).x *= atof(argv[2]);
		cloudPoints->at(i).y *= atof(argv[2]);
		cloudPoints->at(i).z *= atof(argv[2]);
	}
	std::string inputFileName = argv[1];

	size_t lastindex = inputFileName.find_last_of(".");
    std::string rawname = inputFileName.substr(0, lastindex);
	std::string newFileName = rawname + "_rescale" + argv[2] +".pcd";
	pcl::io::savePCDFileASCII(newFileName ,*cloudPoints);
	return 0;
}

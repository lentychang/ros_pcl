#include <pcl/io/io.h>
#include <iostream>
#include <pcl/search/kdtree.h>


double
computeCloudResolution(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
{
	double resolution = 0.0;
	int numberOfPoints = 0;
	int nres;
	std::vector<int> indices(3);
	std::vector<float> squaredDistances(3);
	pcl::search::KdTree<pcl::PointXYZ> tree;
	tree.setInputCloud(cloud);

	for (size_t i = 0; i < cloud->size(); ++i)
	{
		if (! pcl_isfinite((*cloud)[i].x))
			continue;

		// Considering the second neighbor since the first is the point itself.
		nres = tree.nearestKSearch(i, 2, indices, squaredDistances);
		if (nres == 2)
		{
			resolution += sqrt(squaredDistances[1]);
			++numberOfPoints;
		}
	}
	if (numberOfPoints != 0) {
		resolution /= numberOfPoints;
		std::cout << "Cloud resolution is:" << resolution << std::endl;
	}
	
	else {
		std::cout << "[Error] The resolution is 0, check if there're duplicated points" << std::endl;		
	}
		
	return resolution;
}
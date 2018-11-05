#include <pcl_objRec/utils.h>


double
computeCloudResolution(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
{
	double resolution = 0.0;
	int numberOfPoints = 0;
	int nres;
	std::vector<int> indices(11);
	std::vector<float> squaredDistances(11);
	pcl::search::KdTree<pcl::PointXYZ> tree;
	tree.setInputCloud(cloud);
	int j = 1;
	bool flag = true;
	for (size_t i = 0; i < cloud->size(); ++i)
	{
		if (! pcl_isfinite((*cloud)[i].x))
			continue;

		// Considering the second neighbor since the first is the point itself.
		j = 1;
		nres = tree.nearestKSearch(i, j + 1, indices, squaredDistances);
		while (squaredDistances[j] < 0.00000001 || j > 10)
		{
			j += 1;
			nres = tree.nearestKSearch(i, j + 1, indices, squaredDistances);
			if (j ==10) flag = false; 
		}
		if (flag){
			resolution += sqrt(squaredDistances[j]);
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

void
mls_upsampling(pcl::PointCloud<pcl::PointXYZ>::Ptr& inCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &outCloud, float resolution ,float stepRatio){
	// Filtering object.
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> filter;
	filter.setInputCloud(inCloud);
	// Object for searching.
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree;
	filter.setSearchMethod(kdtree);
	// Use all neighbors in a radius of 3cm.
	filter.setSearchRadius(resolution * 2.5);
	// Upsampling method. Other possibilites are DISTINCT_CLOUD, RANDOM_UNIFORM_DENSITY
	// and VOXEL_GRID_DILATION. NONE disables upsampling. Check the API for details.
	filter.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::SAMPLE_LOCAL_PLANE);
	// Radius around each point, where the local plane will be sampled.
	filter.setUpsamplingRadius(resolution * 2.5);
	// Sampling step size. Bigger values will yield less (if any) new points.
	filter.setUpsamplingStepSize(resolution * stepRatio);
	filter.process(*outCloud);
}

void
euclideanSeg (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int minClusterSize, int maxClusterSize, float cluster_tol, bool enableViewer)
{
    // Create EuclideanClusterExtraction and set parameters
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (cluster_tol);
    ec.setMinClusterSize (minClusterSize);
    ec.setMaxClusterSize (maxClusterSize);
    // set input cloud and let it run
    std::vector <pcl::PointIndices> clusters;
    ec.setInputCloud (cloud);
    ec.extract (clusters);

    if (static_cast<int>(clusters.size()==0)){
        ROS_WARN("No cluster found!!");
        saveFile_ = false;
    }

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pntsInCluster (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PointIndices::Ptr pntIdxs (new pcl::PointIndices ());    
    std::string filename;

	pcl::visualization::PCLVisualizer viewer ("Cluster viewer");

    if (enableViewer){
        for (int i=0;i<static_cast<int>(clusters.size());++i){

            extract.setInputCloud (cloud);
            *pntIdxs = clusters[i];
            extract.setIndices(pntIdxs);
            extract.setNegative (false);
            extract.filter(*pntsInCluster);

			viewer.addPointCloud (pntsInCluster, "clusterCloud");
			while(!viewer.wasStopped()) {
				viewer.spinOnce ();
			}
			viewer.removeAllPointClouds();
			viewer.resetStoppedFlag();
        }
    }
}

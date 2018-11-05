#include <pcl_objRec/utils.h>

typedef pcl::PointXYZ PointType;
typedef pcl::Normal NormalType;

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

std::vector <pcl::PointIndices>
euclideanSeg (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud, int minClusterSize, int maxClusterSize, float cluster_tol, bool enableViewer)
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
	int clusterSize = static_cast<int>(clusters.size());

    if (clusterSize>0){
        ROS_INFO("Segmentation succeed! %d clusters found !", clusterSize);
    }
	else {
		ROS_WARN("No cluster found!!");
		enableViewer=false;		
	}
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr pntsInCluster (new pcl::PointCloud<pcl::PointXYZ> ());
	pcl::visualization::PCLVisualizer viewer ("Cluster viewer");
    if (enableViewer){
        for (int i=0;i<clusterSize;++i){
			extractPCDfromCluster(clusters, i+1, cloud, pntsInCluster);
			viewer.addPointCloud (pntsInCluster, "clusterCloud");
			while(!viewer.wasStopped()) {
				viewer.spinOnce ();
			}
			viewer.removeAllPointClouds();
			viewer.resetStoppedFlag();
        }
    }

	return clusters;
}

void extractPCDfromCluster(std::vector <pcl::PointIndices>& clusters, int nth_cluster, const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& outCloud) {
	if ( nth_cluster <= static_cast<int>(clusters.size())){
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		// pcl::PointCloud<pcl::PointXYZ>::Ptr pntsInCluster (new pcl::PointCloud<pcl::PointXYZ> ());
		pcl::PointIndices::Ptr pntIdxs (new pcl::PointIndices ());
		extract.setInputCloud (cloud);
		*pntIdxs = clusters[nth_cluster-1];
		extract.setIndices(pntIdxs);
		extract.setNegative (false);
		extract.filter(*outCloud);
	}
	else {
		ROS_WARN("Due to no cluster found, output is directly copied from input. Please check the parameters");
		pcl::copyPointCloud(*cloud, *outCloud);
	}
}


void cropPcd(const Eigen::Vector4f& min, const Eigen::Vector4f& max, const Eigen::Vector3f &translation, const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& outCloud, bool setNegative){
	pcl::CropBox<pcl::PointXYZ> boxFilter;
	
	boxFilter.setMin(min);
	boxFilter.setMax(max);
	boxFilter.setTranslation(translation);
	boxFilter.setInputCloud(cloud);
	boxFilter.setNegative(setNegative);
	boxFilter.filter(*outCloud);
}


void normalViewer(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pnts, const pcl::PointCloud<pcl::Normal>::ConstPtr& normals) {
	pcl::visualization::PCLVisualizer viewer ("Normal_viewer");
	
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(pnts, 255, 255, 255);
  	viewer.addPointCloud<pcl::PointXYZ> (pnts, single_color, "sample cloud");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(pnts, normals, 3, 0.005, "normals");
	// printf("size of pointcloud: %d\n", pntNormals->size());
	viewer.addCoordinateSystem(0.08);
	viewer.initCameraParameters();
	while(!viewer.wasStopped()) {
		viewer.spinOnce (100);
	}
}


void __test_computeResolution(int argc, char* argv[]) {
	std::cout << "Usage: command [filename contains path]" << std::endl;
	std::string filename = "/root/exchange/tempData/filtered/scene2_points_.pcd";
    if (argc>=2) {
		filename = argv[1];
	}
    pcl::PointCloud<PointType>::Ptr pnts (new pcl::PointCloud<PointType> ());

    if (pcl::io::loadPCDFile<PointType> (filename, *pnts) < 0) {
        ROS_FATAL("Error loading model cloud.");
    }
      
    //remove NaN
	std::vector<int> indices_src;
	pcl::removeNaNFromPointCloud(*pnts, *pnts, indices_src);
	computeCloudResolution(pnts);
}



void __test_euclideanSeg(int argc, char *argv[]){
	std::string filename="/root/exchange/tempData/filtered/scene2_points_.pcd";
	pcl::console::parse_argument(argc, argv, "--filepath", filename);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
	pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud);
	euclideanSeg(cloud, 40, 1500, 0.01, true);
}

void __test_cropPcd(int argc, char *argv[]){
	std::string filename="/root/exchange/tempData/srcPCD/scene.pcd";
	pcl::console::parse_argument(argc, argv, "--filepath", filename);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
	pcl::PointCloud<pcl::PointXYZ>::Ptr outCloud (new pcl::PointCloud<pcl::PointXYZ> ());
	pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud);

	Eigen::Vector4f min = Eigen::Vector4f(0,0,0,1);
	Eigen::Vector4f max = Eigen::Vector4f(1,1,1,1);
	Eigen::Vector3f translation = Eigen::Vector3f(-0.5, -0.5, 0.5);
	cropPcd(min, max, translation, cloud, outCloud, true);

	pcl::visualization::PCLVisualizer viewer ("Cluster viewer");
	viewer.addPointCloud (cloud, "inputCloud");
	while(!viewer.wasStopped()) {
		viewer.spinOnce ();
	}
	viewer.removeAllPointClouds();
	viewer.resetStoppedFlag();
	viewer.addPointCloud (outCloud, "outCloud");
	while(!viewer.wasStopped()) {
		viewer.spinOnce ();
	}
}

void __test_normalViewer(){
	std::string filename = "/root/exchange/tempData/filtered/scene2_points_.pcd";
    pcl::PointCloud<PointType>::Ptr pnts (new pcl::PointCloud<PointType> ());
    if (pcl::io::loadPCDFile<PointType> (filename, *pnts) < 0) {
        ROS_FATAL("Error loading model cloud.");
    }
	//pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal> ());
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal> ());
	pcl::NormalEstimationOMP<PointType, pcl::Normal> norm_est;
	float res = static_cast<float>(computeCloudResolution(pnts));
	float searchR = 2.0;

	norm_est.setRadiusSearch(res * searchR);
	norm_est.setInputCloud (pnts);
	norm_est.compute(*normals);
	printf("size of pnts: %d\n", static_cast<int>(pnts->size()));
	printf("size of pntNormals: %d\n", static_cast<int>(normals->size()));
	normalViewer(pnts ,normals);

}


void __tool_computeResolution() {
	std::string filepath;
	pcl::PointCloud<PointType>::Ptr pnts (new pcl::PointCloud<PointType> ());
	while (true){
		std::cout << "Please enter filepath, or 'exit', 'q, 'quit' to exit" << std::endl;
		std::cin >> filepath;
		if (filepath == "exit" || filepath == "quit" || filepath == "q") {
			break;
		}
		
		if (pcl::io::loadPCDFile<PointType> (filepath, *pnts) < 0) {
			ROS_FATAL("Error loading model cloud.");
		}
		//remove NaN
		std::vector<int> indices_src;
		pcl::removeNaNFromPointCloud(*pnts, *pnts, indices_src);
		computeCloudResolution(pnts);
	}
}

void __tool_normalViewer(){
	std::string filename = "/root/exchange/tempData/filtered/scene2_points_.pcd";
	std::string input;
	float searchR = 2.0;

	pcl::PointCloud<PointType>::Ptr pnts (new pcl::PointCloud<PointType> ());
	pcl::visualization::PCLVisualizer viewer ("Normal_viewer");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(pnts, 255, 255, 255);
	
	while (input != "exit" || input != "q" || input != "quit"){		
		std::cout << "Please enter filepath or 'c' to continue or 'exit, quit, q' to exit: ";
		std::cin >> input;
		if (input != "c"){
			filename = input;
		}

		if (input == "exit" || input == "q" || input == "quit"){
			break;
		}

		std::cout << "Please enter searchR: ";
		std::cin >> searchR;

		if (pcl::io::loadPCDFile<PointType> (filename, *pnts) < 0) {
			ROS_FATAL("Error loading model cloud.");
		}
		pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal> ());
		pcl::NormalEstimationOMP<PointType, pcl::Normal> norm_est;
		float res = static_cast<float>(computeCloudResolution(pnts));
				
		norm_est.setRadiusSearch(res * searchR);
		norm_est.setInputCloud (pnts);
		norm_est.compute(*normals);
		printf("size of pnts: %d\n", static_cast<int>(pnts->size()));
		printf("size of pntNormals: %d\n", static_cast<int>(normals->size()));

		viewer.addPointCloud<pcl::PointXYZ> (pnts, single_color, "sample cloud");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
		viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(pnts, normals, 3, 0.005, "normals");
		// printf("size of pointcloud: %d\n", pntNormals->size());
		viewer.addCoordinateSystem(0.08);
		viewer.initCameraParameters();

		while(!viewer.wasStopped()) {
			viewer.spinOnce ();
		}
		viewer.removeAllPointClouds();
		viewer.resetStoppedFlag();
		// normalViewer(pnts ,normals);
	}
	viewer.close();

}


void __tools() {
	int i = 1;
	char cwd[PATH_MAX];
   	if (getcwd(cwd, sizeof(cwd)) != NULL) {
       printf("Current working dir: %s\n", cwd);
   	} else {
       perror("getcwd() error");
   	}
	printf("Current working dir: %s\n", cwd);
	while (i !=9){
		std::cout << "Please enter function number you want to use:" << std::endl;
		std::cout << "[1] compute cloud resolution (default)" << std::endl;
		std::cout << "[2] Normal viewer" << std::endl;
		std::cout << "[9] Exit" << std::endl;
		std::cin >> i;
		
		if (i == 1){
			__tool_computeResolution();
		}
		else if (i == 2){
			__tool_normalViewer();
		}
		else if (i ==9) {break;}
	}
}


int main(int argc, char *argv[]){
	// __test_euclideanSeg(argc, argv);
	// __test_cropPcd(argc, argv);
	// __test_computeResolution(argc, argv);
	__tools();
	// __test_normalViewer();

}
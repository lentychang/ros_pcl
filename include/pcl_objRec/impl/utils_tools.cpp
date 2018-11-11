#include "utils.cpp"

void __test_computeResolution(int argc, char* argv[]) {
	std::cout << "Usage: command [filename contains path]" << std::endl;
	std::string filename = "/root/exchange/tempData/filtered/scene2_points_.pcd";
    if (argc>=2) {
		filename = argv[1];
	}
    pcl::PointCloud<PointType>::Ptr pnts (new pcl::PointCloud<PointType> ());

    if (pcl::io::loadPCDFile<PointType> (filename, *pnts) < 0) {
        std::cerr << "[FATAL] Error loading model cloud" << std::endl;
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
        std::cerr  << "[FATAL] Error loading model cloud." << std::endl;
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
			std::cerr << "Error loading model cloud." << std::endl;
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
			std::cerr << "Error loading model cloud." << std::endl;
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
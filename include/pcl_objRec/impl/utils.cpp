#include <pcl_objRec/utils.h>

typedef pcl::PointXYZ PointType;
typedef pcl::Normal NormalType;

double computeCloudResolution(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud) {
    double resolution = 0.0;
    int numberOfPoints = 0;
    int nres;
    std::vector<int> indices(11);
    std::vector<float> squaredDistances(11);
    pcl::search::KdTree<pcl::PointXYZ> tree;
    tree.setInputCloud(cloud);
    int j = 1;
    bool flag = true;
    for (size_t i = 0; i < cloud->size(); ++i) {
        if (!pcl_isfinite((*cloud)[i].x)) continue;

        // Considering the second neighbor since the first is the point itself.
        j = 1;
        nres = tree.nearestKSearch(i, j + 1, indices, squaredDistances);
        while (squaredDistances[j] < 0.00000001 || j > 10) {
            j += 1;
            nres = tree.nearestKSearch(i, j + 1, indices, squaredDistances);
            if (j == 10) flag = false;
        }
        if (flag) {
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

void mls_upsampling(pcl::PointCloud<pcl::PointXYZ>::Ptr& inCloud,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr& outCloud,
                    float resolution,
                    float stepRatio) {
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

std::vector<pcl::PointIndices> euclideanSeg(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
                                            int minClusterSize,
                                            int maxClusterSize,
                                            float cluster_tol,
                                            bool enableViewer) {
    // Create EuclideanClusterExtraction and set parameters
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_tol);
    ec.setMinClusterSize(minClusterSize);
    ec.setMaxClusterSize(maxClusterSize);
    // set input cloud and let it run
    std::vector<pcl::PointIndices> clusters;
    ec.setInputCloud(cloud);
    ec.extract(clusters);
    int clusterSize = static_cast<int>(clusters.size());

    if (clusterSize > 0) {
        std::cout << "[INFO] Segmentation succeed! " << clusterSize << " clusters found !" << std::endl;
    }
    else {
        std::cout << "[WARN] No cluster found!!" << std::endl;
        enableViewer = false;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr pntsInCluster(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::visualization::PCLVisualizer viewer("Cluster viewer");
    if (enableViewer) {
        for (int i = 0; i < clusterSize; ++i) {
            extractPCDfromCluster(clusters, i + 1, cloud, pntsInCluster);
            viewer.addPointCloud(pntsInCluster, "clusterCloud");
            while (!viewer.wasStopped()) { viewer.spinOnce(); }
            viewer.removeAllPointClouds();
            viewer.resetStoppedFlag();
        }
    }
    return clusters;
}

void extractPCDfromCluster(std::vector<pcl::PointIndices>& clusters,
                           int nth_cluster,
                           const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr& outCloud) {
    if (nth_cluster <= static_cast<int>(clusters.size())) {
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        // pcl::PointCloud<pcl::PointXYZ>::Ptr pntsInCluster (new pcl::PointCloud<pcl::PointXYZ>
        // ());
        pcl::PointIndices::Ptr pntIdxs(new pcl::PointIndices());
        extract.setInputCloud(cloud);
        *pntIdxs = clusters[nth_cluster - 1];
        extract.setIndices(pntIdxs);
        extract.setNegative(false);
        extract.filter(*outCloud);
    }
    else {
        std::cout << "Due to no cluster found, output is directly copied from input. Please check "
                     "the parameters"
                  << std::endl;
        pcl::copyPointCloud(*cloud, *outCloud);
    }
}

void cropPcd(const Eigen::Vector4f& min,
             const Eigen::Vector4f& max,
             const Eigen::Vector3f& translation,
             const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
             pcl::PointCloud<pcl::PointXYZ>::Ptr& outCloud,
             bool setNegative) {
    pcl::CropBox<pcl::PointXYZ> boxFilter;

    boxFilter.setMin(min);
    boxFilter.setMax(max);
    boxFilter.setTranslation(translation);
    boxFilter.setInputCloud(cloud);
    boxFilter.setNegative(setNegative);
    boxFilter.filter(*outCloud);
}

void normalViewer(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pnts,
                  const pcl::PointCloud<pcl::Normal>::ConstPtr& normals) {
    pcl::visualization::PCLVisualizer viewer("Normal_viewer");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(pnts, 255, 255, 255);
    viewer.addPointCloud<pcl::PointXYZ>(pnts, single_color, "sample cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                            3,
                                            "sample cloud");
    viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(pnts, normals, 3, 0.005, "normals");
    // printf("size of pointcloud: %d\n", pntNormals->size());
    viewer.addCoordinateSystem(0.08);
    viewer.initCameraParameters();
    while (!viewer.wasStopped()) { viewer.spinOnce(100); }
}
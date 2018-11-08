/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/board.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/recognition/hv/hv_go.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl_objRec/impl/utils.cpp>
#include <ros/ros.h>
#include <ros/console.h>

typedef pcl::PointXYZ PointType;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT352 DescriptorType;
typedef pcl::FPFHSignature33 FPFHDescriptor;

struct CloudStyle {
    double r;
    double g;
    double b;
    double size;

    CloudStyle(double r, double g, double b, double size) : r(r), g(g), b(b), size(size) {}
};

CloudStyle style_white(255.0, 255.0, 255.0, 4.0);
CloudStyle style_red(255.0, 0.0, 0.0, 3.0);
CloudStyle style_green(0.0, 255.0, 0.0, 5.0);
CloudStyle style_cyan(93.0, 200.0, 217.0, 4.0);
CloudStyle style_violet(255.0, 0.0, 255.0, 8.0);

std::string modelDir("/root/exchange/tempData");
std::string modelName_("lf064-05");
std::string sceneName_("/filtered/scene2_points_.pcd");
bool disableModelList_(false);

std::string detector_ = "uni";
std::string descriptor_ = "shot";

// Algorithm params
bool show_keypoints_(false);
bool show_correspondences_(false);
bool use_cloud_resolution_(false);
bool use_hough_(true);
float corr_dist_(0.4);
float normal_ss_(0.015f);
float model_ss_(0.01f);
float scene_ss_(0.03f);
float rf_rad_(0.015f);
float descr_rad_(0.02f);
float cg_size_(0.01f);
float cg_thresh_(5.0f);

bool do_icp_(false);
int icp_max_iter_(5);
float icp_corr_distance_(0.005f);
float hv_resolution_(0.005f);
float hv_occupancy_grid_resolution_(0.01f);
float hv_clutter_reg_(5.0f);
float hv_inlier_th_(0.005f);
float hv_occlusion_th_(0.01f);
float hv_rad_clutter_(0.03f);
float hv_regularizer_(3.0f);
float hv_rad_normals_(0.05);
bool hv_detect_clutter_(true);

bool enableViewer_(false);

void showHelp(char* filename);

void parseCommandLine(int argc, char* argv[]);

void setupResolution(float resolution);

void read_pcd(const std::string& baseName, pcl::PointCloud<PointType>::Ptr& pnts);

// after confirmation the correctness of final_tfMatrixList, registered_instances can be removed and generate it outside
// the function
void matchShot(const std::string& modelName,
               pcl::PointCloud<PointType>::Ptr& scene,
               pcl::PointCloud<NormalType>::Ptr& scene_normals,
               pcl::PointCloud<PointType>::Ptr& scene_keypoints,
               pcl::PointCloud<DescriptorType>::Ptr& scene_descriptors,
               std::vector<pcl::PointCloud<PointType>::ConstPtr>& registered_instances,
               std::vector<Eigen::Matrix4f>& final_tfMatrixList,
               bool calculateScene = true);

void matchFpfh(const std::string& modelName,
               pcl::PointCloud<PointType>::Ptr& scene,
               pcl::PointCloud<NormalType>::Ptr& scene_normals,
               pcl::PointCloud<PointType>::Ptr& scene_keypoints,
               pcl::PointCloud<FPFHDescriptor>::Ptr& scene_descriptors,
               std::vector<pcl::PointCloud<PointType>::ConstPtr>& registered_instances,
               bool calculateScene = true);

void viewSingleMatch(pcl::PointCloud<PointType>::Ptr& scene,
                     pcl::PointCloud<PointType>::Ptr& model,
                     pcl::PointCloud<PointType>::Ptr& scene_keypoints,
                     pcl::PointCloud<PointType>::Ptr& model_keypoints,
                     std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>& rototranslations,
                     std::vector<pcl::Correspondences>& clustered_corrs,
                     std::vector<pcl::PointCloud<PointType>::ConstPtr>& registered_instances);

void matchAllModels(bool matchAllmodels,
                    std::vector<std::string>& modelList,
                    pcl::PointCloud<PointType>::Ptr& scene,
                    pcl::PointCloud<pcl::Normal>::Ptr& scene_normals,
                    pcl::PointCloud<PointType>::Ptr& scene_keypoints,
                    pcl::PointCloud<DescriptorType>::Ptr& scene_descriptors,
                    pcl::PointCloud<FPFHDescriptor>::Ptr& scene_fpfhDescriptors,
                    std::vector<int>& no_model_instances,
                    std::vector<Eigen::Matrix4f>& all_final_tfMatrixList,
                    std::vector<pcl::PointCloud<PointType>::ConstPtr>& all_registered_instances);

void hypothesis_verification(const pcl::PointCloud<PointType>::Ptr& scene,
                             std::vector<pcl::PointCloud<PointType>::ConstPtr>& registered_instances,
                             std::vector<bool>& hypotheses_mask);
void __test_final_tranformation(const pcl::PointCloud<PointType>::Ptr& scene,
                                std::vector<pcl::PointCloud<PointType>::Ptr>& modelpcdList,
                                std::vector<int>& no_model_instances,
                                std::vector<Eigen::Matrix4f>& all_final_tfMatrixList);

void view_hv_result(bool enableViewer,
                    const pcl::PointCloud<PointType>::Ptr& scene,
                    std::vector<bool>& hypothesesMask,
                    std::vector<pcl::PointCloud<PointType>::ConstPtr>& all_registered_instances);

// rosrun ros_pcl matchingLocalPipeline --scene /filtered/scene2_points_.pcd --corr_dist 0.4 --model lf064-05 --do_icp
// -v -c -r --algorithm GC --normal_ss 2 --model_ss 1.2 --scene_ss 1.2 --rf_rad 2.4 --descr_rad 2.4 --cg_size 3
// --cg_thresh 8 > log.txt
int main(int argc, char* argv[]) {
    parseCommandLine(argc, argv);
    // Recognition modelList
    std::vector<std::string> modelList{"lf064-01", "lf064-02", "lf064-03", "lf064-04", "lf064-05"};
    if (disableModelList_) {
        modelList.clear();
        modelList.push_back(modelName_);
    }

    pcl::PointCloud<PointType>::Ptr scene(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr scene_keypoints(new pcl::PointCloud<PointType>());
    pcl::PointCloud<NormalType>::Ptr scene_normals(new pcl::PointCloud<NormalType>());
    pcl::PointCloud<DescriptorType>::Ptr scene_descriptors(new pcl::PointCloud<DescriptorType>());
    pcl::PointCloud<FPFHDescriptor>::Ptr scene_fpfhDescriptors(new pcl::PointCloud<FPFHDescriptor>());

    read_pcd(sceneName_, scene);
    setupResolution(static_cast<float>(computeCloudResolution(scene)));

    // initialize variable for storing data
    std::vector<pcl::PointCloud<PointType>::Ptr> modelpcdList; // vector of model pcd
    std::vector<int> no_model_instances; // Matrix Vector to store all transformation matrix after icp
    std::vector<Eigen::Matrix4f> all_final_tfMatrixList;
    std::vector<pcl::PointCloud<PointType>::ConstPtr> all_registered_instances; // Vector of transformed pointclouds

    // ##############################
    // ### Start Recognition, ICP ###
    // ##############################
    matchAllModels(
        true, modelList, scene, scene_normals, scene_keypoints, scene_descriptors, scene_fpfhDescriptors, no_model_instances, all_final_tfMatrixList, all_registered_instances);

    //__test_final_tranformation(scene, modelpcdList, no_model_instances, all_final_tfMatrixList);

    // model trasformation matrix list
    std::vector<Eigen::Matrix4f> tfMatrix_hv_true, tfMatrix_hv_false;

    // #####################################
    // ### Start hypothesis verification ###
    // #####################################
    if (static_cast<int>(all_registered_instances.size()) > 0) {
        std::vector<bool> hypothesesMask;
        hypothesis_verification(scene, all_registered_instances, hypothesesMask);
        // visualization
        view_hv_result(enableViewer_, scene, hypothesesMask, all_registered_instances);
    }
    else {
        std::cout << "No instance found !!" << std::endl;
    }

    return (0);
}

void showHelp(char* filename) {
    std::cout << std::endl;
    std::cout << "***************************************************************************" << std::endl;
    std::cout << "*                                                                         *" << std::endl;
    std::cout << "*             Correspondence Grouping Tutorial - Usage Guide              *" << std::endl;
    std::cout << "*                                                                         *" << std::endl;
    std::cout << "***************************************************************************" << std::endl
              << std::endl;
    std::cout << "Usage: " << filename << " model_filename.pcd scene_filename.pcd [Options]" << std::endl << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "     -h:                         Show this help." << std::endl;
    std::cout << "     -k:                         Show used keypoints." << std::endl;
    std::cout << "     -c:                         Show used correspondences." << std::endl;
    std::cout << "     -r:                         Compute the model cloud resolution and multiply" << std::endl;
    std::cout << "     -v:                         Enable Viewer" << std::endl;
    std::cout << "                                 each radius given by that value." << std::endl;
    std::cout << "     --algorithm (Hough|GC):     Clustering algorithm used (default Hough)." << std::endl;
    std::cout << "     --corr_dist val:            Correspondent distance (default 0.4)" << std::endl;
    std::cout << "     --normal_ss val:            Normal search radius (default 0.015)" << std::endl;
    std::cout << "     --model_ss val:             Model uniform sampling radius (default 0.01)" << std::endl;
    std::cout << "     --scene_ss val:             Scene uniform sampling radius (default 0.03)" << std::endl;
    std::cout << "     --rf_rad val:               Reference frame radius (default 0.015)" << std::endl;
    std::cout << "     --descr_rad val:            Descriptor radius (default 0.02)" << std::endl;
    std::cout << "     --cg_size val:              Cluster size (default 0.01)" << std::endl;
    std::cout << "     --cg_thresh val:            Clustering threshold (default 5)" << std::endl;
    std::cout << "     --model val:                Enalbe single model matching and specified model name (lf064-05)" << std::endl;
    std::cout << "     --scene val:                Specified scene name (/filtered/scene2_points_.pcd)" << std::endl;
    std::cout << "     --do_icp val:               Enable icp registration" << std::endl;
    std::cout << "     --detector (iss|sift|shot): Keypoint detector (default uni)" << std::endl << std::endl;
    std::cout << "     --descriptor (shot):            If changed, need to recompile (default shot)" << std::endl
              << std::endl;
}

void parseCommandLine(int argc, char* argv[]) {
    // Show help
    if (pcl::console::find_switch(argc, argv, "-h")) {
        showHelp(argv[0]);
        exit(0);
    }

    // Program behavior
    if (pcl::console::find_switch(argc, argv, "-k")) show_keypoints_ = true;
    if (pcl::console::find_switch(argc, argv, "-c")) show_correspondences_ = true;
    if (pcl::console::find_switch(argc, argv, "-r")) use_cloud_resolution_ = true;
    if (pcl::console::find_switch(argc, argv, "--do_icp")) do_icp_ = true;
    if (pcl::console::find_switch(argc, argv, "-v")) enableViewer_ = true;

    if (pcl::console::find_switch(argc, argv, "--model")) {
        disableModelList_ = true;
        pcl::console::parse_argument(argc, argv, "--model", modelName_);
    }
    std::string used_algorithm;
    if (pcl::console::parse_argument(argc, argv, "--algorithm", used_algorithm) != -1) {
        if (used_algorithm.compare("Hough") == 0) { use_hough_ = true; }
        else if (used_algorithm.compare("GC") == 0) {
            use_hough_ = false;
        }
        else {
            std::cout << "Wrong algorithm name.\n";
            showHelp(argv[0]);
            exit(-1);
        }
    }

    // General parameters
    pcl::console::parse_argument(argc, argv, "--corr_dist", corr_dist_);
    pcl::console::parse_argument(argc, argv, "--scene", sceneName_);
    pcl::console::parse_argument(argc, argv, "--normal_ss", normal_ss_);
    pcl::console::parse_argument(argc, argv, "--model_ss", model_ss_);
    pcl::console::parse_argument(argc, argv, "--scene_ss", scene_ss_);
    pcl::console::parse_argument(argc, argv, "--rf_rad", rf_rad_);
    pcl::console::parse_argument(argc, argv, "--descr_rad", descr_rad_);
    pcl::console::parse_argument(argc, argv, "--cg_size", cg_size_);
    pcl::console::parse_argument(argc, argv, "--cg_thresh", cg_thresh_);

    pcl::console::parse_argument(argc, argv, "--detector", detector_);
    pcl::console::parse_argument(argc, argv, "--descriptor", descriptor_);
}

void setupResolution(float resolution) {
    //  Set up resolution invariance
    if (use_cloud_resolution_) {
        if (resolution != 0.0f) {
            normal_ss_ *= resolution;
            model_ss_ *= resolution;
            scene_ss_ *= resolution;
            rf_rad_ *= resolution;
            descr_rad_ *= resolution;
            cg_size_ *= resolution;
        }

        std::cout << "Model resolution:       " << resolution << std::endl;
        std::cout << "Model sampling size:    " << model_ss_ << std::endl;
        std::cout << "Scene sampling size:    " << scene_ss_ << std::endl;
        std::cout << "LRF support radius:     " << rf_rad_ << std::endl;
        std::cout << "SHOT descriptor radius: " << descr_rad_ << std::endl;
        std::cout << "Clustering bin size:    " << cg_size_ << std::endl << std::endl;
    }
}

void read_pcd(const std::string& baseName, pcl::PointCloud<PointType>::Ptr& pnts) {
    std::string filename = modelDir + baseName;
    ROS_DEBUG("Read pcd from file: %s", filename.c_str());
    if (pcl::io::loadPCDFile<PointType>(filename, *pnts) < 0) { ROS_FATAL("Error loading model cloud."); }
}

void matchShot(const std::string& modelName,
               pcl::PointCloud<PointType>::Ptr& scene,
               pcl::PointCloud<NormalType>::Ptr& scene_normals,
               pcl::PointCloud<PointType>::Ptr& scene_keypoints,
               pcl::PointCloud<DescriptorType>::Ptr& scene_descriptors,
               std::vector<pcl::PointCloud<PointType>::ConstPtr>& registered_instances,
               std::vector<Eigen::Matrix4f>& final_tfMatrixList,
               bool calculateScene) {
    pcl::PointCloud<PointType>::Ptr model(new pcl::PointCloud<PointType>());
    pcl::PointCloud<NormalType>::Ptr model_normals(new pcl::PointCloud<NormalType>());
    pcl::PointCloud<PointType>::Ptr model_keypoints(new pcl::PointCloud<PointType>());
    pcl::PointCloud<DescriptorType>::Ptr model_descriptors(new pcl::PointCloud<DescriptorType>());
    std::string _modelname = "/srcPCD/" + modelName + ".pcd";
    read_pcd(_modelname, model);

    //  Compute Normals
    pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
    // norm_est.setKSearch (5);
    norm_est.setRadiusSearch(normal_ss_);
    norm_est.setInputCloud(model);
    norm_est.compute(*model_normals);
    if (calculateScene) {
        norm_est.setInputCloud(scene);
        norm_est.compute(*scene_normals);
    }

    //
    //  Downsample Clouds to Extract keypoints
    //

    pcl::UniformSampling<PointType> uniform_sampling;
    uniform_sampling.setInputCloud(model);
    uniform_sampling.setRadiusSearch(model_ss_);
    uniform_sampling.filter(*model_keypoints);
    std::cout << "Model total points: " << model->size() << "; Selected Keypoints: " << model_keypoints->size() << std::endl;

    if (calculateScene) {
        uniform_sampling.setInputCloud(scene);
        uniform_sampling.setRadiusSearch(scene_ss_);
        uniform_sampling.filter(*scene_keypoints);
        std::cout << "Scene total points: " << scene->size() << "; Selected Keypoints: " << scene_keypoints->size() << std::endl;
    }

    //  Compute Descriptor for keypoints
    pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
    descr_est.setRadiusSearch(descr_rad_);

    descr_est.setInputCloud(model_keypoints);
    descr_est.setInputNormals(model_normals);
    descr_est.setSearchSurface(model);
    descr_est.compute(*model_descriptors);

    if (calculateScene) {
        descr_est.setInputCloud(scene_keypoints);
        descr_est.setInputNormals(scene_normals);
        descr_est.setSearchSurface(scene);
        descr_est.compute(*scene_descriptors);
    }

    //  Find Model-Scene Correspondences with KdTree
    //
    pcl::CorrespondencesPtr model_scene_corrs(new pcl::Correspondences());

    pcl::KdTreeFLANN<DescriptorType> match_search;
    match_search.setInputCloud(model_descriptors);
    std::vector<int> model_good_keypoints_indices;
    std::vector<int> scene_good_keypoints_indices;

    //  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it
    //  to the correspondences vector.
    for (size_t i = 0; i < scene_descriptors->size(); ++i) {
        std::vector<int> neigh_indices(1);
        std::vector<float> neigh_sqr_dists(1);
        if (!pcl_isfinite(scene_descriptors->at(i).descriptor[0])) // skipping NaNs
        {
            continue;
        }
        int found_neighs = match_search.nearestKSearch(scene_descriptors->at(i), 1, neigh_indices, neigh_sqr_dists);
        if (found_neighs == 1 && neigh_sqr_dists[0] < corr_dist_) //  add match only if the squared descriptor distance
                                                                  //  is less than 0.25 (SHOT descriptor distances are
                                                                  //  between 0 and 1 by design)
        {
            pcl::Correspondence corr(neigh_indices[0], static_cast<int>(i), neigh_sqr_dists[0]);
            model_scene_corrs->push_back(corr);
            model_good_keypoints_indices.push_back(corr.index_query);
            scene_good_keypoints_indices.push_back(corr.index_match);
        }
    }
    pcl::PointCloud<PointType>::Ptr model_good_kp(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr scene_good_kp(new pcl::PointCloud<PointType>());
    pcl::copyPointCloud(*model_keypoints, model_good_keypoints_indices, *model_good_kp);
    pcl::copyPointCloud(*scene_keypoints, scene_good_keypoints_indices, *scene_good_kp);
    std::cout << "Correspondences found: " << model_scene_corrs->size() << std::endl;

    //
    //  Actual Clustering
    //
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> rototranslations;
    std::vector<pcl::Correspondences> clustered_corrs;

    //  Using Hough3D
    if (use_hough_) {
        //
        //  Compute (Keypoints) Reference Frames only for Hough
        //
        pcl::PointCloud<RFType>::Ptr model_rf(new pcl::PointCloud<RFType>());
        pcl::PointCloud<RFType>::Ptr scene_rf(new pcl::PointCloud<RFType>());

        pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est;
        rf_est.setFindHoles(true);
        rf_est.setRadiusSearch(rf_rad_);

        rf_est.setInputCloud(model_keypoints);
        rf_est.setInputNormals(model_normals);
        rf_est.setSearchSurface(model);
        rf_est.compute(*model_rf);

        rf_est.setInputCloud(scene_keypoints);
        rf_est.setInputNormals(scene_normals);
        rf_est.setSearchSurface(scene);
        rf_est.compute(*scene_rf);

        //  Clustering
        pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;
        clusterer.setHoughBinSize(cg_size_);
        clusterer.setHoughThreshold(cg_thresh_);
        clusterer.setUseInterpolation(true);
        clusterer.setUseDistanceWeight(false);

        clusterer.setInputCloud(model_keypoints);
        clusterer.setInputRf(model_rf);
        clusterer.setSceneCloud(scene_keypoints);
        clusterer.setSceneRf(scene_rf);
        clusterer.setModelSceneCorrespondences(model_scene_corrs);

        // clusterer.cluster (clustered_corrs);
        clusterer.recognize(rototranslations, clustered_corrs);
    }
    else // Using GeometricConsistency
    {
        pcl::GeometricConsistencyGrouping<PointType, PointType> gc_clusterer;
        gc_clusterer.setGCSize(cg_size_);
        gc_clusterer.setGCThreshold(cg_thresh_);

        gc_clusterer.setInputCloud(model_keypoints);
        gc_clusterer.setSceneCloud(scene_keypoints);
        gc_clusterer.setModelSceneCorrespondences(model_scene_corrs);

        // gc_clusterer.cluster (clustered_corrs);
        gc_clusterer.recognize(rototranslations, clustered_corrs);
    }

    /**
     * Stop if no instances
     */
    if (rototranslations.size() <= 0) {
        std::cout << "*** No instances found! ***" << std::endl;
        // return (0);
    }
    else {
        std::cout << "Recognized Instances: " << rototranslations.size() << endl << endl;
    }
    // ############################################################################
    /**
     * Generates clouds for each instances found
     */
    std::vector<pcl::PointCloud<PointType>::ConstPtr> instances;

    for (size_t i = 0; i < rototranslations.size(); ++i) {
        pcl::PointCloud<PointType>::Ptr rotated_model(new pcl::PointCloud<PointType>());
        pcl::transformPointCloud(*model, *rotated_model, rototranslations[i]);
        instances.push_back(rotated_model);
    }

    /**
     * ICP
     */
    if (do_icp_) {
        cout << "--- ICP ---------" << endl;
        for (size_t i = 0; i < rototranslations.size(); ++i) {
            Eigen::Matrix4f icp_tfMatrix;
            Eigen::Matrix4f final_tfMatrix;
            pcl::IterativeClosestPoint<PointType, PointType> icp;
            icp.setMaximumIterations(icp_max_iter_);
            icp.setMaxCorrespondenceDistance(icp_corr_distance_);
            // // The epsilon (difference) between the previous transformation and the current estimated transformation
            // is smaller than an user imposed value
            icp.setTransformationEpsilon(1e-7);
            // // The sum of Euclidean squared errors is smaller than a user defined threshold
            // icp.setEuclideanFitnessEpsilon(1);
            icp.setInputTarget(scene);
            icp.setInputSource(instances[i]);
            pcl::PointCloud<PointType>::Ptr registered(new pcl::PointCloud<PointType>);
            icp.align(*registered);
            // get transformation from initial recognition to fitting after icp.
            icp_tfMatrix = icp.getFinalTransformation();

            // [WARN] multiplication order should be checked A*B or B*A
            final_tfMatrix = icp_tfMatrix * rototranslations[i];
            registered_instances.push_back(registered);
            cout << "Instance " << i << " ";
            final_tfMatrixList.push_back(final_tfMatrix);
            if (icp.hasConverged()) { cout << "Aligned!" << endl; }
            else {
                cout << "Not Aligned!" << endl;
            }
        }
        cout << "-----------------" << endl << endl;
    }
    else {
        for (size_t i = 0; i < rototranslations.size(); ++i) { final_tfMatrixList.push_back(rototranslations[i]); }
    }

    //  Output results
    std::cout << "Model instances found: " << rototranslations.size() << std::endl;
    for (size_t i = 0; i < rototranslations.size(); ++i) {
        std::cout << "        Correspondences belonging to this instance: " << clustered_corrs[i].size() << std::endl;
    }

    //  Visualization before HV
    if (enableViewer_)
        viewSingleMatch(scene, model, scene_keypoints, model_keypoints, rototranslations, clustered_corrs, registered_instances);
}

void matchFpfh(const std::string& modelName,
               pcl::PointCloud<PointType>::Ptr& scene,
               pcl::PointCloud<NormalType>::Ptr& scene_normals,
               pcl::PointCloud<PointType>::Ptr& scene_keypoints,
               pcl::PointCloud<FPFHDescriptor>::Ptr& scene_descriptors,
               std::vector<pcl::PointCloud<PointType>::ConstPtr>& registered_instances,
               bool calculateScene) {
    pcl::PointCloud<PointType>::Ptr model(new pcl::PointCloud<PointType>());
    pcl::PointCloud<NormalType>::Ptr model_normals(new pcl::PointCloud<NormalType>());
    pcl::PointCloud<PointType>::Ptr model_keypoints(new pcl::PointCloud<PointType>());
    pcl::PointCloud<FPFHDescriptor>::Ptr model_descriptors(new pcl::PointCloud<FPFHDescriptor>());
    read_pcd(modelName, model);

    //  Compute Normals
    pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
    // norm_est.setKSearch (5);
    norm_est.setRadiusSearch(normal_ss_);
    norm_est.setInputCloud(model);
    norm_est.compute(*model_normals);
    if (calculateScene) {
        norm_est.setInputCloud(scene);
        norm_est.compute(*scene_normals);
    }

    //
    //  Downsample Clouds to Extract keypoints
    //

    pcl::UniformSampling<PointType> uniform_sampling;
    uniform_sampling.setInputCloud(model);
    uniform_sampling.setRadiusSearch(model_ss_);
    uniform_sampling.filter(*model_keypoints);
    std::cout << "Model total points: " << model->size() << "; Selected Keypoints: " << model_keypoints->size() << std::endl;

    if (calculateScene) {
        uniform_sampling.setInputCloud(scene);
        uniform_sampling.setRadiusSearch(scene_ss_);
        uniform_sampling.filter(*scene_keypoints);
        std::cout << "Scene total points: " << scene->size() << "; Selected Keypoints: " << scene_keypoints->size() << std::endl;
    }

    //  Compute Descriptor for keypoints
    pcl::FPFHEstimationOMP<PointType, NormalType, FPFHDescriptor> descr_est;
    descr_est.setRadiusSearch(descr_rad_);

    descr_est.setInputCloud(model_keypoints);
    descr_est.setInputNormals(model_normals);
    descr_est.setSearchSurface(model);
    descr_est.compute(*model_descriptors);

    if (calculateScene) {
        descr_est.setInputCloud(scene_keypoints);
        descr_est.setInputNormals(scene_normals);
        descr_est.setSearchSurface(scene);
        descr_est.compute(*scene_descriptors);
    }

    //  Find Model-Scene Correspondences with KdTree
    //
    pcl::CorrespondencesPtr model_scene_corrs(new pcl::Correspondences());

    pcl::KdTreeFLANN<FPFHDescriptor> match_search;
    match_search.setInputCloud(model_descriptors);

    //  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it
    //  to the correspondences vector.
    for (size_t i = 0; i < scene_descriptors->size(); ++i) {
        std::vector<int> neigh_indices(1);
        std::vector<float> neigh_sqr_dists(1);
        if (!pcl_isfinite(scene_descriptors->at(i).histogram[0])) // skipping NaNs
        {
            continue;
        }
        int found_neighs = match_search.nearestKSearch(scene_descriptors->at(i), 1, neigh_indices, neigh_sqr_dists);
        if (found_neighs == 1 && neigh_sqr_dists[0] < corr_dist_) //  add match only if the squared descriptor distance
                                                                  //  is less than 0.25 (SHOT descriptor distances are
                                                                  //  between 0 and 1 by design)
        {
            pcl::Correspondence corr(neigh_indices[0], static_cast<int>(i), neigh_sqr_dists[0]);
            model_scene_corrs->push_back(corr);
        }
    }
    std::cout << "Correspondences found: " << model_scene_corrs->size() << std::endl;

    //
    //  Actual Clustering
    //
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> rototranslations;
    std::vector<pcl::Correspondences> clustered_corrs;

    //  Using Hough3D
    if (use_hough_) {
        //
        //  Compute (Keypoints) Reference Frames only for Hough
        //
        pcl::PointCloud<RFType>::Ptr model_rf(new pcl::PointCloud<RFType>());
        pcl::PointCloud<RFType>::Ptr scene_rf(new pcl::PointCloud<RFType>());

        pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est;
        rf_est.setFindHoles(true);
        rf_est.setRadiusSearch(rf_rad_);

        rf_est.setInputCloud(model_keypoints);
        rf_est.setInputNormals(model_normals);
        rf_est.setSearchSurface(model);
        rf_est.compute(*model_rf);

        rf_est.setInputCloud(scene_keypoints);
        rf_est.setInputNormals(scene_normals);
        rf_est.setSearchSurface(scene);
        rf_est.compute(*scene_rf);

        //  Clustering
        pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;
        clusterer.setHoughBinSize(cg_size_);
        clusterer.setHoughThreshold(cg_thresh_);
        clusterer.setUseInterpolation(true);
        clusterer.setUseDistanceWeight(false);

        clusterer.setInputCloud(model_keypoints);
        clusterer.setInputRf(model_rf);
        clusterer.setSceneCloud(scene_keypoints);
        clusterer.setSceneRf(scene_rf);
        clusterer.setModelSceneCorrespondences(model_scene_corrs);

        // clusterer.cluster (clustered_corrs);
        clusterer.recognize(rototranslations, clustered_corrs);
    }
    else // Using GeometricConsistency
    {
        pcl::GeometricConsistencyGrouping<PointType, PointType> gc_clusterer;
        gc_clusterer.setGCSize(cg_size_);
        gc_clusterer.setGCThreshold(cg_thresh_);

        gc_clusterer.setInputCloud(model_keypoints);
        gc_clusterer.setSceneCloud(scene_keypoints);
        gc_clusterer.setModelSceneCorrespondences(model_scene_corrs);

        // gc_clusterer.cluster (clustered_corrs);
        gc_clusterer.recognize(rototranslations, clustered_corrs);
    }

    //
    //  Output results
    //
    std::cout << "Model instances found: " << rototranslations.size() << std::endl;
    for (size_t i = 0; i < rototranslations.size(); ++i) {
        // std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
        std::cout << "        Correspondences belonging to this instance: " << clustered_corrs[i].size() << std::endl;
    }

    if (enableViewer_)
        viewSingleMatch(scene, model, scene_keypoints, model_keypoints, rototranslations, clustered_corrs, registered_instances);
}

void matchAllModels(bool matchAllmodels,
                    std::vector<std::string>& modelList,
                    pcl::PointCloud<PointType>::Ptr& scene,
                    pcl::PointCloud<pcl::Normal>::Ptr& scene_normals,
                    pcl::PointCloud<PointType>::Ptr& scene_keypoints,
                    pcl::PointCloud<DescriptorType>::Ptr& scene_descriptors,
                    pcl::PointCloud<FPFHDescriptor>::Ptr& scene_fpfhDescriptors,
                    std::vector<int>& no_model_instances,
                    std::vector<Eigen::Matrix4f>& all_final_tfMatrixList,
                    std::vector<pcl::PointCloud<PointType>::ConstPtr>& all_registered_instances) {
    // iterate every model in the modelList for recognition
    for (size_t i = 0; i < modelList.size(); ++i) {
        std::vector<pcl::PointCloud<PointType>::ConstPtr> registered_instances;
        std::vector<Eigen::Matrix4f> final_tfMatrixList;
        std::string modelName = modelList[i];
        std::cout << "--- Start recognize " << modelName << " ----" << std::endl;

        if (descriptor_ == "shot") {
            // matching a single model
            matchShot(modelName, scene, scene_normals, scene_keypoints, scene_descriptors, registered_instances, final_tfMatrixList, true);
            no_model_instances.push_back(static_cast<int>(registered_instances.size()));
        }
        else if (descriptor_ == "fpfh") {
            matchFpfh(modelName, scene, scene_normals, scene_keypoints, scene_fpfhDescriptors, registered_instances, true);
        }
        else {
            std::cout << "please enter shot or fpfh(not implemented) for descriptor" << std::endl;
        }
        all_final_tfMatrixList.insert(all_final_tfMatrixList.end(), final_tfMatrixList.begin(), final_tfMatrixList.end());
        all_registered_instances.insert(
            all_registered_instances.end(), registered_instances.begin(), registered_instances.end());
    }
}

void viewSingleMatch(pcl::PointCloud<PointType>::Ptr& scene,
                     pcl::PointCloud<PointType>::Ptr& model,
                     pcl::PointCloud<PointType>::Ptr& scene_keypoints,
                     pcl::PointCloud<PointType>::Ptr& model_keypoints,
                     std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>& rototranslations,
                     std::vector<pcl::Correspondences>& clustered_corrs,
                     std::vector<pcl::PointCloud<PointType>::ConstPtr>& registered_instances) {
    //
    //  Visualization before HV
    //
    pcl::visualization::PCLVisualizer viewer("Correspondence Grouping");
    viewer.addPointCloud(scene, "scene_cloud");

    pcl::PointCloud<PointType>::Ptr off_scene_model(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr off_scene_model_keypoints(new pcl::PointCloud<PointType>());

    if (show_correspondences_ || show_keypoints_) {
        //  We are translating the model so that it doesn't end in the middle of the scene representation
        pcl::transformPointCloud(*model, *off_scene_model, Eigen::Vector3f(-1, 0, 0), Eigen::Quaternionf(1, 0, 0, 0));
        pcl::transformPointCloud(
            *model_keypoints, *off_scene_model_keypoints, Eigen::Vector3f(-1, 0, 0), Eigen::Quaternionf(1, 0, 0, 0));

        pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_color_handler(off_scene_model, 255, 255, 128);
        viewer.addPointCloud(off_scene_model, off_scene_model_color_handler, "off_scene_model");
    }

    if (show_keypoints_) {
        pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_keypoints_color_handler(scene_keypoints, 0, 0, 255);
        viewer.addPointCloud(scene_keypoints, scene_keypoints_color_handler, "scene_keypoints");
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keypoints");

        pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_keypoints_color_handler(
            off_scene_model_keypoints, 0, 0, 255);
        viewer.addPointCloud(
            off_scene_model_keypoints, off_scene_model_keypoints_color_handler, "off_scene_model_keypoints");
        viewer.setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "off_scene_model_keypoints");
    }

    for (size_t i = 0; i < rototranslations.size(); ++i) {
        pcl::PointCloud<PointType>::Ptr rotated_model(new pcl::PointCloud<PointType>());
        pcl::transformPointCloud(*model, *rotated_model, rototranslations[i]);

        std::stringstream ss_cloud;
        ss_cloud << "instance" << i;

        pcl::visualization::PointCloudColorHandlerCustom<PointType> rotated_model_color_handler(rotated_model, 255, 0, 0);
        viewer.addPointCloud(rotated_model, rotated_model_color_handler, ss_cloud.str());

        if (show_correspondences_) {
            for (size_t j = 0; j < clustered_corrs[i].size(); ++j) {
                std::stringstream ss_line;
                ss_line << "correspondence_line" << i << "_" << j;
                PointType& model_point = off_scene_model_keypoints->at(clustered_corrs[i][j].index_query);
                PointType& scene_point = scene_keypoints->at(clustered_corrs[i][j].index_match);

                //  We are drawing a line for each pair of clustered correspondences found between the model and the
                //  scene
                viewer.addLine<PointType, PointType>(model_point, scene_point, 0, 255, 0, ss_line.str());
            }
        }
    }

    // add icp_registered cloud
    if (do_icp_ && registered_instances.size() != 0) {
        for (size_t i = 0; i < registered_instances.size(); ++i) {
            std::stringstream ss_icp;
            ss_icp << "registered_instance" << i;
            pcl::visualization::PointCloudColorHandlerCustom<PointType> icp_registered_model_color_handler(
                registered_instances[i], 0, 255, 0);
            viewer.addPointCloud(registered_instances[i], icp_registered_model_color_handler, ss_icp.str());
        }
    }
    else if (do_icp_ && registered_instances.size() == 0) {
        std::cout << "[WARN] enabled doICP but no registered_instances, probably it's because ICP not yet implemented "
                     "with FPFH"
                  << std::endl;
    }

    while (!viewer.wasStopped()) { viewer.spinOnce(); }
}

void hypothesis_verification(const pcl::PointCloud<PointType>::Ptr& scene,
                             std::vector<pcl::PointCloud<PointType>::ConstPtr>& registered_instances,
                             std::vector<bool>& hypotheses_mask) {
    /**
       * Hypothesis Verification
       */
    std::cout << "--- Hypotheses Verification ---" << std::endl;
    // std::vector<bool> hypotheses_mask;  // Mask Vector to identify positive hypotheses

    pcl::GlobalHypothesesVerification<PointType, PointType> GoHv;

    GoHv.setSceneCloud(scene);                  // Scene Cloud
    GoHv.addModels(registered_instances, true); // Models to verify
    GoHv.setResolution(hv_resolution_);
    GoHv.setResolutionOccupancyGrid(hv_occupancy_grid_resolution_);
    GoHv.setInlierThreshold(hv_inlier_th_);
    GoHv.setOcclusionThreshold(hv_occlusion_th_);
    GoHv.setRegularizer(hv_regularizer_);
    GoHv.setRadiusClutter(hv_rad_clutter_);
    GoHv.setClutterRegularizer(hv_clutter_reg_);
    GoHv.setDetectClutter(hv_detect_clutter_);
    GoHv.setRadiusNormals(hv_rad_normals_);

    GoHv.verify();
    GoHv.getMask(hypotheses_mask); // i-element TRUE if hvModels[i] verifies hypotheses

    for (int i = 0; i < hypotheses_mask.size(); i++) {
        if (hypotheses_mask[i]) { std::cout << "Instance " << i << " is GOOD! <---" << std::endl; }
        else {
            std::cout << "Instance " << i << " is bad!" << std::endl;
        }
    }
    std::cout << "-------------------------------" << std::endl;
}

void view_hv_result(bool enableViewer,
                    const pcl::PointCloud<PointType>::Ptr& scene,
                    std::vector<bool>& hypothesesMask,
                    std::vector<pcl::PointCloud<PointType>::ConstPtr>& all_registered_instances) {
    // if (static_cast<int>( all_registered_instances.size ()) > 0) {
    // std::vector<bool> hypothesesMask;
    // hypothesis_verification(scene, all_registered_instances, hypothesesMask);

    pcl::visualization::PCLVisualizer viewer("Hypotheses Verification");
    viewer.addPointCloud(scene, "scene_cloud");

    for (size_t i = 0; i < all_registered_instances.size(); ++i) {
        std::stringstream ss_instance;

        if (enableViewer) {
            // Instances after ICP, if pass hypothesis verification then it is green, if not then cyan.
            CloudStyle registeredStyles = hypothesesMask[i] ? style_green : style_cyan;
            ss_instance << "registered_instance_" << i << std::endl;
            pcl::visualization::PointCloudColorHandlerCustom<PointType> registered_instance_color_handler(
                all_registered_instances[i], registeredStyles.r, registeredStyles.g, registeredStyles.b);
            viewer.addPointCloud(all_registered_instances[i], registered_instance_color_handler, ss_instance.str());
            viewer.setPointCloudRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_POINT_SIZE, registeredStyles.size, ss_instance.str());
        }
    }
    while (!viewer.wasStopped() && enableViewer) { viewer.spinOnce(); }
}

void readAllmodels(std::vector<std::string>& modelList, std::vector<pcl::PointCloud<PointType>::Ptr>& modelpcdList) {
    for (size_t i = 0; i < modelList.size(); ++i) {
        std::string modelName = modelList[i];
        std::string modelfilename = "/srcPCD/" + modelList[i] + ".pcd";
        pcl::PointCloud<PointType>::Ptr modelpcd(new pcl::PointCloud<PointType>());
        read_pcd(modelfilename, modelpcd);
        modelpcdList.push_back(modelpcd);
    }
}

void __test_final_tranformation(const pcl::PointCloud<PointType>::Ptr& scene,
                                std::vector<pcl::PointCloud<PointType>::Ptr>& modelpcdList,
                                std::vector<int>& no_model_instances,
                                std::vector<Eigen::Matrix4f>& all_final_tfMatrixList) {
    /* Show all the model pcd after transformed, comparing it with the output from viewer in main function
     *
     *
    */
    pcl::visualization::PCLVisualizer viewer("Hypotheses Verification check transformation matrix");
    viewer.addPointCloud(scene, "scene_cloud");

    std::vector<pcl::PointCloud<PointType>::ConstPtr> instances;
    int k = 0;
    int startIdx = 0;
    for (int j = 0; j < static_cast<int>(no_model_instances.size()); ++j) {
        int endIdx = startIdx + static_cast<int>(no_model_instances[j]);
        for (startIdx; k < endIdx; k++) {
            pcl::PointCloud<PointType>::Ptr rotated_model(new pcl::PointCloud<PointType>());
            pcl::PointCloud<PointType>::Ptr final_model(new pcl::PointCloud<PointType>());
            pcl::transformPointCloud(*modelpcdList[j], *rotated_model, all_final_tfMatrixList[k]);
            instances.push_back(rotated_model);
        }
        startIdx = no_model_instances[j];
    }

    // visualize the testing transformed models
    for (size_t i = 0; i < instances.size(); ++i) {
        std::stringstream ss_instance;
        ss_instance << "test_tf_" << i;
        CloudStyle registeredStyles = style_cyan;
        pcl::visualization::PointCloudColorHandlerCustom<PointType> instance_color_handler(
            instances[i], registeredStyles.r, registeredStyles.g, registeredStyles.b);
        viewer.addPointCloud(instances[i], instance_color_handler, ss_instance.str());
        viewer.setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, registeredStyles.size, ss_instance.str());
    }

    while (!viewer.wasStopped()) { viewer.spinOnce(); }
}
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

#ifndef MATCHINGLOCALPIPELINE_H
#define MATCHINGLOCALPIPELINE_H
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/correspondence.h>
#include <pcl/features/board.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/hv/hv_go.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include "impl/utils.cpp"
#include "impl/yamlSimpleLoad.cpp"

typedef pcl::PointXYZ PointType;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT352 DescriptorType;
typedef pcl::FPFHSignature33 FPFHDescriptor;

struct CloudStyle
{
  double r;
  double g;
  double b;
  double size;

  CloudStyle(double r, double g, double b, double size) : r(r), g(g), b(b), size(size)
  {
  }
};

struct RecogResult {
  Eigen::Matrix4f tf_mat;
  std::string modelName;
};

CloudStyle style_white(255.0, 255.0, 255.0, 4.0);
CloudStyle style_red(255.0, 0.0, 0.0, 3.0);
CloudStyle style_green(0.0, 255.0, 0.0, 5.0);
CloudStyle style_cyan(93.0, 200.0, 217.0, 4.0);
CloudStyle style_violet(255.0, 0.0, 255.0, 8.0);

std::string modelDir_("/root/exchange/tempData");
std::string modelName_("lf064-05");
std::string sceneName_("/filtered/scene2_points_.pcd");
std::vector<std::string> modelList_({ "lf064-01", "lf064-02", "lf064-03", "lf064-04", "lf064-05" });
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

// after confirmation the correctness of final_tfMatrixList, registered_instances can be removed and
// generate it outside
// the function
void matchShot(const std::string& modelName, pcl::PointCloud<PointType>::Ptr& scene,
               pcl::PointCloud<NormalType>::Ptr& scene_normals, pcl::PointCloud<PointType>::Ptr& scene_keypoints,
               pcl::PointCloud<DescriptorType>::Ptr& scene_descriptors,
               std::vector<pcl::PointCloud<PointType>::ConstPtr>& registered_instances,
               std::vector<Eigen::Matrix4f>& final_tfMatrixList, bool calculateScene = true);

void matchFpfh(const std::string& modelName, pcl::PointCloud<PointType>::Ptr& scene,
               pcl::PointCloud<NormalType>::Ptr& scene_normals, pcl::PointCloud<PointType>::Ptr& scene_keypoints,
               pcl::PointCloud<FPFHDescriptor>::Ptr& scene_descriptors,
               std::vector<pcl::PointCloud<PointType>::ConstPtr>& registered_instances, bool calculateScene = true);

void viewSingleMatch(pcl::PointCloud<PointType>::Ptr& scene, pcl::PointCloud<PointType>::Ptr& model,
                     pcl::PointCloud<PointType>::Ptr& scene_keypoints, pcl::PointCloud<PointType>::Ptr& model_keypoints,
                     std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>& rototranslations,
                     std::vector<pcl::Correspondences>& clustered_corrs,
                     std::vector<pcl::PointCloud<PointType>::ConstPtr>& registered_instances);

void matchAllModels(bool matchAllmodels, pcl::PointCloud<PointType>::Ptr& scene,
                    pcl::PointCloud<pcl::Normal>::Ptr& scene_normals, pcl::PointCloud<PointType>::Ptr& scene_keypoints,
                    pcl::PointCloud<DescriptorType>::Ptr& scene_descriptors,
                    pcl::PointCloud<FPFHDescriptor>::Ptr& scene_fpfhDescriptors, std::vector<int>& no_model_instances,
                    std::vector<Eigen::Matrix4f>& all_final_tfMatrixList,
                    std::vector<pcl::PointCloud<PointType>::ConstPtr>& all_registered_instances);

void hypothesis_verification(const pcl::PointCloud<PointType>::Ptr& scene,
                             std::vector<pcl::PointCloud<PointType>::ConstPtr>& registered_instances,
                             std::vector<bool>& hypotheses_mask);
void __test_final_tranformation(const pcl::PointCloud<PointType>::Ptr& scene,
                                std::vector<pcl::PointCloud<PointType>::Ptr>& modelpcdList,
                                std::vector<int>& no_model_instances,
                                std::vector<Eigen::Matrix4f>& all_final_tfMatrixList);

void view_hv_result(const pcl::PointCloud<PointType>::Ptr& scene, std::vector<bool>& hypothesesMask,
                    std::vector<pcl::PointCloud<PointType>::ConstPtr>& all_registered_instances);

void recognize(const std::string& configFilePath, std::vector<RecogResult> &accepted, std::vector<RecogResult> &rejected);
void recognize(const std::string& configFilePath);
bool fakeArgvByYaml(const std::string& yamlPath);

#endif
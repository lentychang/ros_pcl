#include "../yamlSimpleLoad.h"
bool __test(){
    // instantiate all parameters
    std::string yamlFile = "/root/catkin_ws/src/ros_pcl/config/test.yaml";

    std::string modelDir_ ("/root/exchange/tempData");
    std::string modelName_("lf064-05");
    std::string sceneName_("/filtered/scene2_points_.pcd");
    bool disableModelList_(false);
    std::vector<std::string> modelList;

    std::string detector_ = "uni";
    std::string descriptor_ = "shot";

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

    // Start to load parameters from yaml
    try{
        LOAD_YAML yaml(yamlFile);
        
        yaml.load<std::string>("model_dir", modelDir_);
        yaml.load<std::string>("model_name", modelName_);
        yaml.loadStringList("model_list", modelList);
        yaml.load<std::string>("scene_name", sceneName_);
        yaml.load<bool>("recog_single_model", disableModelList_);
        yaml.load<std::string>("detector", detector_);
        yaml.load<std::string>("descriptor", descriptor_);
        yaml.load<float>("normal_radius", normal_ss_);
        yaml.load<float>("model_downsample_size", model_ss_);
        yaml.load<float>("scene_downsample_size", scene_ss_);
        yaml.load<float>("descriptor_radius", descr_rad_);

        yaml.load<bool>("use_hough", use_hough_);
        yaml.load<float>("rf_rad", rf_rad_);
        yaml.load<float>("corr_dist", corr_dist_);
        yaml.load<float>("cg_size", cg_size_);
        yaml.load<float>("cg_thres", cg_thresh_);

        yaml.load<bool>("do_icp", do_icp_);
        yaml.load<int>("icp_max_iter", icp_max_iter_);
        yaml.load<float>("icp_corr_dist", icp_corr_distance_);

        yaml.load<float>("hv_resolution", hv_resolution_);
        yaml.load<float>("hv_occupancy_grid_resolution", hv_occupancy_grid_resolution_);
        yaml.load<float>("hv_clutter_reg", hv_clutter_reg_);
        yaml.load<float>("hv_inliner_thres", hv_inlier_th_);
        yaml.load<float>("hv_occlusion_thres", hv_occlusion_th_);
        yaml.load<float>("hv_rad_clutter", hv_rad_clutter_);
        yaml.load<float>("hv_regularizer", hv_regularizer_);
        yaml.load<float>("hv_rad_normals", hv_rad_normals_);
        yaml.load<bool>("hv_detect_clutter", hv_detect_clutter_);

        yaml.load<bool>("enableViewer", enableViewer_);
        yaml.load<bool>("show_keypoints", show_keypoints_);
        yaml.load<bool>("show_correspondences", show_correspondences_);
    }
    catch (const std::string& msg){
        std::cerr << msg << std::endl;
        return false;
    }
    return true;
}

// g++ -L/usr/local/lib -I/usr/local/include -std=c++11 test_yaml.cpp -o testYaml -lyaml-cpp

LOAD_YAML::LOAD_YAML(const std::string &yamlFile){
    yamlNode = YAML::LoadFile(yamlFile);
    __debug = false;
}

template<typename T>
void LOAD_YAML::load(const std::string& key, T &val){
    if (yamlNode[key]) {
        val = yamlNode[key].as<T>();
        if (__debug) {
            std::cout << key << "("<< typeid(val).name() << "): " << val << std::endl;
        }
    }
    else {
        std::string err_msg = "key: [" + static_cast<std::string>(key) + "] doesn't exist!!";
        throw err_msg;
    }
}

void LOAD_YAML::loadStringList(const std::string& key, std::vector<std::string>& vec){
    if (yamlNode[key]){
        vec = yamlNode[key].as<std::vector<std::string>>();
        if (__debug) {
            for (const auto &modelName : vec){
                std::cout << key << " (vector<string>): " << modelName << std::endl;
            }
        }
    }
    else {
        std::string err_msg = "[ERROR] key: [" + static_cast<std::string>(key) + "] doesn't exist!!";
        throw err_msg;
    }
}
#include "yaml_reader.hpp"

namespace test_input{
YamlReader::YamlReader(std::string base_path, std::string which_car)
:base_path_(base_path), which_car_(which_car)
{
    if(which_car_ == "/car1"){
        camera_int_path_ =  base_path_ + which_car_ + "/front_110_intrinsic.yaml";
        camera_ext_path_ =  base_path_ + which_car_ + "/front_110.yaml";
        lidarL_ext_path_ =  base_path_ + which_car_ + "/left_m1.yaml";
        lidarR_ext_path_ = base_path_ + which_car_ + "/right_m1.yaml";
        camera_ext_name_ = "front110_to_vehicle_rt";
        lidarL_ext_name_ = "leftm1_to_vehicle_rt";
        lidarR_ext_name_ = "rightm1_to_vehicle_rt";
    }
    else if(which_car_ == "/car2"){
        camera_int_path_ =  base_path_ + which_car_ + "/front_panoramic_intrinsic.yaml";
        camera_ext_path_ =  base_path_ + which_car_ + "/front_panoramic.yaml";
        lidarL_ext_path_ =  base_path_ + which_car_ + "/left_m1.yaml";
        lidarR_ext_path_ = base_path_ + which_car_ + "/right_m1.yaml";
        camera_ext_name_ = "frontpanoramic_to_vehicle_rt";
        lidarL_ext_name_ = "leftm1_to_vehicle_rt";
        lidarR_ext_name_ = "rightm1_to_vehicle_rt";
    }
    else if(which_car_ == "/Kitti"){
        camera_int_path_ = base_path_ + which_car_ + "/camera_intrinsic.yaml";
        camera_ext_path_ =  base_path_ + which_car_ + "/camera.yaml";
        lidarL_ext_path_ =  base_path_ + which_car_ + "/radar.yaml";
        lidarR_ext_path_ = base_path_ + which_car_ + "/radar_.yaml";
        camera_ext_name_ = "camera_to_vehicle";
        lidarL_ext_name_ = "radar_to_vehicle";
        lidarR_ext_name_ = "rader__to_vehicle";
    }
}

bool YamlReader::Read(){
    std::cout << "Reading camera intrinsic params..." << std::endl;
    if(!IntrinsicReader(params_)){
        std::cout << "Failed!" << std::endl;
        return false;
    }
    std::cout << "Sucess! Reading camera external params..." << std::endl;
    if(!ExternalReader(params_.Tvc, camera_ext_path_, camera_ext_name_)){
        std::cout << "Failed!" << std::endl;
        return false;
    }
    std::cout << "Sucess! Reading LeftLidar external params..." << std::endl;
    if(!ExternalReader(params_.TvlL, lidarL_ext_path_, lidarL_ext_name_)){
        std::cout << "Failed!" << std::endl;
        return false;
    }
    std::cout << "Sucess! Reading RightLidar external params..." << std::endl;
    if(!ExternalReader(params_.TvlR, lidarR_ext_path_, lidarR_ext_name_)){
        std::cout << "Failed!" << std::endl;
        return false;
    }
    // std::cout << "Sucess! Reading Imu external params..." << std::endl;
    // if(!ExternalReader(params_.Tvi, imu_ext_path_)){
    //     std::cout << "Failed!" << std::endl;
    //     return false;
    // }

    std::cout << "Basic Parameters:" << std::endl;
    std::cout << "- fx: " << params_.K.at<double>(0, 0) << std::endl;
    std::cout << "- fy: " << params_.K.at<double>(1, 1) << std::endl;
    std::cout << "- cx: " << params_.K.at<double>(0, 2) << std::endl;
    std::cout << "- cy: " << params_.K.at<double>(1, 2) << std::endl;
    std::cout << "------------------------------" << std::endl;
    std::cout << "- k1: " << params_.distortion_coeffs.at<double>(0, 0) <<std:: endl;
    std::cout << "- k2: " << params_.distortion_coeffs.at<double>(1, 0) << std::endl;
    std::cout << "- p1: " << params_.distortion_coeffs.at<double>(2, 0) << std::endl;
    std::cout << "- p2: " << params_.distortion_coeffs.at<double>(3, 0) << std::endl;
    std::cout << "------------------------------" << std::endl;
    std::cout << "Tranform from camera to vehicle is: " << std::endl;
    std::cout << params_.Tvc << std::endl;
    std::cout <<"Transform from left lidar to vehicle is: " << std::endl;
    std::cout << params_.TvlL << std::endl;
    std::cout <<"Transform from right lidar to vehicle is: " << std::endl;
    std::cout << params_.TvlR << std::endl;
    return true;
}

bool YamlReader::IntrinsicReader(Params& params){
    cv::FileStorage int_file_storage(camera_int_path_.c_str(), cv::FileStorage::READ);
    if(!int_file_storage.isOpened())
    {
        std::cout << "Failed to open settings file at: " << camera_int_path_ << std::endl;
        return false;
    }
    cv::Mat K_ = cv::Mat::eye(3, 3, CV_64F);
    std::vector<double> vecK(4);
    int_file_storage["cam0"]["intrinsics"] >> vecK;
    K_.at<double>(0, 0) = vecK[0];
    K_.at<double>(1, 1) = vecK[1];
    K_.at<double>(0, 2) = vecK[2];
    K_.at<double>(1, 2) = vecK[3];
    K_.copyTo(params.K);

    cv::Mat distortion_coeffs_ = cv::Mat::zeros(4, 1, CV_64F);
    std::vector<double> vecCof(4);
    int_file_storage["cam0"]["distortion_coeffs"] >> vecCof;
    distortion_coeffs_.at<double>(0, 0) = vecCof[0];
    distortion_coeffs_.at<double>(1, 0) = vecCof[1];
    distortion_coeffs_.at<double>(2, 0) = vecCof[2];
    distortion_coeffs_.at<double>(3, 0) = vecCof[3];
    distortion_coeffs_.copyTo(params.distortion_coeffs);

    std::vector<int> resolution(2);
    int_file_storage["cam0"]["resolution"] >> resolution;
    params.resolution.first = resolution[0];
    params.resolution.second = resolution[1];

    int_file_storage["cam0"]["distortion_model"] >> params.distortion_model;

    return true;
}

bool YamlReader::ExternalReader(cv::Mat& T, std::string path, std::string Tname){
    cv::FileStorage extc_file_storage(path.c_str(), cv::FileStorage::READ);
    if(!extc_file_storage.isOpened())
    {
        std::cout << "Failed to open settings file at: " << camera_int_path_ << std::endl;
        return false;
    }
    extc_file_storage[Tname] >> T;
    return true;
}
}
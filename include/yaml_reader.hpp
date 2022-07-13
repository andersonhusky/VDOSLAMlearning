/******************
* @Description：yaml读取类
* @Author：hongfeng
* @Date：2022/06/22
******************/

#ifndef YAML_READER_YAML_READER_HPP_
#define YAML_READER_YAML_READER_HPP_

#include <opencv2/core/core.hpp>
#include <iostream>

namespace test_input{
class YamlReader{
    public:
        class Params{
            public:
                cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
                cv::Mat distortion_coeffs = cv::Mat::zeros(4, 1, CV_64F);
                cv::Mat Tvc = cv::Mat::eye(4, 4, CV_64F);
                cv::Mat TvlL = cv::Mat::eye(4, 4, CV_64F);
                cv::Mat TvlR = cv::Mat::eye(4, 4, CV_64F);
                cv::Mat Tvi = cv::Mat::eye(4, 4, CV_64F);
                std::pair<int, int> resolution;
                std::string distortion_model;
        };
    public:
        YamlReader(std::string base_path, std::string which_path);
        YamlReader() = default;

        bool Read();
        bool IntrinsicReader(Params& params);
        bool ExternalReader(cv::Mat& T, std::string path, std::string Tname);

        Params params_;
    private:
        std::string base_path_;
        std::string which_car_;

        std::string camera_int_path_;
        std::string camera_ext_path_;
        std::string lidarL_ext_path_;
        std::string lidarR_ext_path_;
        std::string imu_ext_path_;

        std::string camera_ext_name_;
        std::string lidarL_ext_name_;
        std::string lidarR_ext_name_;
        std::string imu_ext_name_;
        
};
}

#endif
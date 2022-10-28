#include <iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<unistd.h>

#include<opencv2/core/core.hpp>
#include<opencv2/optflow.hpp>

#include<System.h>
#include<yaml_reader.hpp>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>                                        //PCL对各种格式的点的支持头文件
#include <pcl/io/pcd_io.h>                                              //PCL的PCD格式文件的输入输出头文件
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include<pcl/visualization/pcl_visualizer.h>

using namespace std;

std::string base_path = "/home/hongfeng/code/VDO_SLAM/params";
std::string which_car = "/Kitti";

// car, bus, train, motorcycle, bicucle, rider, person
vector<cv::Vec3b> mask_type{cv::Vec3b{142, 0, 0}, cv::Vec3b{70, 0, 0}, cv::Vec3b{100, 80, 0}, cv::Vec3b{230, 0, 0}, 
                                                cv::Vec3b{32, 11, 119}, cv::Vec3b{100, 60, 0}, cv::Vec3b {0, 0, 255}, cv::Vec3b{60, 20, 200}};

void LoadData(const string & strPathToSequence, vector<string> &vstrFilenamesRGB, vector<string> &vstrFilenamesDEP,
            vector<string> &vstrFilenamesOBJPCD, vector<string> &vstrFilenamesSEM, vector<string> &vstrFilenamesFLO, 
            vector<double> &vTimestamps, vector<cv::Mat> &vPoseGT, vector<vector<float> > &vObjPoseGT);
void LoadObjPcd(const string &strPathObjPcd, vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &imObjPcl, const test_input::YamlReader &yaml_reader);
void LoadMask(const string &strFilenamesMask, cv::Mat &imMask);

int main(int argc, char **argv){
    if(argc != 3){
        cerr << endl << "Usage: rosrun Lidar2Depth VDO pah_to_settings path_to_sequence" << endl;
        return 1;
    }
    test_input::YamlReader yaml_reader(base_path, which_car);
    yaml_reader.Read();

    vector<string> vstrFilenamesRGB;
    vector<string> vstrFilenamesDEP;
    vector<string> vstrFilenamesOBJPCD;
    vector<string> vstrFilenamesSEM;
    vector<string> vstrFilenamesFLO;

    std::vector<cv::Mat> vPoseGT;
    vector<vector<float> > vObjPoseGT;
    vector<double> vTimestamps;

    LoadData(argv[2], vstrFilenamesRGB, vstrFilenamesDEP, vstrFilenamesOBJPCD, vstrFilenamesSEM, vstrFilenamesFLO, 
                    vTimestamps, vPoseGT, vObjPoseGT);
    
    // obj_pose文件中那些行属于f_id帧
    vector<vector<int> > vObjPoseID(vstrFilenamesRGB.size());
    for (int i = 0; i < vObjPoseGT.size(); ++i)
    {
        // 取object位姿所在帧，存储一帧内有那些object
        int f_id = vObjPoseGT[i][0];
        if (f_id>=vstrFilenamesRGB.size())
            break;
        vObjPoseID[f_id].push_back(i);
    }

    int nImages = vstrFilenamesRGB.size()-1;
    if(vstrFilenamesRGB.empty())
    {
        cerr << endl << "No images found in provided path." << endl;
        return 1;
    }
    else if(vstrFilenamesDEP.size()!=vstrFilenamesRGB.size())
    {
        cerr << endl << "Different number of images for depth map." << endl;
        return 1;
    }
    else if(vstrFilenamesOBJPCD.size()!=vstrFilenamesRGB.size())
    {
        cerr << endl << "Different number of pcd folder for obj_pcd." << endl;
        return 1;
    }
    else if(vstrFilenamesSEM.size()!=vstrFilenamesRGB.size())
    {
        cerr << endl << "Different number of images for mask." << endl;
        return 1;
    }
    else if(vstrFilenamesFLO.size()!=vstrFilenamesRGB.size())
    {
        cerr << endl << "Different number of images for flow map." << endl;
        return 1;
    }

    VDO_SLAM::System SLAM(argv[1], VDO_SLAM::System::RGBD);

    cout << endl << "--------------------------------------------------------------------------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    cv::Mat imTraj(1000, 1000, CV_8UC3, cv::Scalar(255,255,255));
    cv::Mat imRGB, imD, mTcw_gt;

    for(int ni=0; ni<56; ni++)
    {
        cout << endl;
        cout << "=======================================================" << endl;
        cout << "Processing Frame: " << ni << endl;

        // Read imreadmage and depthmap from file
        imRGB = cv::imread(vstrFilenamesRGB[ni],CV_LOAD_IMAGE_UNCHANGED);
        imD   = cv::imread(vstrFilenamesDEP[ni],CV_LOAD_IMAGE_UNCHANGED);
        cv::Mat imD_f = cv::Mat::zeros(imRGB.rows, imRGB.cols, CV_32F);
        // imD.convertTo(imD_f, CV_32F);

        // 光流结果，保存的是当前帧到下一帧的光流结果
        cv::Mat imFlow = cv::optflow::readOpticalFlow(vstrFilenamesFLO[ni]);
        // FlowShow(imFlow);

        cv::Mat imSem = cv::Mat::zeros(imRGB.rows, imRGB.cols, CV_32SC1);
        LoadMask(vstrFilenamesSEM[ni],imSem);

        vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> imObjPcl;
        LoadObjPcd(vstrFilenamesOBJPCD[ni], imObjPcl, yaml_reader);

        // 时间戳和位姿真值
        double tframe = vTimestamps[ni];
        mTcw_gt = vPoseGT[ni];

        // 当前帧的出现的obj信息
        vector<vector<float>> vObjPose_gt(vObjPoseID[ni].size());
        for (int i = 0; i < vObjPoseID[ni].size(); ++i)
            vObjPose_gt[i] = vObjPoseGT[vObjPoseID[ni][i]];

        if(imRGB.empty())
        {
            cerr << endl << "Failed to load image at: " << vstrFilenamesRGB[ni] << endl;
            return 1;
        }

        // Pass the image to the SLAM system
        SLAM.TrackRGBD(imRGB, imObjPcl, imFlow, imSem, mTcw_gt, vObjPose_gt, tframe, imTraj, 56);

    }
}

void LoadData(const string & strPathToSequence, vector<string> &vstrFilenamesRGBg, vector<string> &vstrFilenamesDEP,
            vector<string> &vstrFilenamesOBJPCD, vector<string> &vstrFilenamesSEM, vector<string> &vstrFilenamesFLO, 
            vector<double> &vTimestamps, vector<cv::Mat> &vPoseGT, vector<vector<float> > &vObjPoseGT){
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof()){
        string s;
        getline(fTimes, s);
        if(!s.empty()){
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }
    fTimes.close();

    string strPrefixImage = strPathToSequence + "/img/";
    string strPrefixDepth = strPathToSequence + "/depth/";
    string strPrefixObjPcd = strPathToSequence + "/obj_pcd/";
    string strPrefixSEM = strPathToSequence + "/mask/";
    string strPrefixFlow = strPathToSequence + "/flow/";

    const int nTimes = vTimestamps.size();
    vstrFilenamesRGB.resize(nTimes);
    vstrFilenamesDEP.resize(nTimes);
    vstrFilenamesOBJPCD.resize(nTimes);
    vstrFilenamesSEM.resize(nTimes);
    vstrFilenamesFLO.resize(nTimes);

    for(int i=0; i<nTimes; ++i){
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrFilenamesRGB[i] = strPrefixImage + ss.str() + ".png";
        vstrFilenamesDEP[i] = strPrefixDepth + ss.str() + ".png";
        vstrFilenamesOBJPCD[i] = strPrefixObjPcd + ss.str() + "/";
        vstrFilenamesSEM[i] = strPrefixSEM + ss.str() + ".png";
        vstrFilenamesFLO[i] = strPrefixFlow + ss.str() + ".flo";
    }

    string strFilenamePose = strPathToSequence + "/pose_gt.txt";
    ifstream fPose;
    fPose.open(strFilenamePose.c_str());
    while(!fPose.eof()){
        string s;
        getline(fPose, s);
        if(!s.empty()){
            stringstream ss;
            ss << s;
            int t;
            ss >> t;
            cv::Mat Pose_tmp = cv::Mat::eye(4,4,CV_32F);
            ss >> Pose_tmp.at<float>(0,0) >> Pose_tmp.at<float>(0,1) >> Pose_tmp.at<float>(0,2) >> Pose_tmp.at<float>(0,3)
               >> Pose_tmp.at<float>(1,0) >> Pose_tmp.at<float>(1,1) >> Pose_tmp.at<float>(1,2) >> Pose_tmp.at<float>(1,3)
               >> Pose_tmp.at<float>(2,0) >> Pose_tmp.at<float>(2,1) >> Pose_tmp.at<float>(2,2) >> Pose_tmp.at<float>(2,3)
               >> Pose_tmp.at<float>(3,0) >> Pose_tmp.at<float>(3,1) >> Pose_tmp.at<float>(3,2) >> Pose_tmp.at<float>(3,3);

            vPoseGT.push_back(Pose_tmp);
        }
    }
    fPose.close();

    string strFilenameObjPose = strPathToSequence + "/object_pose.txt";
    ifstream fObjPose;
    fObjPose.open(strFilenameObjPose.c_str());

    while(!fObjPose.eof())
    {
        string s;
        getline(fObjPose,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;

            std::vector<float> ObjPose_tmp(10,0);
            ss >> ObjPose_tmp[0] >> ObjPose_tmp[1] >> ObjPose_tmp[2] >> ObjPose_tmp[3]
               >> ObjPose_tmp[4] >> ObjPose_tmp[5] >> ObjPose_tmp[6] >> ObjPose_tmp[7]
               >> ObjPose_tmp[8] >> ObjPose_tmp[9];

            vObjPoseGT.push_back(ObjPose_tmp);

        }
    }
    fObjPose.close();
}

void LoadObjPcd(const string &strPathObjPcd, vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &imObjPcl, const test_input::YamlReader &yaml_reader){
    //  确定obj文件个数
    DIR *pDir;
    struct dirent* ptr;
    int pcd_file_num=0;
    if(!(pDir = opendir(strPathObjPcd.c_str())))
        return;
    while((ptr=readdir(pDir))!=0){
        if(strcmp(ptr->d_name,".")!=0&&strcmp(ptr->d_name,"..")!=0 )
        pcd_file_num++;
    }
    closedir(pDir);

    for(int pcd_num=0, pcd_idx=0; pcd_num<pcd_file_num; ++pcd_num, ++pcd_idx){
        std::string strFilenameObjPcd = strPathObjPcd + "cloud_cluster_" + std::to_string(pcd_idx) + ".pcd";
        pcl::PointCloud<pcl::PointXYZ>::Ptr obj_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        // if(pcl::io::loadPCDFile<pcl::PointXYZ>(strFilenameObjPcd,*obj_cloud)==-1)
        // {
        //     PCL_ERROR("Couldn't read file test_pcd.pcd \n");
        //     return ;
        // }
        while(pcl::io::loadPCDFile<pcl::PointXYZ>(strFilenameObjPcd,*obj_cloud)==-1)
        {
            ++pcd_idx;
            pcl::PointCloud<pcl::PointXYZ>::Ptr void_obj_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            imObjPcl.push_back(void_obj_cloud);
            strFilenameObjPcd = strPathObjPcd + "cloud_cluster_" + std::to_string(pcd_idx) + ".pcd";
            if(pcd_idx>100) return;
        }
        cv::Mat X_l(4, 1, CV_64F);
        cv::Mat X_c(4, 1, CV_64F);
        cv::Mat X(3, 1, CV_64F);
        cv::Mat X_(3, 1, CV_64F);
        double height = 0;
        for(size_t i=0;i<obj_cloud->points.size();++i){
            X_c.at<double>(0, 0) = obj_cloud->points[i].x;
            X_c.at<double>(1, 0) = obj_cloud->points[i].y;
            X_c.at<double>(2, 0) = obj_cloud->points[i].z;
            X_c.at<double>(3, 0) = 1.0;
            X_l = yaml_reader.params_.TvlL.inv()*X_c;
            height += X_l.at<double>(0, 0); 
        }
        height /= obj_cloud->points.size();
        if(height > 35) continue;

        imObjPcl.push_back(obj_cloud);
    }
}

// car, bus, train, motorcycle, bicucle, rider, person
void LoadMask(const string &strFilenamesMask, cv::Mat &imMask){
    cv::Mat mask = cv::imread(strFilenamesMask, CV_LOAD_IMAGE_UNCHANGED);
    int height = mask.size().height;
    int width = mask.size().width;
    cv::Mat M = cv::Mat::zeros(height, width, CV_8UC3);
    for(int r=0; r<height; ++r){
        for(int c=0; c<width; ++c){
            cv::Vec3b tmp = mask.at<cv::Vec3b>(r, c);
            for(int i=0, end=mask_type.size(); i<end; ++i){
                if(tmp==mask_type[i]){
                    imMask.at<int>(r, c) = i+1;
                    M.at<cv::Vec3b>(r, c) = mask_type[i];
                    break;
                }
            }
        }
    }
    // cv::imshow("mask", M);
    // cv::waitKey(0);
}
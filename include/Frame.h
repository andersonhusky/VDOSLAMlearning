/**
* This file is part of VDO-SLAM.
*
* Copyright (C) 2019-2020 Jun Zhang <jun doc zhang2 at anu dot edu doc au> (The Australian National University)
* For more information see <https://github.com/halajun/VDO_SLAM>
*
**/

#ifndef FRAME_H
#define FRAME_H

#include<vector>

#include "ORBextractor.h"

#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>                                        //PCL对各种格式的点的支持头文件
#include <pcl/io/pcd_io.h>                                              //PCL的PCD格式文件的输入输出头文件

namespace VDO_SLAM
{

using namespace std;

#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64

class Frame
{
public:
    Frame();

    // Copy constructor.
    Frame(const Frame &frame);

    // Constructor for RGB-D cameras.
    Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const cv::Mat &imFlow, const cv::Mat &maskSEM, const double &timeStamp, ORBextractor* extractor,
          cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth, const float &thDepthObj, const int &UseSampleFea);
    Frame(const cv::Mat &imGray, cv::Mat &imDepth, cv::Mat &imObjidx, const vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &imObjPcl, const cv::Mat &imFlow, const cv::Mat &maskSEM,
          const double &timeStamp, ORBextractor* extractor, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth, const float &thDepthObj, const int &UseSampleFea);

    // Extract ORB on the image. 0 for left image and 1 for right image.
    void ExtractORB(int flag, const cv::Mat &im);

    // Set the camera pose.
    void SetPose(cv::Mat Tcw);

    // Computes rotation, translation and camera center matrices from the camera pose.
    void UpdatePoseMatrices();

    // Returns the camera center.
    inline cv::Mat GetCameraCenter(){
        return mOw.clone();
    }

    // Returns inverse of rotation
    inline cv::Mat GetRotationInverse(){
        return mRwc.clone();
    }

    // Compute the cell of a keypoint (return false if outside the grid)
    bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);

    vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel=-1, const int maxLevel=-1) const;

    // Associate a "right" coordinate to a keypoint if there is valid depth in the depthmap.
    void ComputeStereoFromRGBD(const cv::Mat &imDepth);

    // Backprojects a keypoint (if stereo/depth info available) into 3D world coordinates.
    cv::Mat UnprojectStereo(const int &i);
    cv::Mat UnprojectStereoStat(const int &i, const bool &addnoise);
    cv::Mat UnprojectStereoObject(const int &i, const bool &addnoise);
    cv::Mat UnprojectStereoObjectCamera(const int &i, const bool &addnoise);
    cv::Mat UnprojectStereoObjectNoise(const int &i, const cv::Point2f of_error);
    cv::Mat ObtainFlowDepthObject(const int &i, const bool &addnoise);
    cv::Mat ObtainFlowDepthCamera(const int &i, const bool &addnoise);

    std::vector<cv::KeyPoint> SampleKeyPoints(const int &rows, const int &cols);
    std::vector<cv::KeyPoint> SampleKeyPointsFromPC(cv::Mat &imDepth, std::vector<float> &depthtmp, const pcl::PointCloud<pcl::PointXYZ>::Ptr &staPcl, const int &rows, const int &cols);

public:

    // Feature extractor. The right is used only in the stereo case.
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;

    // Frame timestamp.
    double mTimeStamp;

    // Calibration matrix and OpenCV distortion parameters.
    cv::Mat mK;
    static float fx;
    static float fy;
    static float cx;
    static float cy;
    static float invfx;
    static float invfy;
    cv::Mat mDistCoef;

    // Stereo baseline multiplied by fx.
    float mbf;

    // Stereo baseline in meters.
    float mb;

    // Threshold close/far points.
    float mThDepth;
    float mThDepthObj;

    // Number of KeyPoints.
    int N;

    // Vector of keypoints (original for visualization) and undistorted (actually used by the system).
    // In the stereo case, mvKeysUn is redundant as images must be rectified.
    // In the RGB-D case, RGB images can be distorted.
    std::vector<cv::KeyPoint> mvKeys, mvKeysRight;
    std::vector<cv::KeyPoint> mvKeysUn;

    // Corresponding stereo coordinate and depth for each keypoint.
    // "Monocular" keypoints have a negative value.
    std::vector<float> mvuRight;
    std::vector<float> mvDepth;

    // ORB descriptor, each row associated to a keypoint.
    cv::Mat mDescriptors, mDescriptorsRight;


    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    // Number of KeyPoints.
    int N_s;

    // Store keypoints and descriptors
    std::vector<cv::KeyPoint> mvStatKeys, mvStatKeysRight;                          // 静态特征点，在tracking中存储的是光流的tmp结果
    // Store dense key points and depths on objects
    std::vector<cv::KeyPoint> mvObjKeys;                                                                // 当前帧obj特征点
    std::vector<float> mvObjDepth;                                                                              // 当前帧obj上点深度
    std::vector<cv::Mat> mvObj3DPoint;
    std::vector<cv::KeyPoint> mvObjCorres;                                                              // obj当前帧特征点对应光流结果的下一帧位置
    std::vector<cv::Point2f> mvObjFlowGT, mvObjFlowNext;                            // FlowNext：当前帧特征点的光流向量
    std::vector<int> vSemObjLabel;                                                                              // obj特征点的语义标签，来自mask结果

    // save the object status (false for outlier, true for inlier)  # added 10 Jan 2020 #
    std::vector<bool> bObjStat;



    // depth for each keypoint
    std::vector<float> mvStatDepth;

    // Store the Label Index for each features: -1(outlier/unknown), 0(static), 1...n(object label).
    std::vector<int> vObjLabel;

    // Store the 3D flow vector and the 2D re-projection error vector
    // 3d场景流
    std::vector<cv::Point3f> vFlow_3d;
    std::vector<cv::Point2f> vFlow_2d;

    // Store the motion of objects
    std::vector<cv::Mat> vObjMod;
    std::vector<cv::Mat> vObjPosePre;
    std::vector<cv::Point2f> vSpeed;                        // 对应object的运动速度
    std::vector<int> nModLabel;                                 // 当前帧的label序号，即这是第几个object
    std::vector<int> nSemPosition;                          // object的语义标签
    std::vector<int> vObjBoxID; // bounding box for each object
    std::vector<std::vector<int> > vnObjID; // object id in current frame
    std::vector<std::vector<int> > vnObjInlierID; // object id in current frame
    std::vector<cv::Mat> vObjCentre3D; // 3D in the world coordinate frame
    std::vector<cv::Mat> vObjCentre2D; // 2D in the image plane

    // for initializing motion
    cv::Mat mInitModel;

    std::vector<cv::KeyPoint> mvCorres;                                                                     // 当前帧静态点对应光流结果的下一帧位置
    std::vector<cv::Point2f> mvFlow,mvFlowNext;                                                 // flownext：当前帧特征点对应的光流向量
    // std::vector<int> vCorSta; // the status of correspondence, -1 (outliers) 1 (has correspondence)

    // temporal saved
    std::vector<cv::KeyPoint> mvStatKeysTmp;                                                        // 存储Frame初始化时构建的静态ORB特征点
    std::vector<float> mvStatDepthTmp;                                                                      // 存储Frame初始化时静态特征点对应的深度值
    std::vector<cv::Mat> mvStat3DPointTmp;                                                          // 通过特征点和深度构造的3D静态点
    std::vector<int> vSemLabelTmp;
    std::vector<int> vObjLabel_gtTmp;
    int N_s_tmp;                                                                        // 一开始生成的特征点的数目

    // inlier ID generated in this frame  (new added Nov 14 2019)
    std::vector<int> nStaInlierID;
    std::vector<int> nDynInlierID;


    // **************** Ground Truth *********************

    std::vector<cv::Mat> vObjPose_gt;                                               // 当前帧出现的obj对应的位姿真值
    std::vector<int> nSemPosi_gt;                                                       // 当前帧出现的obj序号
    std::vector<cv::Mat> vObjMod_gt;
    std::vector<float> vObjSpeed_gt;

    cv::Mat mTcw_gt;
    std::vector<int> vObjLabel_gt; // 0(background), 1...n(instance label)

    // ***************************************************

    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    // Flag to identify outlier associations.
    std::vector<bool> mvbOutlier;

    // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.
    static float mfGridElementWidthInv;
    static float mfGridElementHeightInv;
    std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

    // Camera pose.
    cv::Mat mTcw;

    // Current and Next Frame id.
    static long unsigned int nNextId;
    long unsigned int mnId;

    // Scale pyramid info.
    int mnScaleLevels;
    float mfScaleFactor;
    float mfLogScaleFactor;
    vector<float> mvScaleFactors;
    vector<float> mvInvScaleFactors;
    vector<float> mvLevelSigma2;
    vector<float> mvInvLevelSigma2;

    // Undistorted Image Bounds (computed once).
    static float mnMinX;
    static float mnMaxX;
    static float mnMinY;
    static float mnMaxY;

    static bool mbInitialComputations;


private:

    // Undistort keypoints given OpenCV distortion parameters.
    // Only for the RGB-D case. Stereo must be already rectified!
    // (called in the constructor).
    void UndistortKeyPoints();

    // Computes image bounds for the undistorted image (called in the constructor).
    void ComputeImageBounds(const cv::Mat &imLeft);

    // Assign keypoints to the grid for speed up feature matching (called in the constructor).
    void AssignFeaturesToGrid();

    // Rotation, translation and camera center
    cv::Mat mRcw;
    cv::Mat mtcw;
    cv::Mat mRwc;
    cv::Mat mOw; // mtwc
};

}// namespace VDO_SLAM

#endif // FRAME_H

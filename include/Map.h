/**
* This file is part of VDO-SLAM.
*
* Copyright (C) 2019-2020 Jun Zhang <jun doc zhang2 at anu dot edu doc au> (The Australian National University)
* For more information see <https://github.com/halajun/VDO_SLAM>
*
**/

#ifndef MAP_H
#define MAP_H

#include<opencv2/core/core.hpp>
#include "KeyP.h"

#include <set>

class KeyP;
namespace VDO_SLAM
{

class Map
{
public:
    Map();

    // ==========================================================
    // ============= output for evaluating results ==============

    // ==========================================================
    // ==========================================================

    // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    // <<<<<<<<<<<<<<<<<<<< output for graph structure >>>>>>>>>>>>>>>>>>>

    // static features and depths detected in image plane. (k*n)
    std::vector<std::vector<cv::KeyPoint> > vpFeatSta;                                                                      //vec< 静态特征点vec（一帧）>
    std::vector<std::vector<float> > vfDepSta;                                                                                        // 深度信息
    std::vector<std::vector<cv::Mat> > vp3DPointSta;                                                                        // 恢复的3D点
    // index of temporal matching. (k-1)*n
    std::vector<std::vector<int> > vnAssoSta;                                                                                       // 保存静态匹配是否为内点，一个vec保存一帧信息
    // feature tracklets: pair.first = frameID; pair.second = featureID;
    std::vector<std::vector<std::pair<int, int> > > TrackletSta;


    // dynamic feature correspondences and depths detected in image plane. k*n
    std::vector<std::vector<cv::KeyPoint> > vpFeatDyn;                                                                      // vec< 动态特征点vec（一帧）>
    std::vector<std::vector<float> > vfDepDyn;                                                                                         // 深度信息
    std::vector<std::vector<cv::Mat> > vp3DPointDyn;                                                                        // 恢复的3D点
    // index of temporal matching. (k-1)*n
    std::vector<std::vector<int> > vnAssoDyn;
    // label indicating which object the feature (3D point) belongs to. (k-1)*n
    std::vector<std::vector<int> > vnFeatLabel;
    // feature tracklets: pair.first = frameID; pair.second = featureID;
    std::vector<std::vector<std::pair<int, int> > > TrackletDyn;
    std::vector<int> nObjID;


    // absolute camera pose of each frame, starting from 1st frame. (k*1)
    std::vector<cv::Mat> vmCameraPose;                                                                                                  // 计算的相机位姿结果
    std::vector<cv::Mat> vmCameraPose_RF;                                                                                           // 相机位姿优化后的结果refine result
    std::vector<cv::Mat> vmCameraPose_GT;                                                                                           // 相机位姿真值ground truth result
    // rigid motion of camera and dynamic points. (k-1)*m
    std::vector<std::vector<cv::Mat> > vmRigidCentre;  // ground truth object center
    std::vector<std::vector<cv::Mat> > vmRigidMotion;
    std::vector<std::vector<cv::Mat> > vmObjPosePre; // for new metric 26 Feb 2020
    std::vector<std::vector<cv::Mat> > vmRigidMotion_RF;  // refine result
    std::vector<std::vector<cv::Mat> > vmRigidMotion_GT;  // ground truth result
    std::vector<std::vector<float> > vfAllSpeed_GT; // camera and object speeds
    // rigid motion label in each frame (k-1)*m
    // 0 stands for camera motion; 1,...,l stands for rigid motions.
    std::vector<std::vector<int> > vnRMLabel; // tracking label
    std::vector<std::vector<int> > vnSMLabel; // semantic label
    std::vector<std::vector<int> > vnSMLabelGT;
    // object status (10 Jan 2020)
    std::vector<std::vector<bool> > vbObjStat;
    // object tracking times (10 Jan 2020)
    std::vector<std::vector<int> > vnObjTraTime;
    std::vector<int> nObjTraCount;
    std::vector<int> nObjTraCountGT;
    std::vector<int> nObjTraSemLab;

    // time analysis
    std::vector<float> fLBA_time;
    // (0) frame updating (1) camera estimation (2) object tracking (3) object estimation (4) map updating;
    std::vector<std::vector<float> > vfAll_time;

    std::vector<std::vector<KeyP*>> vpFeatStatic;
    std::vector<std::vector<Obj*>> vpObjDyna;


    // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

protected:

    long unsigned int mnMaxKFid;

    // Index related to a big change in the map
    int mnBigChangeIdx;

};

} //namespace VDO_SLAM

#endif // MAP_H

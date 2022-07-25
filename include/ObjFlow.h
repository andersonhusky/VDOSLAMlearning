/******************
* @Description：Object轨迹类
* @Author：hongfeng
* @Date：2022/07/25
******************/

#ifndef OBJECT_H
#define OBJECT_H

#include "Frame.h"
#include "KeyP.h"
#include "Obj.h"

class KeyP;
class Frame;
class Obj;

class ObjFlow{
    public:
        ObjFlow();

    public:
        bool mvObjStat;
        std::vector<cv::Mat> mvObjMod, mvObjMod_gt;                 // object相互运动
        std::vector<cv::Mat> mvObjPose_gt;                                          // object世界坐标系下的位姿
        std::vector<cv::Mat> mvObjSpeed, mvObjSpeed_gt;         // 换算的速度，换算公式为论文中对应式子25
        std::vector<int> mvObjBoxID;                                                       // 在各个帧对应的2D 框真值序号，序号与iniFrameIdx对齐
        std::vector<cv::Mat> mvObjCentre3D;                                     // 在各个帧对应的质心坐标
        std::vector<std::vector<KeyP*> > mvKeyPoints;                  // 在各个帧的特征点
        std::vector<std::vector<KeyP*> > mvKeyPointsInlier;        // 在各个帧的特征点内点

        int iniFrameIdx;
};

#endif
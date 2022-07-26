/******************
* @Description：Object类
            一个obj对象与一个帧对应，表示帧内的一个物体，obj由objflow连接成全局的运动
* @Author：hongfeng
* @Date：2022/07/21
******************/

#ifndef OBJECT_H
#define OBJECT_H

#include "Frame.h"
#include "KeyP.h"

class KeyP;
class Frame;

class Obj{
    public:
        Obj();
        Obj(const int label);

    public:
        std::vector<KeyP*> mvObjKeyPs;
        std::vector<cv::Point3f> mvFlow_3d;

        cv::Mat mvObjMod, mvObjMod_gt;
        cv::Mat mvObjPose_gt;
        cv::Mat mvObjSpeed, mvObjSpeed_gt;
        cv::Mat mvObjCentre3D;

        int mvLabel;
        int mvIdx;

        Frame* mvFrame;

    private:
};

#endif
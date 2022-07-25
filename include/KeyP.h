/******************
* @Description：Object类
* @Author：hongfeng
* @Date：2022/07/22
******************/

#ifndef KEYP_H
#define KEYP_H

#include "Frame.h"
#include "Obj.h"

class Frame;
class Obj;

class KeyP{
    public:
        KeyP();

        KeyP(const cv::KeyPoint& pt, const cv::KeyPoint corre, const cv::Point2f& flow, const float& depth, const int& label, const bool& dyna);

    public:
        bool dyna;

        // common part
        cv::KeyPoint mvKey;
        cv::KeyPoint mvCorre;
        cv::Point2f mvFlow;
        float mvDepth;
        cv::Mat mv3DPoint;

        // dyna part
        int mvObjLabel;
        int mvObjIdx;

        Frame* mvFrame;
        Obj* mvObj;
    private:

};

#endif
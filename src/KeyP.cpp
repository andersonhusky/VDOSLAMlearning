/******************
* @Description：Object类
* @Author：hongfeng
* @Date：2022/07/22
******************/

#include "KeyP.h"

KeyP::KeyP():
mvKey(cv::KeyPoint()), mvCorre(cv::KeyPoint()), mvFlow(cv::Point2f()), mvDepth(-1), mvObjLabel(-1), dyna(false)
{
}

KeyP::KeyP(const cv::KeyPoint& pt, const cv::KeyPoint corre, const cv::Point2f& flow, const float& depth, const int& label, const bool& dyna):
mvKey(pt), mvCorre(corre), mvFlow(flow), mvDepth(depth), mvObjLabel(label), dyna(dyna)
{
}
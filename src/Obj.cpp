/******************
* @Description：Object类
* @Author：hongfeng
* @Date：2022/07/21
******************/

#include "Obj.h"

Obj::Obj():
mvLabel(0), mvIdx(0), 
mvObjKeyPs(std::vector<KeyP*>()), mvObjStat(true), mvObjMod(cv::Mat())
{

}

Obj::Obj(const int label):
mvLabel(label), mvIdx(0), 
mvObjKeyPs(std::vector<KeyP*>()), mvObjStat(true)
{

}

Obj::Obj(const int label, const int idx):
mvLabel(label), mvIdx(idx), 
mvObjKeyPs(std::vector<KeyP*>()), mvObjStat(true)
{

}
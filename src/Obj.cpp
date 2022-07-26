/******************
* @Description：Object类
* @Author：hongfeng
* @Date：2022/07/21
******************/

#include "Obj.h"

Obj::Obj():
mvLabel(0), mvIdx(0), mvObjKeyPs(std::vector<KeyP*>())
{

}

Obj::Obj(const int label):
mvLabel(label), mvIdx(0), mvObjKeyPs(std::vector<KeyP*>())
{

}
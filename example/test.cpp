#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

Point2f getCrossPoint(Vec4i LineA, Vec4i LineB)
{
    double ka, kb;
    ka = (double)(LineA[3] - LineA[1]) / (double)(LineA[2] - LineA[0]); //求出LineA斜率
    kb = (double)(LineB[3] - LineB[1]) / (double)(LineB[2] - LineB[0]); //求出LineB斜率

    Point2f crossPoint;
    crossPoint.x = (ka*LineA[0] - LineA[1] - kb*LineB[0] + LineB[1]) / (ka - kb);
    crossPoint.y = (ka*kb*(LineA[0] - LineB[0]) + ka*LineB[1] - kb*LineA[1]) / (ka - kb);
    return crossPoint;
}

int main(){
    Vec4i LineA{0, 0, 1, 1};
    Vec4i LineB{0, 1, 1, 0};
    Point2f result = getCrossPoint(LineA, LineB);
    cout << result << endl;
}
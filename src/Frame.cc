/**
* This file is part of VDO-SLAM.
*
* Copyright (C) 2019-2020 Jun Zhang <jun doc zhang2 at anu dot edu doc au> (The Australian National University)
* For more information see <https://github.com/halajun/VDO_SLAM>
*
**/

#include <opencv2/xfeatures2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <opencv2/flann.hpp>

#include "Frame.h"
#include "Converter.h"
#include <thread>
#include<time.h>
#include<chrono>

namespace VDO_SLAM
{

using namespace std;

long unsigned int Frame::nNextId=0;
bool Frame::mbInitialComputations=true;
float Frame::cx, Frame::cy, Frame::fx, Frame::fy, Frame::invfx, Frame::invfy;
float Frame::mnMinX, Frame::mnMinY, Frame::mnMaxX, Frame::mnMaxY;
float Frame::mfGridElementWidthInv, Frame::mfGridElementHeightInv;

Frame::Frame()
{}

//Copy Constructor
Frame::Frame(const Frame &frame)
    :mpORBextractorLeft(frame.mpORBextractorLeft), mpORBextractorRight(frame.mpORBextractorRight),
     mTimeStamp(frame.mTimeStamp), mK(frame.mK.clone()), mDistCoef(frame.mDistCoef.clone()),
     mbf(frame.mbf), mb(frame.mb), mThDepth(frame.mThDepth), mThDepthObj(frame.mThDepthObj), N(frame.N), mvKeys(frame.mvKeys),
     mvKeysRight(frame.mvKeysRight), mvKeysUn(frame.mvKeysUn),  mvuRight(frame.mvuRight), mvDepth(frame.mvDepth),
     mDescriptors(frame.mDescriptors.clone()), mDescriptorsRight(frame.mDescriptorsRight.clone()),
     mvbOutlier(frame.mvbOutlier), mnId(frame.mnId), mnScaleLevels(frame.mnScaleLevels),
     mfScaleFactor(frame.mfScaleFactor), mfLogScaleFactor(frame.mfLogScaleFactor),
     mvScaleFactors(frame.mvScaleFactors), mvInvScaleFactors(frame.mvInvScaleFactors),
     mvLevelSigma2(frame.mvLevelSigma2), mvInvLevelSigma2(frame.mvInvLevelSigma2),
     // new added
     mTcw_gt(frame.mTcw_gt), vObjPose_gt(frame.vObjPose_gt), nSemPosi_gt(frame.nSemPosi_gt),
     vObjLabel(frame.vObjLabel), nModLabel(frame.nModLabel), nSemPosition(frame.nSemPosition),
     bObjStat(frame.bObjStat), vObjMod(frame.vObjMod), mvCorres(frame.mvCorres), mvObjCorres(frame.mvObjCorres),
     mvFlowNext(frame.mvFlowNext), mvObjFlowNext(frame.mvObjFlowNext),
     // change
     mvObjects(frame.mvObjects), mvStatKeyPs(frame.mvStatKeyPs)
{
    for(int i=0;i<FRAME_GRID_COLS;i++)
        for(int j=0; j<FRAME_GRID_ROWS; j++)
            mGrid[i][j]=frame.mGrid[i][j];

    if(!frame.mTcw.empty())
        SetPose(frame.mTcw);
}


Frame::Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const cv::Mat &imFlow, const cv::Mat &maskSEM,
    const double &timeStamp, ORBextractor* extractor, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth, const float &thDepthObj, const int &UseSampleFea)
    :mpORBextractorLeft(extractor),mpORBextractorRight(static_cast<ORBextractor*>(NULL)),
     mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth), mThDepthObj(thDepthObj)
{

    cout << "Start Constructing Frame......" << endl;

    // Frame ID
    mnId=nNextId++;

    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();


    // ------------------------------------------------------------------------------------------
    // ++++++++++++++++++++++++++++ New added for background features +++++++++++++++++++++++++++
    // ------------------------------------------------------------------------------------------

    // clock_t s_1, e_1;
    // double fea_det_time;
    // s_1 = clock();
    // ORB extraction
    ExtractORB(0,imGray);
    // e_1 = clock();
    // fea_det_time = (double)(e_1-s_1)/CLOCKS_PER_SEC*1000;
    // cout << "feature detection time: " << fea_det_time << endl;

    N = mvKeys.size();

    if(mvKeys.empty())
        return;

    if (UseSampleFea==0)
    {
        // // // Option I: ~~~~~~~ use detected features ~~~~~~~~~~ // // //

        for (int i = 0; i < mvKeys.size(); ++i)
        {
            int x = mvKeys[i].pt.x;
            int y = mvKeys[i].pt.y;

            if (maskSEM.at<int>(y,x)!=0)  // new added in Jun 13 2019
                continue;

            if (imDepth.at<float>(y,x)>mThDepth || imDepth.at<float>(y,x)<=0)  // new added in Aug 21 2019
                continue;

            float flow_xe = imFlow.at<cv::Vec2f>(y,x)[0];
            float flow_ye = imFlow.at<cv::Vec2f>(y,x)[1];


            if(flow_xe!=0 && flow_ye!=0)
            {
                if(mvKeys[i].pt.x+flow_xe < imGray.cols && mvKeys[i].pt.y+flow_ye < imGray.rows && mvKeys[i].pt.x < imGray.cols && mvKeys[i].pt.y < imGray.rows)
                {
                    mvStatKeysTmp.push_back(mvKeys[i]);
                    mvCorres.push_back(cv::KeyPoint(mvKeys[i].pt.x+flow_xe,mvKeys[i].pt.y+flow_ye,0,0,0,mvKeys[i].octave,-1));
                    mvFlowNext.push_back(cv::Point2f(flow_xe,flow_ye));
                }
            }
        }
    }
    else
    {
        // // // Option II: ~~~~~~~ use sampled features ~~~~~~~~~~ // // //

        clock_t s_1, e_1;
        double fea_det_time;
        s_1 = clock();
        std::vector<cv::KeyPoint> mvKeysSamp = SampleKeyPoints(imGray.rows, imGray.cols);
        e_1 = clock();
        fea_det_time = (double)(e_1-s_1)/CLOCKS_PER_SEC*1000;
        // std::cout << "feature detection time: " << fea_det_time << std::endl;

        for (int i = 0; i < mvKeysSamp.size(); ++i)
        {
            int x = mvKeysSamp[i].pt.x;
            int y = mvKeysSamp[i].pt.y;

            if (maskSEM.at<int>(y,x)!=0)  // new added in Jun 13 2019
                continue;

            if (imDepth.at<float>(y,x)>mThDepth || imDepth.at<float>(y,x)<=0)  // new added in Aug 21 2019
                continue;

            float flow_xe = imFlow.at<cv::Vec2f>(y,x)[0];
            float flow_ye = imFlow.at<cv::Vec2f>(y,x)[1];


            if(flow_xe!=0 && flow_ye!=0)
            {
                if(mvKeysSamp[i].pt.x+flow_xe < imGray.cols && mvKeysSamp[i].pt.y+flow_ye < imGray.rows && mvKeysSamp[i].pt.x+flow_xe>0 && mvKeysSamp[i].pt.y+flow_ye>0)
                {
                    mvStatKeysTmp.push_back(mvKeysSamp[i]);
                    mvCorres.push_back(cv::KeyPoint(mvKeysSamp[i].pt.x+flow_xe,mvKeysSamp[i].pt.y+flow_ye,0,0,0,mvKeysSamp[i].octave,-1));
                    mvFlowNext.push_back(cv::Point2f(flow_xe,flow_ye));

                }
            }
        }
    }

    // ---------------------------------------------------------------------------------------
    // ---------------------------------------------------------------------------------------

    // cv::Mat img_show;
    // cv::drawKeypoints(imGray, mvKeysSamp, img_show, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
    // cv::imshow("KeyPoints on Background", img_show);
    // cv::waitKey(0);

    N_s_tmp = mvCorres.size();
    cout << "number of random sample points: " << mvCorres.size() << endl;

    // assign the depth value to each keypoint
    mvStatDepthTmp = vector<float>(N_s_tmp,-1);
    for(int i=0; i<N_s_tmp; i++)
    {
        const cv::KeyPoint &kp = mvStatKeysTmp[i];

        const float &v = kp.pt.y;
        const float &u = kp.pt.x;

        float d = imDepth.at<float>(v,u); // be careful with the order  !!!

        if(d>0)
            mvStatDepthTmp[i] = d;
    }

    // ---------------------------------------------------------------------------------------
    // ++++++++++++++++++++++++++++ New added for dense object features ++++++++++++++++++++++
    // ---------------------------------------------------------------------------------------

    // semi-dense features on objects
    int step = 4; // 3
    for (int i = 0; i < imGray.rows; i=i+step)
    {
        for (int j = 0; j < imGray.cols; j=j+step)
        {
            // check ground truth motion mask
            if (maskSEM.at<int>(i,j)!=0 && imDepth.at<float>(i,j)<mThDepthObj && imDepth.at<float>(i,j)>0)
            {
                // get flow
                const float flow_x = imFlow.at<cv::Vec2f>(i,j)[0];
                const float flow_y = imFlow.at<cv::Vec2f>(i,j)[1];

                if(j+flow_x < imGray.cols && j+flow_x > 0 && i+flow_y < imGray.rows && i+flow_y > 0)
                {
                    // save correspondences
                    mvObjFlowNext.push_back(cv::Point2f(flow_x,flow_y));
                    mvObjCorres.push_back(cv::KeyPoint(j+flow_x,i+flow_y,0,0,0,-1));
                    // save pixel location
                    mvObjKeys.push_back(cv::KeyPoint(j,i,0,0,0,-1));
                    // save depth
                    mvObjDepth.push_back(imDepth.at<float>(i,j));
                    // save label
                    vSemObjLabel.push_back(maskSEM.at<int>(i,j));
                }
            }
        }
    }

    // ---------------------------------------------------------------------------------------
    // ---------------------------------------------------------------------------------------

    UndistortKeyPoints();

    ComputeStereoFromRGBD(imDepth);

    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        ComputeImageBounds(imGray);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    mb = mbf/fx;

    AssignFeaturesToGrid();

    cout << "Constructing Frame, Done!" << endl;
}

Frame::Frame(const cv::Mat &imGray, cv::Mat &imDepth, cv::Mat &imObjidx, const vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &imObjPcl,const cv::Mat &imFlow, const cv::Mat &maskSEM,
    const double &timeStamp, ORBextractor* extractor, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth, const float &thDepthObj, const int &UseSampleFea)
    :mpORBextractorLeft(extractor),mpORBextractorRight(static_cast<ORBextractor*>(NULL)),
     mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth), mThDepthObj(thDepthObj)
{

    cout << "Start Constructing Frame......" << endl;

    // Frame ID
    mnId=nNextId++;

    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();


    // ------------------------------------------------------------------------------------------
    // ++++++++++++++++++++++++++++ New added for background features +++++++++++++++++++++++++++
    // ------------------------------------------------------------------------------------------

    // clock_t s_1, e_1;
    // double fea_det_time;
    // s_1 = clock();
    // ORB extraction
    ExtractORB(0,imGray);
    // e_1 = clock();
    // fea_det_time = (double)(e_1-s_1)/CLOCKS_PER_SEC*1000;
    // cout << "feature detection time: " << fea_det_time << endl;

    N = mvKeys.size();

    if(mvKeys.empty())
        return;

    // static part
    {
        clock_t s_1, e_1;
        double fea_det_time;
        s_1 = clock();
        std::vector<float> depthtmp;
        std::vector<cv::KeyPoint> mvKeysSamp = SampleKeyPointsFromPC(imDepth, depthtmp, imObjPcl[0], imGray.rows, imGray.cols);
        e_1 = clock();
        fea_det_time = (double)(e_1-s_1)/CLOCKS_PER_SEC*1000;
        // std::cout << "feature detection time: " << fea_det_time << std::endl;

        mvStatDepthTmp.reserve(depthtmp.size());
        for (int i = 0; i < mvKeysSamp.size(); ++i)
        {
            int x = mvKeysSamp[i].pt.x;
            int y = mvKeysSamp[i].pt.y;

            if (maskSEM.at<int>(y,x)!=0)  // new added in Jun 13 2019
                continue;

            // change here
            // if (imDepth.at<float>(y,x)>mThDepth || imDepth.at<float>(y,x)<=0)  // new added in Aug 21 2019
            //     continue;

            float flow_xe = imFlow.at<cv::Vec2f>(y,x)[0];
            float flow_ye = imFlow.at<cv::Vec2f>(y,x)[1];

            if(flow_xe!=0 && flow_ye!=0)
            {
                if(mvKeysSamp[i].pt.x+flow_xe < imGray.cols && mvKeysSamp[i].pt.y+flow_ye < imGray.rows && mvKeysSamp[i].pt.x+flow_xe>0 && mvKeysSamp[i].pt.y+flow_ye>0)
                {
                    mvStatKeysTmp.push_back(mvKeysSamp[i]);
                    mvCorres.push_back(cv::KeyPoint(mvKeysSamp[i].pt.x+flow_xe,mvKeysSamp[i].pt.y+flow_ye,0,0,0,mvKeysSamp[i].octave,-1));
                    mvFlowNext.push_back(cv::Point2f(flow_xe,flow_ye));
                    if(depthtmp[i]>0){
                        mvStatDepthTmp.push_back(depthtmp[i]);
                    }
                }
            }
        }
        // cv::Mat img_show;
        // cv::drawKeypoints(imGray, mvStatKeysTmp, img_show, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
        // cv::imshow("KeyPoints on Background", img_show);
        // cv::waitKey(0);
    }
    // ---------------------------------------------------------------------------------------
    // ---------------------------------------------------------------------------------------

    N_s_tmp = mvCorres.size();
    // cout << "number of random sample points: " << mvCorres.size() << endl;

    // ---------------------------------------------------------------------------------------
    // ++++++++++++++++++++++++++++ New added for dense object features ++++++++++++++++++++++
    // ---------------------------------------------------------------------------------------

    // semi-dense features on objects
    {
        for(int k=1; k<imObjPcl.size(); ++k){
            pcl::PointCloud<pcl::PointXYZ>::Ptr objPcl = imObjPcl[k];
            cv::Mat X(3, 1, CV_32F);
            cv::Mat X_(3, 1, CV_32F);
            for(size_t i=0;  i<objPcl->points.size(); ++i){
                X.at<float>(0, 0) = float(objPcl->points[i].x);
                X.at<float>(1, 0) = float(objPcl->points[i].y);
                X.at<float>(2, 0) = float(objPcl->points[i].z);
                X_ = mK*X;

                float d = X_.at<float>(2, 0);
                if(d<=0)
                    continue;
                cv::Point pt;
                pt.x = X_.at<float>(0, 0) / d;
                pt.y = X_.at<float>(1, 0) / d;

                int x = int(pt.x); 
                int y = int(pt.y);
                if(x<0 || x>=imGray.cols || y<0 || y>=imGray.rows)
                    continue;
                const float flow_x = imFlow.at<cv::Vec2f>(y, x)[0];
                const float flow_y = imFlow.at<cv::Vec2f>(y, x)[1];

                if(x+flow_x < imGray.cols && x+flow_x > 0 && y+flow_y < imGray.rows && y+flow_y > 0)
                {
                    mvObjFlowNext.push_back(cv::Point2f(flow_x,flow_y));
                    mvObjCorres.push_back(cv::KeyPoint(x+flow_x,y+flow_y,0,0,0,-1));
                    mvObjKeys.push_back(cv::KeyPoint(x,y,0,0,0,-1));
                    mvObjDepth.push_back(d);
                    vSemObjLabel.push_back(k);
                    imDepth.at<float>(y,x) = d;
                    imObjidx.at<ushort>(y,x) = k;
                }
            }
        }
    }

    // ---------------------------------------------------------------------------------------
    // ---------------------------------------------------------------------------------------
    // 特征点去畸变
    UndistortKeyPoints();

    // point：这里先不改，看后面ORB特征点干了啥
    ComputeStereoFromRGBD(imDepth);

    // This is done only for the first Frame (or after a change in the calibration)
    // 第一帧才运行
    if(mbInitialComputations)
    {
        // 划定图像边界
        ComputeImageBounds(imGray);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    mb = mbf/fx;

    // 特征点分配到网格中
    AssignFeaturesToGrid();

    cout << "Constructing Frame, Done!" << endl;
}

void Frame::AssignFeaturesToGrid()
{
    int nReserve = 0.5f*N/(FRAME_GRID_COLS*FRAME_GRID_ROWS);
    for(unsigned int i=0; i<FRAME_GRID_COLS;i++)
        for (unsigned int j=0; j<FRAME_GRID_ROWS;j++)
            mGrid[i][j].reserve(nReserve);

    for(int i=0;i<N;i++)
    {
        const cv::KeyPoint &kp = mvKeysUn[i];

        int nGridPosX, nGridPosY;
        if(PosInGrid(kp,nGridPosX,nGridPosY))
            mGrid[nGridPosX][nGridPosY].push_back(i);
    }
}

void Frame::ExtractORB(int flag, const cv::Mat &im)
{
    if(flag==0){
        (*mpORBextractorLeft)(im,cv::Mat(),mvKeys,mDescriptors);
        // cv::Ptr<cv::Feature2D> f2d = cv::xfeatures2d::SURF::create(400);
        // cv::Ptr<cv::Feature2D> f2d = cv::xfeatures2d::SIFT::create();
        // f2d->compute(im, mvKeys, mSift);
    }
    else{
        (*mpORBextractorRight)(im,cv::Mat(),mvKeysRight,mDescriptorsRight);
        // cv::Ptr<cv::Feature2D> f2d = cv::xfeatures2d::SURF::create(400);
        // cv::Ptr<cv::Feature2D> f2d = cv::xfeatures2d::SIFT::create();
        // f2d->compute(im, mvKeysRight, mSiftRight);
    }
}

void Frame::SetPose(cv::Mat Tcw)
{
    mTcw = Tcw.clone();
    // mTcw.at<float>(0,0)=Tcw.at<float>(0,0);mTcw.at<float>(0,1)=Tcw.at<float>(0,1);mTcw.at<float>(0,2)=Tcw.at<float>(0,2);mTcw.at<float>(0,3)=Tcw.at<float>(0,3);
    // mTcw.at<float>(1,0)=Tcw.at<float>(1,0);mTcw.at<float>(1,1)=Tcw.at<float>(1,1);mTcw.at<float>(1,2)=Tcw.at<float>(1,2);mTcw.at<float>(1,3)=Tcw.at<float>(1,3);
    // mTcw.at<float>(2,0)=Tcw.at<float>(2,0);mTcw.at<float>(2,1)=Tcw.at<float>(2,1);mTcw.at<float>(2,2)=Tcw.at<float>(2,2);mTcw.at<float>(2,3)=Tcw.at<float>(2,3);
    // mTcw.at<float>(3,0)=Tcw.at<float>(3,0);mTcw.at<float>(3,1)=Tcw.at<float>(3,1);mTcw.at<float>(3,2)=Tcw.at<float>(3,2);mTcw.at<float>(2,3)=Tcw.at<float>(3,3);
    UpdatePoseMatrices();
}

void Frame::UpdatePoseMatrices()
{
    mRcw = mTcw.rowRange(0,3).colRange(0,3);
    mRwc = mRcw.t();
    mtcw = mTcw.rowRange(0,3).col(3);
    mOw = -mRcw.t()*mtcw;
}

vector<size_t> Frame::GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel, const int maxLevel) const
{
    vector<size_t> vIndices;
    vIndices.reserve(N);

    const int nMinCellX = max(0,(int)floor((x-mnMinX-r)*mfGridElementWidthInv));
    if(nMinCellX>=FRAME_GRID_COLS)
        return vIndices;

    const int nMaxCellX = min((int)FRAME_GRID_COLS-1,(int)ceil((x-mnMinX+r)*mfGridElementWidthInv));
    if(nMaxCellX<0)
        return vIndices;

    const int nMinCellY = max(0,(int)floor((y-mnMinY-r)*mfGridElementHeightInv));
    if(nMinCellY>=FRAME_GRID_ROWS)
        return vIndices;

    const int nMaxCellY = min((int)FRAME_GRID_ROWS-1,(int)ceil((y-mnMinY+r)*mfGridElementHeightInv));
    if(nMaxCellY<0)
        return vIndices;

    const bool bCheckLevels = (minLevel>0) || (maxLevel>=0);

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            const vector<size_t> vCell = mGrid[ix][iy];
            if(vCell.empty())
                continue;

            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
                if(bCheckLevels)
                {
                    if(kpUn.octave<minLevel)
                        continue;
                    if(maxLevel>=0)
                        if(kpUn.octave>maxLevel)
                            continue;
                }

                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;

                if(fabs(distx)<r && fabs(disty)<r)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}

bool Frame::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY)
{
    posX = round((kp.pt.x-mnMinX)*mfGridElementWidthInv);
    posY = round((kp.pt.y-mnMinY)*mfGridElementHeightInv);

    //Keypoint's coordinates are undistorted, which could cause to go out of the image
    if(posX<0 || posX>=FRAME_GRID_COLS || posY<0 || posY>=FRAME_GRID_ROWS)
        return false;

    return true;
}

void Frame::UndistortKeyPoints()
{
    if(mDistCoef.at<float>(0)==0.0)
    {
        mvKeysUn=mvKeys;
        return;
    }

    // Fill matrix with points
    cv::Mat mat(N,2,CV_32F);
    for(int i=0; i<N; i++)
    {
        mat.at<float>(i,0)=mvKeys[i].pt.x;
        mat.at<float>(i,1)=mvKeys[i].pt.y;
    }

    // Undistort points
    mat=mat.reshape(2);
    cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
    mat=mat.reshape(1);

    // Fill undistorted keypoint vector
    mvKeysUn.resize(N);
    for(int i=0; i<N; i++)
    {
        cv::KeyPoint kp = mvKeys[i];
        kp.pt.x=mat.at<float>(i,0);
        kp.pt.y=mat.at<float>(i,1);
        mvKeysUn[i]=kp;
    }
}

void Frame::ComputeImageBounds(const cv::Mat &imLeft)
{
    if(mDistCoef.at<float>(0)!=0.0)
    {
        cv::Mat mat(4,2,CV_32F);
        mat.at<float>(0,0)=0.0; mat.at<float>(0,1)=0.0;
        mat.at<float>(1,0)=imLeft.cols; mat.at<float>(1,1)=0.0;
        mat.at<float>(2,0)=0.0; mat.at<float>(2,1)=imLeft.rows;
        mat.at<float>(3,0)=imLeft.cols; mat.at<float>(3,1)=imLeft.rows;

        // Undistort corners
        mat=mat.reshape(2);
        cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
        mat=mat.reshape(1);

        mnMinX = min(mat.at<float>(0,0),mat.at<float>(2,0));
        mnMaxX = max(mat.at<float>(1,0),mat.at<float>(3,0));
        mnMinY = min(mat.at<float>(0,1),mat.at<float>(1,1));
        mnMaxY = max(mat.at<float>(2,1),mat.at<float>(3,1));

    }
    else
    {
        mnMinX = 0.0f;
        mnMaxX = imLeft.cols;
        mnMinY = 0.0f;
        mnMaxY = imLeft.rows;
    }
}


void Frame::ComputeStereoFromRGBD(const cv::Mat &imDepth)
{
    mvuRight = vector<float>(N,-1);
    mvDepth = vector<float>(N,-1);

    for(int i=0; i<N; i++)
    {
        const cv::KeyPoint &kp = mvKeys[i];
        const cv::KeyPoint &kpU = mvKeysUn[i];

        const float &v = kp.pt.y;
        const float &u = kp.pt.x;

        const float d = imDepth.at<float>(v,u); // be careful with the order

        if(d>0)
        {
            mvDepth[i] = d;
            mvuRight[i] = kpU.pt.x-mbf/d;
        }
    }
}

cv::Mat Frame::UnprojectStereo(const int &i)
{
    const float z = mvDepth[i];
    if(z>0)
    {
        const float u = mvKeysUn[i].pt.x;
        const float v = mvKeysUn[i].pt.y;
        // cout << "xyz: " << u << " " << v << " " << z << endl;
        const float x = (u-cx)*z*invfx;
        const float y = (v-cy)*z*invfy;
        cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);
        return mRwc*x3Dc+mOw;
    }
    else
        return cv::Mat();
}

//! 恢复静态点的深度
//!
//! \param i 需要恢复的点序号
//! \param addnoise 是否添加噪声标志位
//! \return 点在世界坐标系下的坐标
// change here
cv::Mat Frame::UnprojectStereoStat(const int &i, const bool &addnoise)
{
    float z = mvStatDepth[i];

    // used for adding noise
    cv::RNG rng((unsigned)time(NULL));

    if(addnoise){
        z = z + rng.gaussian(z*z/(725*0.5)*0.15);  // sigma = z*0.01
        // z = z + 0.0;
    }

    if(z>0)
    {
        const float u = mvStatKeys[i].pt.x;
        const float v = mvStatKeys[i].pt.y;
        // cout << "xyz: " << u << " " << v << " " << z << endl;
        const float x = (u-cx)*z*invfx;
        const float y = (v-cy)*z*invfy;
        cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);

        // using ground truth
        const cv::Mat Rlw = mTcw.rowRange(0,3).colRange(0,3);
        const cv::Mat Rwl = Rlw.t();
        const cv::Mat tlw = mTcw.rowRange(0,3).col(3);
        const cv::Mat twl = -Rlw.t()*tlw;

        return Rwl*x3Dc+twl;
        // return mRwc*x3Dc+mOw;
    }
    else
    {
        cout << "found a depth value < 0 ..." << endl;
        return cv::Mat();
    }
}

//! 恢复object上点的深度
//!
//! \param i 需要恢复的点序号
//! \param addnoise 是否添加噪声标志位
//! \return 点在世界坐标系下的坐标
cv::Mat Frame::UnprojectStereoObject(const int &i, const bool &addnoise)
{
    float z = mvObjDepth[i];

    // used for adding noise
    cv::RNG rng((unsigned)time(NULL));

    if(addnoise){
        float noise = rng.gaussian(z*z/(725*0.5)*0.15);
        z = z + noise;  // sigma = z*0.01
        // z = z + 0.0;
        // cout << "noise: " << noise << endl;
    }

    if(z>0)
    {
        const float u = mvObjKeys[i].pt.x;
        const float v = mvObjKeys[i].pt.y;
        const float x = (u-cx)*z*invfx;
        const float y = (v-cy)*z*invfy;
        cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);

        // using ground truth
        const cv::Mat Rlw = mTcw.rowRange(0,3).colRange(0,3);
        const cv::Mat Rwl = Rlw.t();
        const cv::Mat tlw = mTcw.rowRange(0,3).col(3);
        const cv::Mat twl = -Rlw.t()*tlw;

        return Rwl*x3Dc+twl;
    }
    else{
        cout << "found a depth value < 0 ..." << endl;
        return cv::Mat();
    }
}

cv::Mat Frame::UnprojectStereoObjectCamera(const int &i, const bool &addnoise)
{
    float z = mvObjDepth[i];
    // cout << "depth check: " << z << endl;

    // used for adding noise
    cv::RNG rng((unsigned)time(NULL));

    if(addnoise){
        // sigma = z*0.01
        z = z + rng.gaussian(z*z/(725*0.5)*0.15);  // sigma = z*0.01
        // z = z + 0.0;
    }

    if(z>0)
    {
        const float u = mvObjKeys[i].pt.x;
        const float v = mvObjKeys[i].pt.y;
        const float x = (u-cx)*z*invfx;
        const float y = (v-cy)*z*invfy;
        cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);

        return x3Dc;
    }
    else{
        cout << "found a depth value < 0 ..." << endl;
        return cv::Mat();
    }
}

cv::Mat Frame::UnprojectStereoObjectNoise(const int &i, const cv::Point2f of_error)
{
    float z = mvObjDepth[i];

    // if(addnoise){
    //     z = z + rng.gaussian(z*0.01);  // sigma = z*0.01
    // }

    if(z>0)
    {
        const float u = mvObjKeys[i].pt.x + of_error.x;
        const float v = mvObjKeys[i].pt.y + of_error.y;
        const float x = (u-cx)*z*invfx;
        const float y = (v-cy)*z*invfy;
        cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);

        // using ground truth
        const cv::Mat Rlw = mTcw.rowRange(0,3).colRange(0,3);
        const cv::Mat Rwl = Rlw.t();
        const cv::Mat tlw = mTcw.rowRange(0,3).col(3);
        const cv::Mat twl = -Rlw.t()*tlw;

        return Rwl*x3Dc+twl;
    }
    else{
        cout << "found a depth value < 0 ..." << endl;
        return cv::Mat();
    }
}

cv::Mat Frame::ObtainFlowDepthObject(const int &i, const bool &addnoise)
{
    float z = mvObjDepth[i];

    // used for adding noise
    cv::RNG rng((unsigned)time(NULL));

    if(addnoise){
        z = z + rng.gaussian(z*z/(725*0.5)*0.15);  // sigma = z*0.01 or z*z/(725*0.5)*0.12
        // z = z + 0.0;
    }

    if(z>0)
    {
        const float flow_u = mvObjFlowNext[i].x;
        const float flow_v = mvObjFlowNext[i].y;

        cv::Mat x3Dc = (cv::Mat_<float>(3,1) << flow_u, flow_v, z);

        return x3Dc;
    }
    else{
        cout << "found a depth value < 0 ..." << endl;
        return cv::Mat();
    }
}

cv::Mat Frame::ObtainFlowDepthCamera(const int &i, const bool &addnoise)
{
    float z = mvStatDepth[i];

    // used for adding noise
    cv::RNG rng((unsigned)time(NULL));

    if(addnoise){
        z = z + rng.gaussian(z*z/(725*0.5)*0.15);  // sigma = z*0.01 or z*z/(725*0.5)*0.12
        // z = z + 0.0;
    }

    if(z>0)
    {
        const float flow_u = mvFlowNext[i].x;
        const float flow_v = mvFlowNext[i].y;

        cv::Mat x3Dc = (cv::Mat_<float>(3,1) << flow_u, flow_v, z);

        return x3Dc;
    }
    else
    {
        cout << "found a depth value < 0 ..." << endl;
        return cv::Mat();
    }
}

std::vector<cv::KeyPoint> Frame::SampleKeyPoints(const int &rows, const int &cols)
{
    cv::RNG rng((unsigned)time(NULL));
    int N = 3000;
    int n_div = 20;
    std::vector<cv::KeyPoint> KeySave;
    std::vector<std::vector<cv::KeyPoint> >  KeyinGrid(n_div*n_div);

    // (1) construct grid
    int x_step = cols/n_div, y_step = rows/n_div;

    // main loop
    int key_num = 0;
    while (key_num<N)
    {
        for (int i = 0; i < n_div; ++i)
        {
            for (int j = 0; j < n_div; ++j)
            {
                const float x = rng.uniform(i*x_step,(i+1)*x_step);
                const float y = rng.uniform(j*y_step,(j+1)*y_step);

                if (x>=cols || y>=rows || x<=0 || y<=0)
                    continue;

                // // check if this key point is already been used
                // float min_dist = 1000;
                // bool used = false;
                // for (int k = 0; k < KeyinGrid[].size(); ++k)
                // {
                //     float cur_dist = std::sqrt( (KeyinGrid[].pt.x-x)*(KeyinGrid[].pt.x-x) + (KeyinGrid[].pt.y-y)*(KeyinGrid[].pt.y-y) );
                //     if (cur_dist<min_dist)
                //         min_dist = cur_dist;
                //     if (min_dist<5.0)
                //     {
                //         used = true;
                //         break;
                //     }
                // }

                // if (used)
                //     continue;

                cv::KeyPoint Key_tmp = cv::KeyPoint(x,y,0,0,0,-1);
                KeyinGrid[i*n_div+j].push_back(Key_tmp);
                key_num = key_num + 1;
                if (key_num>=N)
                    break;
            }
            if (key_num>=N)
                break;
        }
    }

    // cout << "key_num: " << key_num << endl;

    // save points
    for (int i = 0; i < KeyinGrid.size(); ++i)
    {
        for (int j = 0; j < KeyinGrid[i].size(); ++j)
        {
            KeySave.push_back(KeyinGrid[i][j]);
        }
    }

    return KeySave;

}

std::vector<cv::KeyPoint> Frame::SampleKeyPointsFromPC(cv::Mat &imDepth, std::vector<float> &depthtmp, const pcl::PointCloud<pcl::PointXYZ>::Ptr &staPcl, const int &rows, const int &cols)
{
    int N = 3000;
    std::vector<cv::KeyPoint> KeySave;
    KeySave.reserve(N);
    depthtmp.reserve(N);

    cv::RNG rng((unsigned)time(NULL));
    int n_div = 20;
    std::vector<std::vector<pair<cv::KeyPoint, float>>> KeyinGrid(n_div*n_div);
    int x_step = cols/n_div, y_step = rows/n_div;

    cv::Mat X(3, 1, CV_32F);
    cv::Mat X_(3, 1, CV_32F);
    for(size_t i=0;i<staPcl->points.size();++i){
        X.at<float>(0, 0) = float(staPcl->points[i].x);
        X.at<float>(1, 0) = float(staPcl->points[i].y);
        X.at<float>(2, 0) = float(staPcl->points[i].z);
        X_ = mK*X;

        float d = X_.at<float>(2, 0);
        if(d<=0)
            continue;
        cv::Point pt;
        pt.x = X_.at<float>(0, 0) / d;
        pt.y = X_.at<float>(1, 0) / d;

        int x = int(pt.x); int y = int(pt.y);
        if(x<0 || x>=cols || y<0 || y>=rows)
            continue;

        int idx_i = y/y_step, idx_j = x/x_step;
        idx_i = idx_i>=n_div? n_div-1: idx_i;
        idx_j = idx_j>=n_div? n_div-1: idx_j;
        cv::KeyPoint Key_tmp = cv::KeyPoint(x,y,0,0,0,-1);
        KeyinGrid[idx_i*n_div+idx_j].push_back(make_pair(Key_tmp, d));
        imDepth.at<float>(y,x) = d;
    }

    int key_num = 0;
    while(key_num<N){
        int delta_num=0;
        for (int i = 0; i < KeyinGrid.size(); ++i)
        {
            int len = KeyinGrid[i].size();
            if(len==0) continue;
            int j = rng.uniform(0, len);
            KeySave.push_back(KeyinGrid[i][j].first);
            depthtmp.push_back(KeyinGrid[i][j].second);
            swap(KeyinGrid[i][j], KeyinGrid[i][len-1]);
            KeyinGrid[i].pop_back();

            ++key_num;
            ++delta_num;
            if (key_num>=N) break;
        }
        if(delta_num==0)    break;
    }
    return KeySave;
}

Frame::Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const cv::Mat &imFlow, const cv::Mat &maskSEM,
    const double &timeStamp, ORBextractor* extractor, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth, const float &thDepthObj, const int &UseSampleFea, const bool &change)
    :mpORBextractorLeft(extractor),mpORBextractorRight(static_cast<ORBextractor*>(NULL)),
     mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth), mThDepthObj(thDepthObj)
{

    cout << "Start Constructing Frame......" << endl;

    // Frame ID
    mnId=nNextId++;

    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();


    // ------------------------------------------------------------------------------------------
    // ++++++++++++++++++++++++++++ New added for background features +++++++++++++++++++++++++++
    // ------------------------------------------------------------------------------------------

    // clock_t s_1, e_1;
    // double fea_det_time;
    // s_1 = clock();
    // ORB extraction
    ExtractORB(0,imGray);
    // e_1 = clock();
    // fea_det_time = (double)(e_1-s_1)/CLOCKS_PER_SEC*1000;
    // cout << "feature detection time: " << fea_det_time << endl;

    N = mvKeys.size();

    if(mvKeys.empty())
        return;

    if (UseSampleFea==0)
    {
        // // // Option I: ~~~~~~~ use detected features ~~~~~~~~~~ // // //

        for (int i = 0; i < mvKeys.size(); ++i)
        {
            int x = mvKeys[i].pt.x;
            int y = mvKeys[i].pt.y;

            if (maskSEM.at<int>(y,x)!=0)  // new added in Jun 13 2019
                continue;

            if (imDepth.at<float>(y,x)>mThDepth || imDepth.at<float>(y,x)<=0)  // new added in Aug 21 2019
                continue;

            float flow_xe = imFlow.at<cv::Vec2f>(y,x)[0];
            float flow_ye = imFlow.at<cv::Vec2f>(y,x)[1];


            if(flow_xe!=0 && flow_ye!=0)
            {
                if(mvKeys[i].pt.x+flow_xe < imGray.cols && mvKeys[i].pt.y+flow_ye < imGray.rows && mvKeys[i].pt.x < imGray.cols && mvKeys[i].pt.y < imGray.rows)
                {
                    cv::KeyPoint point = mvKeys[i];
                    cv::KeyPoint corre(x+flow_xe, y+flow_ye,0,0,0,mvKeys[i].octave,-1);
                    cv::Point2f flow(flow_xe,flow_ye);
                    KeyP* key_point = new KeyP(point, corre, flow, -1, -1, false);
                    mvStatKeyPs.push_back(key_point);
                }
            }
        }
    }
    else
    {
        // // // Option II: ~~~~~~~ use sampled features ~~~~~~~~~~ // // //

        clock_t s_1, e_1;
        double fea_det_time;
        s_1 = clock();
        std::vector<cv::KeyPoint> mvKeysSamp = SampleKeyPoints(imGray.rows, imGray.cols);
        e_1 = clock();
        fea_det_time = (double)(e_1-s_1)/CLOCKS_PER_SEC*1000;
        // std::cout << "feature detection time: " << fea_det_time << std::endl;

        for (int i = 0; i < mvKeysSamp.size(); ++i)
        {
            int x = mvKeysSamp[i].pt.x;
            int y = mvKeysSamp[i].pt.y;

            if (maskSEM.at<int>(y,x)!=0)  // new added in Jun 13 2019
                continue;

            if (imDepth.at<float>(y,x)>mThDepth || imDepth.at<float>(y,x)<=0)  // new added in Aug 21 2019
                continue;

            float flow_xe = imFlow.at<cv::Vec2f>(y,x)[0];
            float flow_ye = imFlow.at<cv::Vec2f>(y,x)[1];


            if(flow_xe!=0 && flow_ye!=0)
            {
                if(mvKeysSamp[i].pt.x+flow_xe < imGray.cols && mvKeysSamp[i].pt.y+flow_ye < imGray.rows && mvKeysSamp[i].pt.x+flow_xe>0 && mvKeysSamp[i].pt.y+flow_ye>0)
                {
                    cv::KeyPoint point = mvKeysSamp[i];
                    cv::KeyPoint corre(mvKeysSamp[i].pt.x+flow_xe, mvKeysSamp[i].pt.y+flow_ye,0,0,0,mvKeysSamp[i].octave,-1);
                    cv::Point2f flow(flow_xe,flow_ye);
                    KeyP* key_point = new KeyP(point, corre, flow, -1, 0, false);
                    mvStatKeyPs.push_back(key_point);
                }
            }
        }
    }

    // ---------------------------------------------------------------------------------------
    // ---------------------------------------------------------------------------------------

    // cv::Mat img_show;
    // cv::drawKeypoints(imGray, mvKeysSamp, img_show, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
    // cv::imshow("KeyPoints on Background", img_show);
    // cv::waitKey(0);

    N_s_tmp = mvStatKeyPs.size();
    cout << "number of random sample points: " << N_s_tmp << endl;

    // assign the depth value to each keypoint
    for(int i=0; i<N_s_tmp; ++i)
    {
        const cv::KeyPoint &kp = mvStatKeyPs[i]->mvKey;

        const float &v = kp.pt.y;
        const float &u = kp.pt.x;

        float d = imDepth.at<float>(v,u); // be careful with the order  !!!

        if(d>0)
            mvStatKeyPs[i]->mvDepth = d;
    }

    // ---------------------------------------------------------------------------------------
    // ++++++++++++++++++++++++++++ New added for dense object features ++++++++++++++++++++++
    // ---------------------------------------------------------------------------------------

    // semi-dense features on objects
    int step = 4; // 3
    for (int i = 0; i < imGray.rows; i=i+step)
    {
        for (int j = 0; j < imGray.cols; j=j+step)
        {
            // check ground truth motion mask
            if (maskSEM.at<int>(i,j)!=0 && imDepth.at<float>(i,j)<mThDepthObj && imDepth.at<float>(i,j)>0)
            {
                // get flow
                const float flow_x = imFlow.at<cv::Vec2f>(i,j)[0];
                const float flow_y = imFlow.at<cv::Vec2f>(i,j)[1];

                if(j+flow_x < imGray.cols && j+flow_x > 0 && i+flow_y < imGray.rows && i+flow_y > 0)
                {
                    // save correspondences
                    cv::KeyPoint point(j,i,0,0,0,-1);
                    cv::KeyPoint corre(j+flow_x,i+flow_y,0,0,0,-1);
                    cv::Point2f flow(flow_x,flow_y);
                    float depth = imDepth.at<float>(i,j);
                    int label = maskSEM.at<int>(i,j);
                    KeyP* key_point = new KeyP(point, corre, flow, depth, label, true);

                    int idx=0;
                    while(idx < mvObjects.size()){
                        if(mvObjects[idx]->mvLabel==label){
                            break;
                        }
                        ++idx;
                    }
                    // create a new obj
                    if(idx==mvObjects.size()){
                        Obj* new_obj = new Obj(label);
                        new_obj->mvObjKeyPs.push_back(key_point);
                        mvObjects.push_back(new_obj);
                    }
                    else{
                        mvObjects[idx]->mvObjKeyPs.push_back(key_point);
                    }

                    // mvObjFlowNext.push_back(cv::Point2f(flow_x,flow_y));
                    // mvObjCorres.push_back(cv::KeyPoint(j+flow_x,i+flow_y,0,0,0,-1));
                    // // save pixel location
                    // mvObjKeys.push_back(cv::KeyPoint(j,i,0,0,0,-1));
                    // // save depth
                    // mvObjDepth.push_back(imDepth.at<float>(i,j));
                    // // save label
                    // vSemObjLabel.push_back(maskSEM.at<int>(i,j));
                }
            }
        }
    }

    // ---------------------------------------------------------------------------------------
    // ---------------------------------------------------------------------------------------

    UndistortKeyPoints();

    ComputeStereoFromRGBD(imDepth);

    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        ComputeImageBounds(imGray);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    mb = mbf/fx;

    AssignFeaturesToGrid();

    cout << "Constructing Frame, Done!" << endl;
}

cv::Mat Frame::UnprojectStereoStat_change(const int &i, const bool &addnoise)
{
    float z = mvStatKeyPs[i]->mvDepth;

    // used for adding noise
    cv::RNG rng((unsigned)time(NULL));

    if(addnoise){
        z = z + rng.gaussian(z*z/(725*0.5)*0.15);  // sigma = z*0.01
        // z = z + 0.0;
    }

    if(z>0)
    {
        const float u =mvStatKeyPs[i]->mvKey.pt.x;
        const float v = mvStatKeyPs[i]->mvKey.pt.y;
        // cout << "xyz: " << u << " " << v << " " << z << endl;
        const float x = (u-cx)*z*invfx;
        const float y = (v-cy)*z*invfy;
        cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);

        // using ground truth
        const cv::Mat Rlw = mTcw.rowRange(0,3).colRange(0,3);
        const cv::Mat Rwl = Rlw.t();
        const cv::Mat tlw = mTcw.rowRange(0,3).col(3);
        const cv::Mat twl = -Rlw.t()*tlw;

        return Rwl*x3Dc+twl;
        // return mRwc*x3Dc+mOw;
    }
    else
    {
        cout << "found a depth value < 0 ..." << endl;
        return cv::Mat();
    }
}

cv::Mat Frame::ObtainFlowDepthCamera_change(const int &i, const bool &addnoise)
{
    float z = mvStatKeyPs[i]->mvDepth;

    // used for adding noise
    cv::RNG rng((unsigned)time(NULL));

    if(addnoise){
        z = z + rng.gaussian(z*z/(725*0.5)*0.15);  // sigma = z*0.01 or z*z/(725*0.5)*0.12
        // z = z + 0.0;
    }

    if(z>0)
    {
        const float flow_u = mvStatKeyPs[i]->mvFlow.x;
        const float flow_v = mvStatKeyPs[i]->mvFlow.y;

        cv::Mat x3Dc = (cv::Mat_<float>(3,1) << flow_u, flow_v, z);

        return x3Dc;
    }
    else
    {
        cout << "found a depth value < 0 ..." << endl;
        return cv::Mat();
    }
}

cv::Mat Frame::UnprojectStereoObject_change(const cv::Point2d &match, const bool &addnoise)
{
    int i = match.x;
    int j = match.y;
    float z = mvObjects[i]->mvObjKeyPs[j]->mvDepth;

    // used for adding noise
    cv::RNG rng((unsigned)time(NULL));

    if(addnoise){
        float noise = rng.gaussian(z*z/(725*0.5)*0.15);
        z = z + noise;  // sigma = z*0.01
        // z = z + 0.0;
        // cout << "noise: " << noise << endl;
    }

    if(z>0)
    {
        const float u = mvObjects[i]->mvObjKeyPs[j]->mvKey.pt.x;
        const float v = mvObjects[i]->mvObjKeyPs[j]->mvKey.pt.y;
        const float x = (u-cx)*z*invfx;
        const float y = (v-cy)*z*invfy;
        cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);

        // using ground truth
        const cv::Mat Rlw = mTcw.rowRange(0,3).colRange(0,3);
        const cv::Mat Rwl = Rlw.t();
        const cv::Mat tlw = mTcw.rowRange(0,3).col(3);
        const cv::Mat twl = -Rlw.t()*tlw;

        return Rwl*x3Dc+twl;
    }
    else{
        cout << "found a depth value < 0 ..." << endl;
        return cv::Mat();
    }
}

cv::Mat Frame::UnprojectStereoObjectMatch_change(const int &i, const bool &addnoise)
{
    float z = mvObjKeyPsMatch[i]->mvDepth;

    // used for adding noise
    cv::RNG rng((unsigned)time(NULL));

    if(addnoise){
        float noise = rng.gaussian(z*z/(725*0.5)*0.15);
        z = z + noise;  // sigma = z*0.01
        // z = z + 0.0;
        // cout << "noise: " << noise << endl;
    }

    if(z>0)
    {
        const float u = mvObjKeyPsMatch[i]->mvKey.pt.x;
        const float v = mvObjKeyPsMatch[i]->mvKey.pt.y;
        const float x = (u-cx)*z*invfx;
        const float y = (v-cy)*z*invfy;
        cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);

        // using ground truth
        const cv::Mat Rlw = mTcw.rowRange(0,3).colRange(0,3);
        const cv::Mat Rwl = Rlw.t();
        const cv::Mat tlw = mTcw.rowRange(0,3).col(3);
        const cv::Mat twl = -Rlw.t()*tlw;

        return Rwl*x3Dc+twl;
    }
    else{
        cout << "found a depth value < 0 ..." << endl;
        return cv::Mat();
    }
}

int Frame::GetObjKeyPsNum(){
    int result=0;
    for(auto obj: mvObjects){
        result += obj->mvObjKeyPs.size();
    }
    return result;
}

} //namespace VDO_SLAM

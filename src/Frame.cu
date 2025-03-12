/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include "Frame.h"
#include "Tracking.h"
#include "G2oTypes.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "ORBextractor.h"
#include "Converter.h"
#include "ORBmatcher.h"
#include "GeometricCamera.h"
#include "YoloDetector.h"
#include <thread>
#include <include/CameraModels/Pinhole.h>
#include <include/CameraModels/KannalaBrandt8.h>


std::vector<cv::Point2f> Prepoint, Curpoint;
cv::Mat imGrayPre,mask;
std::vector<uchar> state;
std::vector<float> Err;
std::vector<double> epiErr, EpiErr;
std::vector<uchar> status;

namespace ORB_SLAM3
{

long unsigned int Frame::nNextId=0;
bool Frame::mbInitialComputations=true;
float Frame::cx, Frame::cy, Frame::fx, Frame::fy, Frame::invfx, Frame::invfy;
float Frame::mnMinX, Frame::mnMinY, Frame::mnMaxX, Frame::mnMaxY;
float Frame::mfGridElementWidthInv, Frame::mfGridElementHeightInv;

//For stereo fisheye matching
cv::BFMatcher Frame::BFmatcher = cv::BFMatcher(cv::NORM_HAMMING);

Frame::Frame(): mpcpi(NULL), mpImuPreintegrated(NULL), mpPrevFrame(NULL), mpImuPreintegratedFrame(NULL), mpReferenceKF(static_cast<KeyFrame*>(NULL)), mbIsSet(false), mbImuPreintegrated(false), mbHasPose(false), mbHasVelocity(false)
{
#ifdef REGISTER_TIMES
    mTimeStereoMatch = 0;
    mTimeORB_Ext = 0;
#endif
}


//Copy Constructor
Frame::Frame(const Frame &frame)
    :mpcpi(frame.mpcpi),mpORBvocabulary(frame.mpORBvocabulary), mpORBextractorLeft(frame.mpORBextractorLeft), mpORBextractorRight(frame.mpORBextractorRight),
     mTimeStamp(frame.mTimeStamp), mK(frame.mK.clone()), mK_(Converter::toMatrix3f(frame.mK)), mDistCoef(frame.mDistCoef.clone()),
     mbf(frame.mbf), mb(frame.mb), mThDepth(frame.mThDepth), N(frame.N), mvKeys(frame.mvKeys),
     mvKeysRight(frame.mvKeysRight), mvKeysUn(frame.mvKeysUn), mvuRight(frame.mvuRight),
     mvDepth(frame.mvDepth), mBowVec(frame.mBowVec), mFeatVec(frame.mFeatVec),
     mDescriptors(frame.mDescriptors.clone()), mDescriptorsRight(frame.mDescriptorsRight.clone()),
     mvpMapPoints(frame.mvpMapPoints), mvbOutlier(frame.mvbOutlier), mImuCalib(frame.mImuCalib), mnCloseMPs(frame.mnCloseMPs),
     mpImuPreintegrated(frame.mpImuPreintegrated), mpImuPreintegratedFrame(frame.mpImuPreintegratedFrame), mImuBias(frame.mImuBias),
     mnId(frame.mnId), mpReferenceKF(frame.mpReferenceKF), mnScaleLevels(frame.mnScaleLevels),
     mfScaleFactor(frame.mfScaleFactor), mfLogScaleFactor(frame.mfLogScaleFactor),
     mvScaleFactors(frame.mvScaleFactors), mvInvScaleFactors(frame.mvInvScaleFactors), mNameFile(frame.mNameFile), mnDataset(frame.mnDataset),
     mvLevelSigma2(frame.mvLevelSigma2), mvInvLevelSigma2(frame.mvInvLevelSigma2), mpPrevFrame(frame.mpPrevFrame), mpLastKeyFrame(frame.mpLastKeyFrame),
     mbIsSet(frame.mbIsSet), mbImuPreintegrated(frame.mbImuPreintegrated), mpMutexImu(frame.mpMutexImu),
     mpCamera(frame.mpCamera), mpCamera2(frame.mpCamera2), Nleft(frame.Nleft), Nright(frame.Nright),
     monoLeft(frame.monoLeft), monoRight(frame.monoRight), mvLeftToRightMatch(frame.mvLeftToRightMatch),
     mvRightToLeftMatch(frame.mvRightToLeftMatch), mvStereo3Dpoints(frame.mvStereo3Dpoints),
     mTlr(frame.mTlr), mRlr(frame.mRlr), mtlr(frame.mtlr), mTrl(frame.mTrl),
     mTcw(frame.mTcw), mbHasPose(false), mbHasVelocity(false)
{
    for(int i=0;i<FRAME_GRID_COLS;i++)
        for(int j=0; j<FRAME_GRID_ROWS; j++){
            mGrid[i][j]=frame.mGrid[i][j];
            if(frame.Nleft > 0){
                mGridRight[i][j] = frame.mGridRight[i][j];
            }
        }

    if(frame.mbHasPose)
        SetPose(frame.GetPose());

    if(frame.HasVelocity())
    {
        SetVelocity(frame.GetVelocity());
    }

    mmProjectPoints = frame.mmProjectPoints;
    mmMatchedInImage = frame.mmMatchedInImage;

    // copy image
    assert(!frame.mImColor.empty());
    if(!frame.mImColor.empty())
        mImColor = frame.mImColor.clone();
    if(!frame.mImDepth.empty())
        mImDepth = frame.mImDepth.clone();

#ifdef REGISTER_TIMES
    mTimeStereoMatch = frame.mTimeStereoMatch;
    mTimeORB_Ext = frame.mTimeORB_Ext;
#endif
}


Frame::Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor* extractorLeft, ORBextractor* extractorRight, ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth, GeometricCamera* pCamera, Frame* pPrevF, const IMU::Calib &ImuCalib)
    :mpcpi(NULL), mpORBvocabulary(voc),mpORBextractorLeft(extractorLeft),mpORBextractorRight(extractorRight), mTimeStamp(timeStamp), mK(K.clone()), mK_(Converter::toMatrix3f(K)), mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth),
     mImuCalib(ImuCalib), mpImuPreintegrated(NULL), mpPrevFrame(pPrevF),mpImuPreintegratedFrame(NULL), mpReferenceKF(static_cast<KeyFrame*>(NULL)), mbIsSet(false), mbImuPreintegrated(false),
     mpCamera(pCamera) ,mpCamera2(nullptr), mbHasPose(false), mbHasVelocity(false)
{
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

    // ORB extraction
#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_StartExtORB = std::chrono::steady_clock::now();
#endif
    thread threadLeft(&Frame::ExtractORB,this,0,imLeft,0,0);
    thread threadRight(&Frame::ExtractORB,this,1,imRight,0,0);
    threadLeft.join();
    threadRight.join();
#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_EndExtORB = std::chrono::steady_clock::now();

    mTimeORB_Ext = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndExtORB - time_StartExtORB).count();
#endif

    N = mvKeys.size();
    if(mvKeys.empty())
        return;

    UndistortKeyPoints();

#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_StartStereoMatches = std::chrono::steady_clock::now();
#endif
    ComputeStereoMatches();
#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_EndStereoMatches = std::chrono::steady_clock::now();

    mTimeStereoMatch = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndStereoMatches - time_StartStereoMatches).count();
#endif

    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));
    mvbOutlier = vector<bool>(N,false);
    mmProjectPoints.clear();
    mmMatchedInImage.clear();


    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        ComputeImageBounds(imLeft);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/(mnMaxY-mnMinY);



        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    mb = mbf/fx;

    if(pPrevF)
    {
        if(pPrevF->HasVelocity())
            SetVelocity(pPrevF->GetVelocity());
    }
    else
    {
        mVw.setZero();
    }

    mpMutexImu = new std::mutex();

    //Set no stereo fisheye information
    Nleft = -1;
    Nright = -1;
    mvLeftToRightMatch = vector<int>(0);
    mvRightToLeftMatch = vector<int>(0);
    mvStereo3Dpoints = vector<Eigen::Vector3f>(0);
    monoLeft = -1;
    monoRight = -1;

    AssignFeaturesToGrid();
}

Frame::Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth, GeometricCamera* pCamera,Frame* pPrevF, const IMU::Calib &ImuCalib)
    :mpcpi(NULL),mpORBvocabulary(voc),mpORBextractorLeft(extractor),mpORBextractorRight(static_cast<ORBextractor*>(NULL)),
     mTimeStamp(timeStamp), mK(K.clone()), mK_(Converter::toMatrix3f(K)),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth),
     mImuCalib(ImuCalib), mpImuPreintegrated(NULL), mpPrevFrame(pPrevF), mpImuPreintegratedFrame(NULL), mpReferenceKF(static_cast<KeyFrame*>(NULL)), mbIsSet(false), mbImuPreintegrated(false),
     mpCamera(pCamera),mpCamera2(nullptr), mbHasPose(false), mbHasVelocity(false)
{
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

    // ORB extraction
#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_StartExtORB = std::chrono::steady_clock::now();
#endif
    ExtractORB(0,imGray,0,0);
#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_EndExtORB = std::chrono::steady_clock::now();

    mTimeORB_Ext = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndExtORB - time_StartExtORB).count();
#endif


    N = mvKeys.size();

    if(mvKeys.empty())
        return;

    UndistortKeyPoints();

    ComputeStereoFromRGBD(imDepth);

    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));

    mmProjectPoints.clear();
    mmMatchedInImage.clear();

    mvbOutlier = vector<bool>(N,false);

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

    if(pPrevF){
        if(pPrevF->HasVelocity())
            SetVelocity(pPrevF->GetVelocity());
    }
    else{
        mVw.setZero();
    }

    mpMutexImu = new std::mutex();

    //Set no stereo fisheye information
    Nleft = -1;
    Nright = -1;
    mvLeftToRightMatch = vector<int>(0);
    mvRightToLeftMatch = vector<int>(0);
    mvStereo3Dpoints = vector<Eigen::Vector3f>(0);
    monoLeft = -1;
    monoRight = -1;

    AssignFeaturesToGrid();
}

Frame::Frame(Tracking* pTracker, const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth, GeometricCamera* pCamera,Frame* pPrevF, const IMU::Calib &ImuCalib)
    :mpcpi(NULL),mpORBvocabulary(voc),mpORBextractorLeft(extractor),mpORBextractorRight(static_cast<ORBextractor*>(NULL)),
    mTimeStamp(timeStamp), mK(K.clone()), mK_(Converter::toMatrix3f(K)),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth),
    mImuCalib(ImuCalib), mpImuPreintegrated(NULL), mpPrevFrame(pPrevF), mpImuPreintegratedFrame(NULL), mpReferenceKF(static_cast<KeyFrame*>(NULL)), mbIsSet(false), mbImuPreintegrated(false),
    mpCamera(pCamera),mpCamera2(nullptr), mbHasPose(false), mbHasVelocity(false), mpTracker(pTracker)
{
//    std::chrono::steady_clock::time_point t4 = std::chrono::steady_clock::now();
    // Frame ID
    // Step 1 帧的ID 自增
    mnId=nNextId++;
//    ImGray = imGray.clone();
    // Scale Level Info
    // Step 2 计算图像金字塔的参数
    // 获取图像金字塔的层数
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    // 获得层与层之间的缩放比
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    // 计算上面缩放比的对数
    mfLogScaleFactor = log(mfScaleFactor);
    // 获取每层图像的缩放因子
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    // 同样获取每层图像缩放因子的倒数
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    // 高斯模糊的时候，使用的方差
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    // 获取sigma^2的倒数
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // ORB extraction
#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_StartExtORB = std::chrono::steady_clock::now();
#endif
#ifdef TIMES
    std::chrono::steady_clock::time_point extract1 = std::chrono::steady_clock::now();
#endif
    ExtractORB(0,imGray,0,0);
#ifdef TIMES
    std::chrono::steady_clock::time_point extract2 = std::chrono::steady_clock::now();
    cout << "extract time:" << std::chrono::duration_cast<std::chrono::duration<double> >(extract2 - extract1).count() <<endl;
#endif
#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_EndExtORB = std::chrono::steady_clock::now();

mTimeORB_Ext = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndExtORB - time_StartExtORB).count();
#endif

#ifdef REMOVE_DYNAMIC_OBJECT
    cv::Mat imGrayT = imGray;
    if(imGrayPre.data)
    {
        EpiloarWithFundMatrix(imGrayPre,imGray);
        std::swap(imGrayPre,imGrayT);
    }
    else
        std::swap(imGrayPre,imGrayT);

    while(!mpTracker->isNewSegmentImgArrived())
    {
        usleep(1);
    }

    if(!mvKeys.empty())
    {
//        cout<<mpTracker->mbNewSegImgFlag<<endl;
#ifdef TIMES
        std::chrono::steady_clock::time_point RemoveMovingPointsWithEpiAndYOLO1 = std::chrono::steady_clock::now();
#endif
        RemoveMovingPointsWithEpiAndYOLO(epiErr,imGray,imDepth);
#ifdef TIMES
        std::chrono::steady_clock::time_point RemoveMovingPointsWithEpiAndYOLO2 = std::chrono::steady_clock::now();
        cout << "RemoveMovingPointsWithEpiAndYOLO time:" << std::chrono::duration_cast<std::chrono::duration<double> >(RemoveMovingPointsWithEpiAndYOLO2 - RemoveMovingPointsWithEpiAndYOLO1).count() <<endl;
        std::chrono::steady_clock::time_point RemoveMovingKeyPoints1 = std::chrono::steady_clock::now();
#endif
        RemoveMovingKeyPoints(imGray, imDepth);
#ifdef TIMES
        std::chrono::steady_clock::time_point RemoveMovingKeyPoints2 = std::chrono::steady_clock::now();
        cout << "RemoveMovingKeyPoints time:" << std::chrono::duration_cast<std::chrono::duration<double> >(RemoveMovingKeyPoints2 - RemoveMovingKeyPoints1).count() <<endl;
#endif
    }
#endif
    // 获取特征点的个数
    N = mvKeys.size();

    // 如果这一帧没有能够提取出特征点，那么就直接返回了
    if(mvKeys.empty())
        return;

    // Step 4 用OpenCV的矫正函数、内参对提取到的特征点进行矫正
    UndistortKeyPoints();

    // Step 5 获取图像的深度，并且根据这个深度推算其右图中匹配的特征点的视差
    ComputeStereoFromRGBD(imDepth);
    // 初始化本帧的地图点
    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));

    mmProjectPoints.clear();
    mmMatchedInImage.clear();

    // 记录地图点是否为外点，初始化均为外点false
    mvbOutlier = vector<bool>(N,false);

    // This is done only for the first Frame (or after a change in the calibration)
    //  Step 5 计算去畸变后图像边界，将特征点分配到网格中。这个过程一般是在第一帧或者是相机标定参数发生变化之后进行
    if(mbInitialComputations)
    {
    // 计算去畸变后图像的边界
    ComputeImageBounds(imGray);

    // 表示一个图像像素相当于多少个图像网格列（宽）
    mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
    // 表示一个图像像素相当于多少个图像网格行（高）
    mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

    fx = K.at<float>(0,0);
    fy = K.at<float>(1,1);
    cx = K.at<float>(0,2);
    cy = K.at<float>(1,2);
    // 猜测是因为这种除法计算需要的时间略长，所以这里直接存储了这个中间计算结果
    invfx = 1.0f/fx;
    invfy = 1.0f/fy;

    // 特殊的初始化过程完成，标志复位
    mbInitialComputations=false;
    }

    // 计算假想的基线长度 baseline= mbf/fx
    // 后面要对从RGBD相机输入的特征点,结合相机基线长度,焦距,以及点的深度等信息来计算其在假想的"右侧图像"上的匹配点
    mb = mbf/fx;

    if(pPrevF){
    if(pPrevF->HasVelocity())
    SetVelocity(pPrevF->GetVelocity());
    }
    else{
    mVw.setZero();
    }

    mpMutexImu = new std::mutex();

    //Set no stereo fisheye information
    Nleft = -1;
    Nright = -1;
    mvLeftToRightMatch = vector<int>(0);
    mvRightToLeftMatch = vector<int>(0);
    mvStereo3Dpoints = vector<Eigen::Vector3f>(0);
    monoLeft = -1;
    monoRight = -1;

    // 将特征点分配到图像网格中
    AssignFeaturesToGrid();
}

void Frame::EpiloarWithFundMatrix(const cv::Mat &ImGrayPre, const cv::Mat &ImGray)
{
    Curpoint.clear();
    Prepoint.clear();
    epiErr.clear();
    state.clear();
    status.clear();
    Err.clear();
    for(auto it : mvKeys)
        Curpoint.emplace_back(it.pt);
//    for(auto it : mpTracker->mLastFrame.mvKeys)
#ifdef TIMES
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#endif
    cv::calcOpticalFlowPyrLK(ImGray,ImGrayPre,Curpoint,Prepoint,state,Err, cv::Size(21, 21), 3);
#ifdef TIMES
    std::chrono::steady_clock::time_point optical = std::chrono::steady_clock::now();
    cout << "optical flow time:" << std::chrono::duration_cast<std::chrono::duration<double> >(optical - t1).count() <<endl;
#endif
    //将跟踪误差小的用来估计基础矩阵，提高基础矩阵计算速度
    std::vector<cv::Point2f> PrepointT = Prepoint;
    std::vector<cv::Point2f> CurpointT = Curpoint;
    std::vector<float> flowErr = Err;
    sort(flowErr.begin(),flowErr.end());
    float th = flowErr[0.5*flowErr.size()];
    std::vector<bool> isErase(Curpoint.size(),true);

    reduceVector(Curpoint, isErase);
    reduceVector(Prepoint, isErase);
    int Nmp = Curpoint.size();

    // 平均光流幅值
    float sumFlow = 0.0;
    for (float err : Err) {
        sumFlow += err;
    }
    float avgFlow = sumFlow / Nmp;

    // 存储光流幅值
    std::vector<float> OpticalFL(Err.begin(), Err.end());
    for(auto err=Err.begin(); err!=Err.end(); ++err)
    {
        int i = err - Err.begin();
        if((*err)>th)
        {
            isErase[i] = false;
        }
    }

    reduceVector(Curpoint,isErase);
    reduceVector(Prepoint,isErase);
    int N = Curpoint.size();

    if(N>8)
    {
        cv::Mat FundMatrix;
        if(Curpoint.size()<15)
        {
#ifdef TIMES
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#endif
            FundMatrix = cv::findFundamentalMat(CurpointT,PrepointT,
                                                cv::FM_RANSAC,0.1,0.99,status);

#ifdef TIMES
            std::chrono::steady_clock::time_point fundmat = std::chrono::steady_clock::now();
            cout << "fundmat time:" << std::chrono::duration_cast<std::chrono::duration<double> >(fundmat - t2).count() <<endl;
            std::chrono::steady_clock::time_point CalcEpiDistChi21 = std::chrono::steady_clock::now();
#endif
            //计算极线距离
            CalcEpiDistChi2(mvKeys,PrepointT,FundMatrix,state);
#ifdef TIMES
            std::chrono::steady_clock::time_point CalcEpiDistChi22 = std::chrono::steady_clock::now();
            cout << "CalcEpiDistChi2 time:" << std::chrono::duration_cast<std::chrono::duration<double> >(CalcEpiDistChi22 - CalcEpiDistChi21).count() <<endl;
#endif
        }
        else
        {
#ifdef TIMES
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#endif
            FundMatrix = cv::findFundamentalMat(Curpoint,Prepoint,
                                                cv::FM_RANSAC,0.1,0.99,status);
            // 输出显示基础矩阵的元素值
//            for (int row = 0; row < 3; row++) {
//                for (int col = 0; col < 3; col++) {
//                    std::cout << "Element at (" << row << ", " << col << "): "
//                              << FundMatrix.at<double>(row, col) << std::endl;
//                }
//            }
#ifdef TIMES
            std::chrono::steady_clock::time_point fundmat = std::chrono::steady_clock::now();
            cout << "fundmat time:" << std::chrono::duration_cast<std::chrono::duration<double> >(fundmat - t2).count() <<endl;
            std::chrono::steady_clock::time_point CalcEpiDistChi21 = std::chrono::steady_clock::now();
#endif
            CalcEpiDistChi2(mvKeys,PrepointT,FundMatrix,state);
#ifdef TIMES
            std::chrono::steady_clock::time_point CalcEpiDistChi22 = std::chrono::steady_clock::now();
            cout << "CalcEpiDistChi2 time:" << std::chrono::duration_cast<std::chrono::duration<double> >(CalcEpiDistChi22 - CalcEpiDistChi21).count() <<endl;
#endif
        }
    }
}

void Frame::reduceVector(std::vector<cv::Point2f> &v, std::vector<bool> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

static double chiSquaredPDF(double x, double k) {
    if (x < 0.0 || k <= 0.0) {
        return 0.0;
    }

    double gammaTerm = std::tgamma(k / 2.0);
    double powTerm = std::pow(x, (k / 2.0) - 1.0);
    double expTerm = std::exp(-x / 2.0);
    double normalization = std::pow(2.0, k / 2.0) * gammaTerm;

    return (powTerm * expTerm) / normalization;
}

void Frame::CalcEpiDistChi2(const std::vector<cv::KeyPoint> &kp1, const std::vector<cv::Point2f> &kp2, cv::Mat &F, std::vector<uchar> state)
{
    int N=kp1.size();
//    EpiErr.assign(N,1);
//    epiErr.assign(N,0);
//    tbb::parallel_for(tbb::blocked_range<std::size_t>(0,N),[&](tbb::blocked_range<std::size_t>rN)
//    {
        for(size_t i=0; i<N; i++)
        {
            double A1 = F.at<double>(0, 0)*kp1[i].pt.x + F.at<double>(0, 1)*kp1[i].pt.y + F.at<double>(0, 2);
            double B1 = F.at<double>(1, 0)*kp1[i].pt.x + F.at<double>(1, 1)*kp1[i].pt.y + F.at<double>(1, 2);
            double C1 = F.at<double>(2, 0)*kp1[i].pt.x + F.at<double>(2, 1)*kp1[i].pt.y + F.at<double>(2, 2);
            double dis1 = fabs(A1*kp2[i].x + B1*kp2[i].y + C1) / sqrt(A1*A1 + B1*B1);

            double A2 = F.at<double>(0, 0)*kp2[i].x + F.at<double>(1, 0)*kp2[i].y + F.at<double>(2, 0);
            double B2 = F.at<double>(0, 1)*kp2[i].x + F.at<double>(1, 1)*kp2[i].y + F.at<double>(2, 1);
            double C2 = F.at<double>(0, 2)*kp2[i].x + F.at<double>(1, 2)*kp2[i].y + F.at<double>(2, 2);
            double dis2 = fabs(A2*kp1[i].pt.x + B2*kp1[i].pt.y + C2) / sqrt(A2*A2 + B2*B2);

            double dis = (dis1 + dis2)/2.0;
            EpiErr.emplace_back(dis);
            // 定义卡方分布变量
            double pdfValue = 2 * chiSquaredPDF(dis * dis, 2.0);
            epiErr.emplace_back(pdfValue);
        }
//    });
}

bool Frame::isInBox(cv::KeyPoint &kp,YoloBoundingBox &Box,bool extend)
{
    cv::Rect2f tmpRect = Box.GetRect();
    if(!extend) {
        if (kp.pt.x > tmpRect.tl().x - 10
            && kp.pt.y > tmpRect.tl().y - 10
            && kp.pt.x < tmpRect.br().x + 10
            && kp.pt.y < tmpRect.br().y + 10) {
            return true;
        }
    }
    else {
        if (kp.pt.x > tmpRect.tl().x - 0.2 * Box.width
            && kp.pt.y > tmpRect.tl().y - 10
            && kp.pt.x < tmpRect.br().x + 0.2 * Box.width
            && kp.pt.y < tmpRect.br().y + 10) {
            return true;
        }
    }
    return false;
}

void Frame::RemoveMovingPointsWithEpiAndYOLO(std::vector<double> epiErr, const cv::Mat &ImGray, const cv::Mat &ImDepth)
{
    if(epiErr.empty())
        return;
    std::vector<cv::KeyPoint> staticPoints;
    std::vector<double> epiErrStatic;
    std::vector<cv::KeyPoint> _mvKeys;
    cv::Mat _mDescriptors;
    std::vector<cv::Point2f> Pixeldynamic; // 新增的全局变量
    //没有检测到物体，仅做极线匹配
    if(mpTracker->yoloBoundingBoxList.empty())
    {
        for(int i=0; i<mvKeys.size(); i++)
        {
            if(epiErr[i] < 1.0)
            {
                _mvKeys.emplace_back(mvKeys[i]);
                _mDescriptors.push_back(mDescriptors.row(i));
            }
            else
            {
                mvKeysDynamic.emplace_back(mvKeys[i]);
                mDescriptorsDynamic.push_back(mDescriptors.row(i));
            }
        }
        mvKeys.swap(_mvKeys);
        mDescriptors = _mDescriptors.clone();
        return;
    }
    else
    {
        //遍历所有点
        for(int i=0; i<mvKeys.size(); i++)
        {
            //遍历所有框
            for(int j=0; j<mpTracker->yoloBoundingBoxList.size(); j++)
            {
                //判断是否在框内
                if(isInBox(mvKeys[i],mpTracker->yoloBoundingBoxList[j]))
                {
                    mpTracker->yoloBoundingBoxList[j].KeyPointsInBox.emplace_back(make_pair(i,mvKeys[i]));
                    mpTracker->yoloBoundingBoxList[j].epiErr.emplace_back(epiErr[i]);
                    Pixeldynamic.push_back(mvKeys[i].pt); // 将像素坐标加入Pixeldynamic中
                }
            }
        }
        //计算每个框的动静属性
        for(auto it=mpTracker->yoloBoundingBoxList.begin(); it!=mpTracker->yoloBoundingBoxList.end(); ++it)
        {
            int M = it->epiErr.size();
            int M1 = 0.1*M;
            int M2 = 0.2*M;
            int M3 = 0.3*M;
            if(M == 0)
                continue;
            sort(it->epiErr.begin(),it->epiErr.end());
            double proStatic = (it->epiErr[M1]+it->epiErr[M2]+it->epiErr[M3])/3.0;
            if(proStatic<1.0)
            {
                if(it->GetLabel() == "person" && proStatic<0.96)
                {
                    it->isDynamic = true;
                    it->proStatic = proStatic;
                }
                else if(it->GetLabel() != "person" && proStatic<0.8)
                {
                    it->isDynamic = true;
                    it->proStatic = proStatic;
                }
                else
                {
                    it->isDynamic = false;
                    it->proStatic = proStatic;
                }
            }
        }
    }
    // 在这里可以使用Pixeldynamic进行后续操作
}

void ComputePixelFlow(const std::vector<cv::Point2f>& Pixeldynamic, const cv::Mat& prevImg, const cv::Mat& curImg, const cv::Point2f& avgflow, std::vector<cv::Point2f>& VectorGs, cv::Mat& ImDepth)
{
    const double PYR_SCALE = 0.5;
    const int NUM_LEVELS = 3;
    const int WIN_SIZE = 15;
    const int NUM_ITERATIONS = 3;
    const double POLY_N = 5;
    const double POLY_SIGMA = 1.2;
    const int FLAGS = 0;

    std::vector<cv::Point2f> nextPoints;
    std::vector<double> flowDifferences;

    cv::calcOpticalFlowFarneback(prevImg, curImg, nextPoints, PYR_SCALE, NUM_LEVELS, WIN_SIZE, NUM_ITERATIONS, POLY_N, POLY_SIGMA, FLAGS);

    for (auto pt : Pixeldynamic)
    {
        if (pt.x >= 0 && pt.y >= 0 && pt.x < curImg.cols && pt.y < curImg.rows)
        {
            cv::Point2f flow = nextPoints.at(pt.y * curImg.cols + pt.x);
            cv::Point2f diff = flow - avgflow;
            double l1norm = std::abs(diff.x) + std::abs(diff.y);

            flowDifferences.push_back(l1norm);
        }
    }

    std::sort(flowDifferences.begin(), flowDifferences.end());
    int thresholdIndex = flowDifferences.size() / 3 * 2; // 获取后三分之二位置的索引

    for (auto pt : Pixeldynamic)
    {
        if (pt.x >= 0 && pt.y >= 0 && pt.x < curImg.cols && pt.y < curImg.rows)
        {
            cv::Point2f flow = nextPoints.at(pt.y * curImg.cols + pt.x);
            cv::Point2f diff = flow - avgflow;
            double l1norm = std::abs(diff.x) + std::abs(diff.y);

            if (l1norm > 1 && l1norm > flowDifferences[thresholdIndex])
            {
                VectorGs.push_back(flow);
                ImDepth.at<float>(pt.y, pt.x) = 0; // 将深度设置为0
            }
        }
    }
}



void Frame::RemoveMovingKeyPoints(const cv::Mat &ImGray, const cv::Mat &ImDepth)
{
    if(mpTracker->yoloBoundingBoxList.empty()||epiErr.empty())
        return;
//    for(auto it=mpTracker->yoloBoundingBoxList.begin(); it!=mpTracker->yoloBoundingBoxList.end(); ++it)
//    {
//        if(it->isDynamic)
//            break;
//        else if(it==mpTracker->yoloBoundingBoxList.end())
//            return;
//    }
    cv::Mat imDepth = ImDepth;
    cv::Mat imGray = ImGray;
    std::vector<cv::KeyPoint> _mvKeys;
    cv::Mat _mDescriptors;
    std::vector<cv::KeyPoint> tmpKeys;

    for(auto it=mpTracker->yoloBoundingBoxList.begin(); it!=mpTracker->yoloBoundingBoxList.end(); ++it)
    {
        int BoxSize = it->KeyPointsInBox.size();
        if(BoxSize == 0)
            continue;
        //将框内的疑似动态点放入容器
        for(int i=0; i<BoxSize; ++i)
        {
            it->mvKeysDynam.emplace_back(imDepth.at<float>(mvKeys[it->KeyPointsInBox[i].first].pt.y,
                                                           mvKeys[it->KeyPointsInBox[i].first].pt.x));
        }
        //计算当前框内点深度的平均值和标准差
        int M = it->mvKeysDynam.size();
        sort(it->mvKeysDynam.begin(),it->mvKeysDynam.end());
        it->mvKeysDynam.erase(it->mvKeysDynam.begin()+0.8*M,it->mvKeysDynam.end());
        M = it->mvKeysDynam.size();
        it->average = std::accumulate(it->mvKeysDynam.begin(),it->mvKeysDynam.end(),0.0)/M;
//        it->average = (it->mvKeysDynam[0.3*M]+it->mvKeysDynam[0.5*M]+it->mvKeysDynam[0.7*M])/3.0;
        float sum = 0.0;
        for(auto d:it->mvKeysDynam)
        {
            sum += (d-it->average)*(d-it->average);
        }
        it->stdcov = std::sqrt(sum/M);
    }
    // 对所有点进行判断
    for(auto kp=mvKeys.begin(); kp!=mvKeys.end(); ++kp)
    {
        bool flag = true;
        bool Isstatic = false;
        int i = kp - mvKeys.begin();
        for(auto it=mpTracker->yoloBoundingBoxList.begin(); it!=mpTracker->yoloBoundingBoxList.end(); ++it)
        {
            if(it->GetLabel()=="person"&&it->proStatic<0.96)
            {
                if(isInBox(*kp,*it,true) && flag == true)
                {
                    flag = false;
                    const float invSigma2 = 128 * imDepth.at<float>(kp->pt.y, kp->pt.x) *
                                            imDepth.at<float>(kp->pt.y, kp->pt.x) / (fx);
                    float chi = (imDepth.at<float>(kp->pt.y, kp->pt.x) - it->average) / it->stdcov;
                    if (chi *invSigma2*invSigma2* chi > 3.2 && EpiErr[i]<0.3)
                    {
                        if(imDepth.at<float>(kp->pt.y, kp->pt.x)>0)
                            flag = true;
                    }
                }
            }
            else if(it->isDynamic)
            {
                if(isInBox(*kp,*it))
                {
                    if(EpiErr[i]>0.2)
                        flag = false;
                }
            }
        }
        if(EpiErr[i]>1.0)
            flag = false;
        if(flag || Isstatic)
        {
//            epiErr2.emplace_back(epiErr[i]);
            _mvKeys.emplace_back(mvKeys[i]);
            _mDescriptors.push_back(mDescriptors.row(i));
        }
        else
        {
//            epiErr2.emplace_back(epiErr[i]);
            mvKeysDynamic.emplace_back(mvKeys[i]);
            mDescriptorsDynamic.push_back(mDescriptors.row(i));
        }
    }
    mvKeys.swap(_mvKeys);
    mDescriptors = _mDescriptors.clone();

    for(auto it=mpTracker->yoloBoundingBoxList.begin(); it!=mpTracker->yoloBoundingBoxList.end(); ++it)
    {
        int x1 = it->GetRect().tl().x;
        int y1 = it->GetRect().tl().y;
        int x2 = it->GetRect().br().x;
        int y2 = it->GetRect().br().y;
//        if(it->GetLabel()=="chair")
//        {
//            cv::rectangle(mpTracker->mimColor, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(255, 0, 0), -1);
//            cv::rectangle(mpTracker->mimLeft, cv::Point(x1, y1), cv::Point(x2, y2-(y2-y1)/3), cv::Scalar(255, 0, 0), -1);
//        }
//        else if(it->GetLabel()=="keyboard")
//            cv::rectangle(mpTracker->mimLeft, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 0), -1);
//        else if(it->GetLabel()=="tvmonitor")
//            cv::rectangle(mpTracker->mimLeft, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 255), -1);
//        else if(it->GetLabel()=="mouse")
//            cv::rectangle(mpTracker->mimLeft, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(255, 255, 0), -1);
//        else if(it->GetLabel()=="book")
//            cv::rectangle(mpTracker->mimLeft, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(255, 0, 255), -1);
    }

}

Frame::Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, GeometricCamera* pCamera, cv::Mat &distCoef, const float &bf, const float &thDepth, Frame* pPrevF, const IMU::Calib &ImuCalib)
    :mpcpi(NULL),mpORBvocabulary(voc),mpORBextractorLeft(extractor),mpORBextractorRight(static_cast<ORBextractor*>(NULL)),
     mTimeStamp(timeStamp), mK(static_cast<Pinhole*>(pCamera)->toK()), mK_(static_cast<Pinhole*>(pCamera)->toK_()), mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth),
     mImuCalib(ImuCalib), mpImuPreintegrated(NULL),mpPrevFrame(pPrevF),mpImuPreintegratedFrame(NULL), mpReferenceKF(static_cast<KeyFrame*>(NULL)), mbIsSet(false), mbImuPreintegrated(false), mpCamera(pCamera),
     mpCamera2(nullptr), mbHasPose(false), mbHasVelocity(false)
{
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

    // ORB extraction
#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_StartExtORB = std::chrono::steady_clock::now();
#endif
    ExtractORB(0,imGray,0,1000);
#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_EndExtORB = std::chrono::steady_clock::now();

    mTimeORB_Ext = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndExtORB - time_StartExtORB).count();
#endif


    N = mvKeys.size();
    if(mvKeys.empty())
        return;

    UndistortKeyPoints();

    // Set no stereo information
    mvuRight = vector<float>(N,-1);
    mvDepth = vector<float>(N,-1);
    mnCloseMPs = 0;

    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));

    mmProjectPoints.clear();// = map<long unsigned int, cv::Point2f>(N, static_cast<cv::Point2f>(NULL));
    mmMatchedInImage.clear();

    mvbOutlier = vector<bool>(N,false);

    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        ComputeImageBounds(imGray);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

        fx = static_cast<Pinhole*>(mpCamera)->toK().at<float>(0,0);
        fy = static_cast<Pinhole*>(mpCamera)->toK().at<float>(1,1);
        cx = static_cast<Pinhole*>(mpCamera)->toK().at<float>(0,2);
        cy = static_cast<Pinhole*>(mpCamera)->toK().at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }


    mb = mbf/fx;

    //Set no stereo fisheye information
    Nleft = -1;
    Nright = -1;
    mvLeftToRightMatch = vector<int>(0);
    mvRightToLeftMatch = vector<int>(0);
    mvStereo3Dpoints = vector<Eigen::Vector3f>(0);
    monoLeft = -1;
    monoRight = -1;

    AssignFeaturesToGrid();

    if(pPrevF)
    {
        if(pPrevF->HasVelocity())
        {
            SetVelocity(pPrevF->GetVelocity());
        }
    }
    else
    {
        mVw.setZero();
    }

    mpMutexImu = new std::mutex();
}


void Frame::AssignFeaturesToGrid()
{
    // Fill matrix with points
    const int nCells = FRAME_GRID_COLS*FRAME_GRID_ROWS;

    int nReserve = 0.5f*N/(nCells);

    for(unsigned int i=0; i<FRAME_GRID_COLS;i++)
        for (unsigned int j=0; j<FRAME_GRID_ROWS;j++){
            mGrid[i][j].reserve(nReserve);
            if(Nleft != -1){
                mGridRight[i][j].reserve(nReserve);
            }
        }



    for(int i=0;i<N;i++)
    {
        const cv::KeyPoint &kp = (Nleft == -1) ? mvKeysUn[i]
                                                 : (i < Nleft) ? mvKeys[i]
                                                                 : mvKeysRight[i - Nleft];

        int nGridPosX, nGridPosY;
        if(PosInGrid(kp,nGridPosX,nGridPosY)){
            if(Nleft == -1 || i < Nleft)
                mGrid[nGridPosX][nGridPosY].push_back(i);
            else
                mGridRight[nGridPosX][nGridPosY].push_back(i - Nleft);
        }
    }
}

void Frame::ExtractORB(int flag, const cv::Mat &im, const int x0, const int x1)
{
    vector<int> vLapping = {x0,x1};
    if(flag==0)
        monoLeft = (*mpORBextractorLeft)(im,cv::Mat(),mvKeys,mDescriptors,vLapping);
    else
        monoRight = (*mpORBextractorRight)(im,cv::Mat(),mvKeysRight,mDescriptorsRight,vLapping);
}

bool Frame::isSet() const {
    return mbIsSet;
}

void Frame::SetPose(const Sophus::SE3<float> &Tcw) {
    mTcw = Tcw;

    UpdatePoseMatrices();
    mbIsSet = true;
    mbHasPose = true;
}

void Frame::SetNewBias(const IMU::Bias &b)
{
    mImuBias = b;
    if(mpImuPreintegrated)
        mpImuPreintegrated->SetNewBias(b);
}

void Frame::SetVelocity(Eigen::Vector3f Vwb)
{
    mVw = Vwb;
    mbHasVelocity = true;
}

Eigen::Vector3f Frame::GetVelocity() const
{
    return mVw;
}

void Frame::SetImuPoseVelocity(const Eigen::Matrix3f &Rwb, const Eigen::Vector3f &twb, const Eigen::Vector3f &Vwb)
{
    mVw = Vwb;
    mbHasVelocity = true;

    Sophus::SE3f Twb(Rwb, twb);
    Sophus::SE3f Tbw = Twb.inverse();

    mTcw = mImuCalib.mTcb * Tbw;

    UpdatePoseMatrices();
    mbIsSet = true;
    mbHasPose = true;
}

void Frame::UpdatePoseMatrices()
{
    Sophus::SE3<float> Twc = mTcw.inverse();
    mRwc = Twc.rotationMatrix();
    mOw = Twc.translation();
    mRcw = mTcw.rotationMatrix();
    mtcw = mTcw.translation();
}

Eigen::Matrix<float,3,1> Frame::GetImuPosition() const {
    return mRwc * mImuCalib.mTcb.translation() + mOw;
}

Eigen::Matrix<float,3,3> Frame::GetImuRotation() {
    return mRwc * mImuCalib.mTcb.rotationMatrix();
}

Sophus::SE3<float> Frame::GetImuPose() {
    return mTcw.inverse() * mImuCalib.mTcb;
}

Sophus::SE3f Frame::GetRelativePoseTrl()
{
    return mTrl;
}

Sophus::SE3f Frame::GetRelativePoseTlr()
{
    return mTlr;
}

Eigen::Matrix3f Frame::GetRelativePoseTlr_rotation(){
    return mTlr.rotationMatrix();
}

Eigen::Vector3f Frame::GetRelativePoseTlr_translation() {
    return mTlr.translation();
}


bool Frame::isInFrustum(MapPoint *pMP, float viewingCosLimit)
{
    if(Nleft == -1){
        pMP->mbTrackInView = false;
        pMP->mTrackProjX = -1;
        pMP->mTrackProjY = -1;

        // 3D in absolute coordinates
        Eigen::Matrix<float,3,1> P = pMP->GetWorldPos();

        // 3D in camera coordinates
        const Eigen::Matrix<float,3,1> Pc = mRcw * P + mtcw;
        const float Pc_dist = Pc.norm();

        // Check positive depth
        const float &PcZ = Pc(2);
        const float invz = 1.0f/PcZ;
        if(PcZ<0.0f)
            return false;

        const Eigen::Vector2f uv = mpCamera->project(Pc);

        if(uv(0)<mnMinX || uv(0)>mnMaxX)
            return false;
        if(uv(1)<mnMinY || uv(1)>mnMaxY)
            return false;

        pMP->mTrackProjX = uv(0);
        pMP->mTrackProjY = uv(1);

        // Check distance is in the scale invariance region of the MapPoint
        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();
        const Eigen::Vector3f PO = P - mOw;
        const float dist = PO.norm();

        if(dist<minDistance || dist>maxDistance)
            return false;

        // Check viewing angle
        Eigen::Vector3f Pn = pMP->GetNormal();

        const float viewCos = PO.dot(Pn)/dist;

        if(viewCos<viewingCosLimit)
            return false;

        // Predict scale in the image
        const int nPredictedLevel = pMP->PredictScale(dist,this);

        // Data used by the tracking
        pMP->mbTrackInView = true;
        pMP->mTrackProjX = uv(0);
        pMP->mTrackProjXR = uv(0) - mbf*invz;

        pMP->mTrackDepth = Pc_dist;

        pMP->mTrackProjY = uv(1);
        pMP->mnTrackScaleLevel= nPredictedLevel;
        pMP->mTrackViewCos = viewCos;

        return true;
    }
    else{
        pMP->mbTrackInView = false;
        pMP->mbTrackInViewR = false;
        pMP -> mnTrackScaleLevel = -1;
        pMP -> mnTrackScaleLevelR = -1;

        pMP->mbTrackInView = isInFrustumChecks(pMP,viewingCosLimit);
        pMP->mbTrackInViewR = isInFrustumChecks(pMP,viewingCosLimit,true);

        return pMP->mbTrackInView || pMP->mbTrackInViewR;
    }
}

bool Frame::ProjectPointDistort(MapPoint* pMP, cv::Point2f &kp, float &u, float &v)
{

    // 3D in absolute coordinates
    Eigen::Vector3f P = pMP->GetWorldPos();

    // 3D in camera coordinates
    const Eigen::Vector3f Pc = mRcw * P + mtcw;
    const float &PcX = Pc(0);
    const float &PcY= Pc(1);
    const float &PcZ = Pc(2);

    // Check positive depth
    if(PcZ<0.0f)
    {
        cout << "Negative depth: " << PcZ << endl;
        return false;
    }

    // Project in image and check it is not outside
    const float invz = 1.0f/PcZ;
    u=fx*PcX*invz+cx;
    v=fy*PcY*invz+cy;

    if(u<mnMinX || u>mnMaxX)
        return false;
    if(v<mnMinY || v>mnMaxY)
        return false;

    float u_distort, v_distort;

    float x = (u - cx) * invfx;
    float y = (v - cy) * invfy;
    float r2 = x * x + y * y;
    float k1 = mDistCoef.at<float>(0);
    float k2 = mDistCoef.at<float>(1);
    float p1 = mDistCoef.at<float>(2);
    float p2 = mDistCoef.at<float>(3);
    float k3 = 0;
    if(mDistCoef.total() == 5)
    {
        k3 = mDistCoef.at<float>(4);
    }

    // Radial distorsion
    float x_distort = x * (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);
    float y_distort = y * (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);

    // Tangential distorsion
    x_distort = x_distort + (2 * p1 * x * y + p2 * (r2 + 2 * x * x));
    y_distort = y_distort + (p1 * (r2 + 2 * y * y) + 2 * p2 * x * y);

    u_distort = x_distort * fx + cx;
    v_distort = y_distort * fy + cy;


    u = u_distort;
    v = v_distort;

    kp = cv::Point2f(u, v);

    return true;
}

Eigen::Vector3f Frame::inRefCoordinates(Eigen::Vector3f pCw)
{
    return mRcw * pCw + mtcw;
}

vector<size_t> Frame::GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel, const int maxLevel, const bool bRight) const
{
    vector<size_t> vIndices;
    vIndices.reserve(N);

    float factorX = r;
    float factorY = r;

    const int nMinCellX = max(0,(int)floor((x-mnMinX-factorX)*mfGridElementWidthInv));
    if(nMinCellX>=FRAME_GRID_COLS)
    {
        return vIndices;
    }

    const int nMaxCellX = min((int)FRAME_GRID_COLS-1,(int)ceil((x-mnMinX+factorX)*mfGridElementWidthInv));
    if(nMaxCellX<0)
    {
        return vIndices;
    }

    const int nMinCellY = max(0,(int)floor((y-mnMinY-factorY)*mfGridElementHeightInv));
    if(nMinCellY>=FRAME_GRID_ROWS)
    {
        return vIndices;
    }

    const int nMaxCellY = min((int)FRAME_GRID_ROWS-1,(int)ceil((y-mnMinY+factorY)*mfGridElementHeightInv));
    if(nMaxCellY<0)
    {
        return vIndices;
    }

    const bool bCheckLevels = (minLevel>0) || (maxLevel>=0);

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            const vector<size_t> vCell = (!bRight) ? mGrid[ix][iy] : mGridRight[ix][iy];
            if(vCell.empty())
                continue;

            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = (Nleft == -1) ? mvKeysUn[vCell[j]]
                                                         : (!bRight) ? mvKeys[vCell[j]]
                                                                     : mvKeysRight[vCell[j]];
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

                if(fabs(distx)<factorX && fabs(disty)<factorY)
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


void Frame::ComputeBoW()
{
    if(mBowVec.empty())
    {
        vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
        mpORBvocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);
    }
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
    cv::undistortPoints(mat,mat, static_cast<Pinhole*>(mpCamera)->toK(),mDistCoef,cv::Mat(),mK);
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

        mat=mat.reshape(2);
        cv::undistortPoints(mat,mat,static_cast<Pinhole*>(mpCamera)->toK(),mDistCoef,cv::Mat(),mK);
        mat=mat.reshape(1);

        // Undistort corners
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

void Frame::ComputeStereoMatches()
{
    mvuRight = vector<float>(N,-1.0f);
    mvDepth = vector<float>(N,-1.0f);

    const int thOrbDist = (ORBmatcher::TH_HIGH+ORBmatcher::TH_LOW)/2;

    const int nRows = mpORBextractorLeft->mvImagePyramid[0].rows;

    //Assign keypoints to row table
    vector<vector<size_t> > vRowIndices(nRows,vector<size_t>());

    for(int i=0; i<nRows; i++)
        vRowIndices[i].reserve(200);

    const int Nr = mvKeysRight.size();

    for(int iR=0; iR<Nr; iR++)
    {
        const cv::KeyPoint &kp = mvKeysRight[iR];
        const float &kpY = kp.pt.y;
        const float r = 2.0f*mvScaleFactors[mvKeysRight[iR].octave];
        const int maxr = ceil(kpY+r);
        const int minr = floor(kpY-r);

        for(int yi=minr;yi<=maxr;yi++)
            vRowIndices[yi].push_back(iR);
    }

    // Set limits for search
    const float minZ = mb;
    const float minD = 0;
    const float maxD = mbf/minZ;

    // For each left keypoint search a match in the right image
    vector<pair<int, int> > vDistIdx;
    vDistIdx.reserve(N);

    for(int iL=0; iL<N; iL++)
    {
        const cv::KeyPoint &kpL = mvKeys[iL];
        const int &levelL = kpL.octave;
        const float &vL = kpL.pt.y;
        const float &uL = kpL.pt.x;

        const vector<size_t> &vCandidates = vRowIndices[vL];

        if(vCandidates.empty())
            continue;

        const float minU = uL-maxD;
        const float maxU = uL-minD;

        if(maxU<0)
            continue;

        int bestDist = ORBmatcher::TH_HIGH;
        size_t bestIdxR = 0;

        const cv::Mat &dL = mDescriptors.row(iL);

        // Compare descriptor to right keypoints
        for(size_t iC=0; iC<vCandidates.size(); iC++)
        {
            const size_t iR = vCandidates[iC];
            const cv::KeyPoint &kpR = mvKeysRight[iR];

            if(kpR.octave<levelL-1 || kpR.octave>levelL+1)
                continue;

            const float &uR = kpR.pt.x;

            if(uR>=minU && uR<=maxU)
            {
                const cv::Mat &dR = mDescriptorsRight.row(iR);
                const int dist = ORBmatcher::DescriptorDistance(dL,dR);

                if(dist<bestDist)
                {
                    bestDist = dist;
                    bestIdxR = iR;
                }
            }
        }

        // Subpixel match by correlation
        if(bestDist<thOrbDist)
        {
            // coordinates in image pyramid at keypoint scale
            const float uR0 = mvKeysRight[bestIdxR].pt.x;
            const float scaleFactor = mvInvScaleFactors[kpL.octave];
            const float scaleduL = round(kpL.pt.x*scaleFactor);
            const float scaledvL = round(kpL.pt.y*scaleFactor);
            const float scaleduR0 = round(uR0*scaleFactor);

            // sliding window search
            const int w = 5;
            cv::Mat IL = mpORBextractorLeft->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduL-w,scaleduL+w+1);

            int bestDist = INT_MAX;
            int bestincR = 0;
            const int L = 5;
            vector<float> vDists;
            vDists.resize(2*L+1);

            const float iniu = scaleduR0+L-w;
            const float endu = scaleduR0+L+w+1;
            if(iniu<0 || endu >= mpORBextractorRight->mvImagePyramid[kpL.octave].cols)
                continue;

            for(int incR=-L; incR<=+L; incR++)
            {
                cv::Mat IR = mpORBextractorRight->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduR0+incR-w,scaleduR0+incR+w+1);

                float dist = cv::norm(IL,IR,cv::NORM_L1);
                if(dist<bestDist)
                {
                    bestDist =  dist;
                    bestincR = incR;
                }

                vDists[L+incR] = dist;
            }

            if(bestincR==-L || bestincR==L)
                continue;

            // Sub-pixel match (Parabola fitting)
            const float dist1 = vDists[L+bestincR-1];
            const float dist2 = vDists[L+bestincR];
            const float dist3 = vDists[L+bestincR+1];

            const float deltaR = (dist1-dist3)/(2.0f*(dist1+dist3-2.0f*dist2));

            if(deltaR<-1 || deltaR>1)
                continue;

            // Re-scaled coordinate
            float bestuR = mvScaleFactors[kpL.octave]*((float)scaleduR0+(float)bestincR+deltaR);

            float disparity = (uL-bestuR);

            if(disparity>=minD && disparity<maxD)
            {
                if(disparity<=0)
                {
                    disparity=0.01;
                    bestuR = uL-0.01;
                }
                mvDepth[iL]=mbf/disparity;
                mvuRight[iL] = bestuR;
                vDistIdx.push_back(pair<int,int>(bestDist,iL));
            }
        }
    }

    sort(vDistIdx.begin(),vDistIdx.end());
    const float median = vDistIdx[vDistIdx.size()/2].first;
    const float thDist = 1.5f*1.4f*median;

    for(int i=vDistIdx.size()-1;i>=0;i--)
    {
        if(vDistIdx[i].first<thDist)
            break;
        else
        {
            mvuRight[vDistIdx[i].second]=-1;
            mvDepth[vDistIdx[i].second]=-1;
        }
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

        const float d = imDepth.at<float>(v,u);

        if(d>0)
        {
            mvDepth[i] = d;
            mvuRight[i] = kpU.pt.x-mbf/d;
        }
    }
}

bool Frame::UnprojectStereo(const int &i, Eigen::Vector3f &x3D)
{
    const float z = mvDepth[i];
    if(z>0) {
        const float u = mvKeysUn[i].pt.x;
        const float v = mvKeysUn[i].pt.y;
        const float x = (u-cx)*z*invfx;
        const float y = (v-cy)*z*invfy;
        Eigen::Vector3f x3Dc(x, y, z);
        x3D = mRwc * x3Dc + mOw;
        return true;
    } else
        return false;
}

bool Frame::imuIsPreintegrated()
{
    unique_lock<std::mutex> lock(*mpMutexImu);
    return mbImuPreintegrated;
}

void Frame::setIntegrated()
{
    unique_lock<std::mutex> lock(*mpMutexImu);
    mbImuPreintegrated = true;
}

Frame::Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor* extractorLeft, ORBextractor* extractorRight, ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth, GeometricCamera* pCamera, GeometricCamera* pCamera2, Sophus::SE3f& Tlr,Frame* pPrevF, const IMU::Calib &ImuCalib)
        :mpcpi(NULL), mpORBvocabulary(voc),mpORBextractorLeft(extractorLeft),mpORBextractorRight(extractorRight), mTimeStamp(timeStamp), mK(K.clone()), mK_(Converter::toMatrix3f(K)),  mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth),
         mImuCalib(ImuCalib), mpImuPreintegrated(NULL), mpPrevFrame(pPrevF),mpImuPreintegratedFrame(NULL), mpReferenceKF(static_cast<KeyFrame*>(NULL)), mbImuPreintegrated(false), mpCamera(pCamera), mpCamera2(pCamera2),
         mbHasPose(false), mbHasVelocity(false)

{
    imgLeft = imLeft.clone();
    imgRight = imRight.clone();

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

    // ORB extraction
#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_StartExtORB = std::chrono::steady_clock::now();
#endif
    thread threadLeft(&Frame::ExtractORB,this,0,imLeft,static_cast<KannalaBrandt8*>(mpCamera)->mvLappingArea[0],static_cast<KannalaBrandt8*>(mpCamera)->mvLappingArea[1]);
    thread threadRight(&Frame::ExtractORB,this,1,imRight,static_cast<KannalaBrandt8*>(mpCamera2)->mvLappingArea[0],static_cast<KannalaBrandt8*>(mpCamera2)->mvLappingArea[1]);
    threadLeft.join();
    threadRight.join();
#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_EndExtORB = std::chrono::steady_clock::now();

    mTimeORB_Ext = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndExtORB - time_StartExtORB).count();
#endif

    Nleft = mvKeys.size();
    Nright = mvKeysRight.size();
    N = Nleft + Nright;

    if(N == 0)
        return;

    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        ComputeImageBounds(imLeft);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/(mnMaxY-mnMinY);

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    mb = mbf / fx;

    // Sophus/Eigen
    mTlr = Tlr;
    mTrl = mTlr.inverse();
    mRlr = mTlr.rotationMatrix();
    mtlr = mTlr.translation();

#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_StartStereoMatches = std::chrono::steady_clock::now();
#endif
    ComputeStereoFishEyeMatches();
#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_EndStereoMatches = std::chrono::steady_clock::now();

    mTimeStereoMatch = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndStereoMatches - time_StartStereoMatches).count();
#endif

    //Put all descriptors in the same matrix
    cv::vconcat(mDescriptors,mDescriptorsRight,mDescriptors);

    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(nullptr));
    mvbOutlier = vector<bool>(N,false);

    AssignFeaturesToGrid();

    mpMutexImu = new std::mutex();

    UndistortKeyPoints();

}

void Frame::ComputeStereoFishEyeMatches() {
    //Speed it up by matching keypoints in the lapping area
    vector<cv::KeyPoint> stereoLeft(mvKeys.begin() + monoLeft, mvKeys.end());
    vector<cv::KeyPoint> stereoRight(mvKeysRight.begin() + monoRight, mvKeysRight.end());

    cv::Mat stereoDescLeft = mDescriptors.rowRange(monoLeft, mDescriptors.rows);
    cv::Mat stereoDescRight = mDescriptorsRight.rowRange(monoRight, mDescriptorsRight.rows);

    mvLeftToRightMatch = vector<int>(Nleft,-1);
    mvRightToLeftMatch = vector<int>(Nright,-1);
    mvDepth = vector<float>(Nleft,-1.0f);
    mvuRight = vector<float>(Nleft,-1);
    mvStereo3Dpoints = vector<Eigen::Vector3f>(Nleft);
    mnCloseMPs = 0;

    //Perform a brute force between Keypoint in the left and right image
    vector<vector<cv::DMatch>> matches;

    BFmatcher.knnMatch(stereoDescLeft,stereoDescRight,matches,2);

    int nMatches = 0;
    int descMatches = 0;

    //Check matches using Lowe's ratio
    for(vector<vector<cv::DMatch>>::iterator it = matches.begin(); it != matches.end(); ++it){
        if((*it).size() >= 2 && (*it)[0].distance < (*it)[1].distance * 0.7){
            //For every good match, check parallax and reprojection error to discard spurious matches
            Eigen::Vector3f p3D;
            descMatches++;
            float sigma1 = mvLevelSigma2[mvKeys[(*it)[0].queryIdx + monoLeft].octave], sigma2 = mvLevelSigma2[mvKeysRight[(*it)[0].trainIdx + monoRight].octave];
            float depth = static_cast<KannalaBrandt8*>(mpCamera)->TriangulateMatches(mpCamera2,mvKeys[(*it)[0].queryIdx + monoLeft],mvKeysRight[(*it)[0].trainIdx + monoRight],mRlr,mtlr,sigma1,sigma2,p3D);
            if(depth > 0.0001f){
                mvLeftToRightMatch[(*it)[0].queryIdx + monoLeft] = (*it)[0].trainIdx + monoRight;
                mvRightToLeftMatch[(*it)[0].trainIdx + monoRight] = (*it)[0].queryIdx + monoLeft;
                mvStereo3Dpoints[(*it)[0].queryIdx + monoLeft] = p3D;
                mvDepth[(*it)[0].queryIdx + monoLeft] = depth;
                nMatches++;
            }
        }
    }
}

bool Frame::isInFrustumChecks(MapPoint *pMP, float viewingCosLimit, bool bRight) {
    // 3D in absolute coordinates
    Eigen::Vector3f P = pMP->GetWorldPos();

    Eigen::Matrix3f mR;
    Eigen::Vector3f mt, twc;
    if(bRight){
        Eigen::Matrix3f Rrl = mTrl.rotationMatrix();
        Eigen::Vector3f trl = mTrl.translation();
        mR = Rrl * mRcw;
        mt = Rrl * mtcw + trl;
        twc = mRwc * mTlr.translation() + mOw;
    }
    else{
        mR = mRcw;
        mt = mtcw;
        twc = mOw;
    }

    // 3D in camera coordinates
    Eigen::Vector3f Pc = mR * P + mt;
    const float Pc_dist = Pc.norm();
    const float &PcZ = Pc(2);

    // Check positive depth
    if(PcZ<0.0f)
        return false;

    // Project in image and check it is not outside
    Eigen::Vector2f uv;
    if(bRight) uv = mpCamera2->project(Pc);
    else uv = mpCamera->project(Pc);

    if(uv(0)<mnMinX || uv(0)>mnMaxX)
        return false;
    if(uv(1)<mnMinY || uv(1)>mnMaxY)
        return false;

    // Check distance is in the scale invariance region of the MapPoint
    const float maxDistance = pMP->GetMaxDistanceInvariance();
    const float minDistance = pMP->GetMinDistanceInvariance();
    const Eigen::Vector3f PO = P - twc;
    const float dist = PO.norm();

    if(dist<minDistance || dist>maxDistance)
        return false;

    // Check viewing angle
    Eigen::Vector3f Pn = pMP->GetNormal();

    const float viewCos = PO.dot(Pn) / dist;

    if(viewCos<viewingCosLimit)
        return false;

    // Predict scale in the image
    const int nPredictedLevel = pMP->PredictScale(dist,this);

    if(bRight){
        pMP->mTrackProjXR = uv(0);
        pMP->mTrackProjYR = uv(1);
        pMP->mnTrackScaleLevelR= nPredictedLevel;
        pMP->mTrackViewCosR = viewCos;
        pMP->mTrackDepthR = Pc_dist;
    }
    else{
        pMP->mTrackProjX = uv(0);
        pMP->mTrackProjY = uv(1);
        pMP->mnTrackScaleLevel= nPredictedLevel;
        pMP->mTrackViewCos = viewCos;
        pMP->mTrackDepth = Pc_dist;
    }

    return true;
}

Eigen::Vector3f Frame::UnprojectStereoFishEye(const int &i){
    return mRwc * mvStereo3Dpoints[i] + mOw;
}

cv::Mat Frame::GetColorImage() 
{
    return mImColor;
}

cv::Mat Frame::GetDepthImage()
{
    return mImDepth;
}

} //namespace ORB_SLAM

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
#include "KeyFrame.h"
#include "MapPoint.h"
#include <Eigen/Core>
#include "opencv2/core/eigen.hpp"
#include "Map.h"
#include <neural-graphics-primitives/common.h> 
#include<mutex>

namespace Eigen
{
    // reference from https://github.com/nlohmann/json/issues/3267
    template<typename Scalar, int Rows, int Cols>
    void to_json(nlohmann::json& j, const Matrix<Scalar, Rows, Cols>& matrix) {
        for (int row = 0; row < matrix.rows(); ++row) {
            nlohmann::json column = nlohmann::json::array();
            for (int col = 0; col < matrix.cols(); ++col) {
                column.push_back(matrix(row, col));
            }
            j.push_back(column);
        }
    }

    template<typename Scalar, int Rows, int Cols>
    void from_json(const nlohmann::json& j, Matrix<Scalar, Rows, Cols>& matrix) {        
        for (std::size_t row = 0; row < j.size(); ++row) {
            const auto& jrow = j.at(row);
            for (std::size_t col = 0; col < jrow.size(); ++col) {
                const auto& value = jrow.at(col);
                value.get_to(matrix(row, col));
            }
        }
    }
}

namespace ORB_SLAM3
{

long unsigned int Map::nNextId=0;

Map::Map():mnMaxKFid(0),mnBigChangeIdx(0), mbImuInitialized(false), mnMapChange(0), mpFirstRegionKF(static_cast<KeyFrame*>(NULL)),
mbFail(false), mIsInUse(false), mHasTumbnail(false), mbBad(false), mnMapChangeNotified(0), mbIsInertial(false), mbIMU_BA1(false), mbIMU_BA2(false), mbDataIsReady(false)
{
    mnId=nNextId++;
    mThumbnail = static_cast<GLubyte*>(NULL);
}

Map::Map(int initKFid):mnInitKFid(initKFid), mnMaxKFid(initKFid),/*mnLastLoopKFid(initKFid),*/ mnBigChangeIdx(0), mIsInUse(false),
                       mHasTumbnail(false), mbBad(false), mbImuInitialized(false), mpFirstRegionKF(static_cast<KeyFrame*>(NULL)),
                       mnMapChange(0), mbFail(false), mnMapChangeNotified(0), mbIsInertial(false), mbIMU_BA1(false), mbIMU_BA2(false), mbDataIsReady(false)
{
    mnId=nNextId++;
    mThumbnail = static_cast<GLubyte*>(NULL);
}

Map::~Map()
{
    //TODO: erase all points from memory
    mspMapPoints.clear();

    //TODO: erase all keyframes from memory
    mspKeyFrames.clear();

    if(mThumbnail)
        delete mThumbnail;
    mThumbnail = static_cast<GLubyte*>(NULL);

    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
}

cv::FileNode Map::KeyCheck(cv::FileNode& parent_node, const string& key) 
{
    cv::FileNode node = parent_node[key];
    if (node.empty()) 
    {
        throw runtime_error{string{"The key: "} + key + string{" is not in the yaml. "}};
    }
    else{
        return node;
    }
}

void Map::init_window()
{
    unique_lock<mutex> lock(mMutexMap);
#ifdef ORBEEZ_GUI
    mpTestbed->init_window(1920, 1080);
#endif
}

void Map::StopTraining()
{
    mpTestbed->m_train = false;
}

bool Map::frame()
{
    // The lock ensure modifying dataset (add keyframe) and training won't do simultaneously
    unique_lock<mutex> lock(mMutexMap);

    if (mbDataIsReady){

        // Draw sparse point cloud
        const vector<MapPoint*> &vpMPs    = vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
        const vector<MapPoint*> &vpRefMPs = mvpReferenceMapPoints;
        set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

        std::vector<Eigen::Vector3f> map_points;
        for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
        {
            if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
                continue;
            Eigen::Vector3f  pos = vpMPs[i]->GetWorldPos();
            Eigen::Vector3f map_point = pos;
            map_points.push_back(map_point);
        }

        std::vector<Eigen::Vector3f> ref_map_points;
        for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
        {
            if((*sit)->isBad())
                continue;
            Eigen::Vector3f  pos = (*sit)->GetWorldPos();
            Eigen::Vector3f ref_map_point = pos;
            ref_map_points.push_back(ref_map_point);
        }

        mpTestbed->add_sparse_point_cloud(map_points, ref_map_points);

        // train and render
        bool value = mpTestbed->frame();
        // cout<<mpTestbed->m_train<<endl;
        if(mpTestbed->m_train)
            tlog::info() << "iteration=" << mpTestbed->m_training_step << " loss=" << mpTestbed->m_loss_scalar.val();

        return value;
    }
    else
        return false;
}

Eigen::Matrix<float, 3, 4> Map::KeyFrameWorldPoseToNGPFormat(const Eigen::Matrix<float, 3, 4>& slam_matrix) const
{
    return mpTestbed->m_nerf.training.dataset.slam_matrix_to_ngp(slam_matrix);
}

Eigen::Matrix<float, 3, 4> Map::KeyFrameNGPFormatToWorldPose(const Eigen::Matrix<float, 3, 4>& ngp_matrix) const
{
    return mpTestbed->m_nerf.training.dataset.ngp_matrix_to_slam(ngp_matrix);
}

Eigen::Matrix<float, 3, 4> Map::PoseWithPhotometric(int index) const
{
    return KeyFrameNGPFormatToWorldPose(mpTestbed->m_nerf.training.transforms[index].start);
}

void Map::AddKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    if(mspKeyFrames.empty()){
        cout << "First KF:" << pKF->mnId << "; Map init KF:" << mnInitKFid << endl;
        mnInitKFid = pKF->mnId;
        mpKFinitial = pKF;
        mpKFlowerID = pKF;
    }
    mspKeyFrames.insert(pKF);
    if(pKF->mnId>mnMaxKFid)
    {
        mnMaxKFid=pKF->mnId;
    }
    if(pKF->mnId<mpKFlowerID->mnId)
    {
        mpKFlowerID = pKF;
    }
    mbDataIsReady = true;
    Eigen::MatrixXf transform_matrix = pKF->GetPoseInverse().matrix();

    nlohmann::json frame_config;
    frame_config["Id"]                        = pKF->mnId;  
    frame_config["fl_x"]                      = m_scene_config["fl_x"];
    frame_config["fl_y"]                      = m_scene_config["fl_y"];
    frame_config["transform_matrix"]          = transform_matrix;
    frame_config["w"]                         = m_scene_config["w"];
    frame_config["h"]                         = m_scene_config["h"];
    
    cv::Mat color_image                       = pKF->GetColorImage();
    cv::Mat depth_image                       = pKF->GetDepthImage();

    // std::cout << "[Map] frame_config:" << frame_config.dump(4) << std::endl;
    // cout<<m_scene_config["w"]<<endl;
    // cout<<m_scene_config["h"]<<endl;
    // cout<<color_image.cols <<endl;
    // cout<<color_image.rows<<endl;
    if (frame_config["w"] != color_image.cols || frame_config["h"] != color_image.rows)
    {
        throw runtime_error("Image size does not match the yaml. Maybe use the wrong yaml");
    }

    // instant ngp requires image to have 4 channels
    cv::Mat rgba;
    if (color_image.channels() == 3) {
        // https://stackoverflow.com/questions/32290096/python-opencv-add-alpha-channel-to-rgb-image
        // First create the image with alpha channel
        cv::cvtColor(color_image, rgba, cv::COLOR_BGR2RGBA);

        // Split the image for access to alpha channel
        cv::Mat channels[4];
        cv::split(rgba, channels);

        // Assign the mask to the last channel of the image
        channels[3] = 255 * cv::Mat::ones(channels[3].rows, channels[3].cols, CV_8UC1);

        // Finally concat channels for rgba image
        cv::merge(channels, 4, rgba);
    }
    else if (color_image.channels() == 4){
        // instant-ngp may release the image memory. Therefore, clone the image.
        rgba = color_image.clone();
    }
    else{
        throw std::runtime_error("incorrect image format");
    }

    assert(rgba.isContinuous());

    // prepare an additional memory for instant-ngp. It will be free in instant-ngp
    uint8_t *img = (uint8_t*)malloc(sizeof(uint8_t) * rgba.rows * rgba.cols * rgba.channels());
    memcpy(img, rgba.data, sizeof(uint8_t) * rgba.rows * rgba.cols * rgba.channels());

    uint16_t *depth = nullptr;
    
    // if(!depth_image.empty()){

    //     if (frame_config["w"] != depth_image.cols || frame_config["h"] != depth_image.rows)
    //     {
    //         throw runtime_error("depth image must be same as color_image. Please resize them in the main program");
    //     }

    //     depth = (uint16_t*)malloc(sizeof(uint16_t) * depth_image.rows * depth_image.cols * depth_image.channels());
    //     memcpy(depth, depth_image.data, sizeof(uint8_t) * depth_image.rows * depth_image.cols * depth_image.channels());
    // }

    std::tuple<ngp::TrainingXForm*,int> t = mpTestbed->add_training_image(frame_config, img, depth);
    ngp::TrainingXForm *pXform = std::get<0>(t);
    int index = std::get<1>(t);

    pKF->SetNerfXformPointer(pXform, index);
}

void Map::update_transformsGPU()
{
    unique_lock<mutex> lock(mMutexMap);
    mpTestbed->update_camera(mpTestbed->m_training_stream);
}

bool Map::NerfCameraIsUpdated()
{
    unique_lock<mutex> lock(mMutexMap);
    return (mpTestbed->m_nerf.training.optimize_extrinsics) && (mpTestbed->m_nerf.training.n_steps_since_cam_update == 0);
}

void Map::GetCameraInfo(float *slice_plane_z, float *scale, float *fov, float *dof)
{
    *slice_plane_z = mpTestbed->m_slice_plane_z;
    *scale = mpTestbed->scale();
    *fov = mpTestbed->fov();
    *dof = mpTestbed->m_dof;
}

nlohmann::json Map::GetSceneConfig()
{
    return m_scene_config;
}

void Map::CullEmptyRegion()
{
    unique_lock<mutex> lock(mMutexMap);
    std::cout << "Cull empty region (camera can't see) in density grid" << std::endl;
    // Mesh use inference stream
    mpTestbed->cull_empty_region(false, mpTestbed->m_inference_stream);
}

void Map::SaveSnapShot(const string &filename)
{
    unique_lock<mutex> lock(mMutexMap);
    mpTestbed->save_snapshot(filename, false);
}

void Map::SaveMesh(const string &filename, uint32_t marching_cubes_res)
{
    unique_lock<mutex> lock(mMutexMap);
    // std::cout << "SaveMesh" << std::endl;
    Eigen::Vector3i res3d(3);
    res3d << marching_cubes_res, marching_cubes_res, marching_cubes_res;
    mpTestbed->compute_and_save_marching_cubes_mesh(filename.c_str(), res3d);
}

void Map::AddGroundTruthTraj(const std::string& gtTrajPath)
{
    unique_lock<mutex> lock(mMutexMap);
    mpTestbed->AddGroundTruthTraj(gtTrajPath);
}

void Map::AddMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.insert(pMP);
}

void Map::SetImuInitialized()
{
    unique_lock<mutex> lock(mMutexMap);
    mbImuInitialized = true;
}

bool Map::isImuInitialized()
{
    unique_lock<mutex> lock(mMutexMap);
    return mbImuInitialized;
}

void Map::EraseMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.erase(pMP);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::EraseKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.erase(pKF);
    if(mspKeyFrames.size()>0)
    {
        if(pKF->mnId == mpKFlowerID->mnId)
        {
            vector<KeyFrame*> vpKFs = vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
            sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);
            mpKFlowerID = vpKFs[0];
        }
    }
    else
    {
        mpKFlowerID = 0;
    }

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
{
    unique_lock<mutex> lock(mMutexMap);
    mvpReferenceMapPoints = vpMPs;
}

void Map::InformNewBigChange()
{
    unique_lock<mutex> lock(mMutexMap);
    mnBigChangeIdx++;
}

int Map::GetLastBigChangeIdx()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnBigChangeIdx;
}

vector<KeyFrame*> Map::GetAllKeyFrames()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
}

vector<MapPoint*> Map::GetAllMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
}

long unsigned int Map::MapPointsInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspMapPoints.size();
}

long unsigned int Map::KeyFramesInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspKeyFrames.size();
}

vector<MapPoint*> Map::GetReferenceMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return mvpReferenceMapPoints;
}

long unsigned int Map::GetId()
{
    return mnId;
}
long unsigned int Map::GetInitKFid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnInitKFid;
}

void Map::SetInitKFid(long unsigned int initKFif)
{
    unique_lock<mutex> lock(mMutexMap);
    mnInitKFid = initKFif;
}

long unsigned int Map::GetMaxKFid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMaxKFid;
}

KeyFrame* Map::GetOriginKF()
{
    return mpKFinitial;
}

void Map::SetCurrentMap()
{
    mIsInUse = true;
}

void Map::SetStoredMap()
{
    mIsInUse = false;
}

void Map::clear()
{
//    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
//        delete *sit;

    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
    {
        KeyFrame* pKF = *sit;
        pKF->UpdateMap(static_cast<Map*>(NULL));
//        delete *sit;
    }

    mspMapPoints.clear();
    mspKeyFrames.clear();
    mnMaxKFid = mnInitKFid;
    mbImuInitialized = false;
    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
    mbIMU_BA1 = false;
    mbIMU_BA2 = false;
}

bool Map::IsInUse()
{
    return mIsInUse;
}

void Map::SetBad()
{
    mbBad = true;
}

bool Map::IsBad()
{
    return mbBad;
}


void Map::ApplyScaledRotation(const Sophus::SE3f &T, const float s, const bool bScaledVel)
{
    unique_lock<mutex> lock(mMutexMap);

    // Body position (IMU) of first keyframe is fixed to (0,0,0)
    Sophus::SE3f Tyw = T;
    Eigen::Matrix3f Ryw = Tyw.rotationMatrix();
    Eigen::Vector3f tyw = Tyw.translation();

    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(); sit!=mspKeyFrames.end(); sit++)
    {
        KeyFrame* pKF = *sit;
        Sophus::SE3f Twc = pKF->GetPoseInverse();
        Twc.translation() *= s;
        Sophus::SE3f Tyc = Tyw*Twc;
        Sophus::SE3f Tcy = Tyc.inverse();
        pKF->SetPose(Tcy);
        Eigen::Vector3f Vw = pKF->GetVelocity();
        if(!bScaledVel)
            pKF->SetVelocity(Ryw*Vw);
        else
            pKF->SetVelocity(Ryw*Vw*s);

    }
    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(); sit!=mspMapPoints.end(); sit++)
    {
        MapPoint* pMP = *sit;
        pMP->SetWorldPos(s * Ryw * pMP->GetWorldPos() + tyw);
        pMP->UpdateNormalAndDepth();
    }
    mnMapChange++;
}

void Map::SetInertialSensor()
{
    unique_lock<mutex> lock(mMutexMap);
    mbIsInertial = true;
}

bool Map::IsInertial()
{
    unique_lock<mutex> lock(mMutexMap);
    return mbIsInertial;
}

void Map::SetIniertialBA1()
{
    unique_lock<mutex> lock(mMutexMap);
    mbIMU_BA1 = true;
}

void Map::SetIniertialBA2()
{
    unique_lock<mutex> lock(mMutexMap);
    mbIMU_BA2 = true;
}

bool Map::GetIniertialBA1()
{
    unique_lock<mutex> lock(mMutexMap);
    return mbIMU_BA1;
}

bool Map::GetIniertialBA2()
{
    unique_lock<mutex> lock(mMutexMap);
    return mbIMU_BA2;
}

void Map::ChangeId(long unsigned int nId)
{
    mnId = nId;
}

unsigned int Map::GetLowerKFID()
{
    unique_lock<mutex> lock(mMutexMap);
    if (mpKFlowerID) {
        return mpKFlowerID->mnId;
    }
    return 0;
}

int Map::GetMapChangeIndex()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMapChange;
}

void Map::IncreaseChangeIndex()
{
    unique_lock<mutex> lock(mMutexMap);
    mnMapChange++;
}

int Map::GetLastMapChange()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMapChangeNotified;
}

void Map::SetLastMapChange(int currentChangeId)
{
    unique_lock<mutex> lock(mMutexMap);
    mnMapChangeNotified = currentChangeId;
}

void Map::PreSave(std::set<GeometricCamera*> &spCams)
{
    int nMPWithoutObs = 0;
    for(MapPoint* pMPi : mspMapPoints)
    {
        if(!pMPi || pMPi->isBad())
            continue;

        if(pMPi->GetObservations().size() == 0)
        {
            nMPWithoutObs++;
        }
        map<KeyFrame*, std::tuple<int,int>> mpObs = pMPi->GetObservations();
        for(map<KeyFrame*, std::tuple<int,int>>::iterator it= mpObs.begin(), end=mpObs.end(); it!=end; ++it)
        {
            if(it->first->GetMap() != this || it->first->isBad())
            {
                pMPi->EraseObservation(it->first);
            }

        }
    }

    // Saves the id of KF origins
    mvBackupKeyFrameOriginsId.clear();
    mvBackupKeyFrameOriginsId.reserve(mvpKeyFrameOrigins.size());
    for(int i = 0, numEl = mvpKeyFrameOrigins.size(); i < numEl; ++i)
    {
        mvBackupKeyFrameOriginsId.push_back(mvpKeyFrameOrigins[i]->mnId);
    }


    // Backup of MapPoints
    mvpBackupMapPoints.clear();
    for(MapPoint* pMPi : mspMapPoints)
    {
        if(!pMPi || pMPi->isBad())
            continue;

        mvpBackupMapPoints.push_back(pMPi);
        pMPi->PreSave(mspKeyFrames,mspMapPoints);
    }

    // Backup of KeyFrames
    mvpBackupKeyFrames.clear();
    for(KeyFrame* pKFi : mspKeyFrames)
    {
        if(!pKFi || pKFi->isBad())
            continue;

        mvpBackupKeyFrames.push_back(pKFi);
        pKFi->PreSave(mspKeyFrames,mspMapPoints, spCams);
    }

    mnBackupKFinitialID = -1;
    if(mpKFinitial)
    {
        mnBackupKFinitialID = mpKFinitial->mnId;
    }

    mnBackupKFlowerID = -1;
    if(mpKFlowerID)
    {
        mnBackupKFlowerID = mpKFlowerID->mnId;
    }

}

void Map::PostLoad(KeyFrameDatabase* pKFDB, ORBVocabulary* pORBVoc/*, map<long unsigned int, KeyFrame*>& mpKeyFrameId*/, map<unsigned int, GeometricCamera*> &mpCams)
{
    std::copy(mvpBackupMapPoints.begin(), mvpBackupMapPoints.end(), std::inserter(mspMapPoints, mspMapPoints.begin()));
    std::copy(mvpBackupKeyFrames.begin(), mvpBackupKeyFrames.end(), std::inserter(mspKeyFrames, mspKeyFrames.begin()));

    map<long unsigned int,MapPoint*> mpMapPointId;
    for(MapPoint* pMPi : mspMapPoints)
    {
        if(!pMPi || pMPi->isBad())
            continue;

        pMPi->UpdateMap(this);
        mpMapPointId[pMPi->mnId] = pMPi;
    }

    map<long unsigned int, KeyFrame*> mpKeyFrameId;
    for(KeyFrame* pKFi : mspKeyFrames)
    {
        if(!pKFi || pKFi->isBad())
            continue;

        pKFi->UpdateMap(this);
        pKFi->SetORBVocabulary(pORBVoc);
        pKFi->SetKeyFrameDatabase(pKFDB);
        mpKeyFrameId[pKFi->mnId] = pKFi;
    }

    // References reconstruction between different instances
    for(MapPoint* pMPi : mspMapPoints)
    {
        if(!pMPi || pMPi->isBad())
            continue;

        pMPi->PostLoad(mpKeyFrameId, mpMapPointId);
    }

    for(KeyFrame* pKFi : mspKeyFrames)
    {
        if(!pKFi || pKFi->isBad())
            continue;

        pKFi->PostLoad(mpKeyFrameId, mpMapPointId, mpCams);
        pKFDB->add(pKFi);
    }


    if(mnBackupKFinitialID != -1)
    {
        mpKFinitial = mpKeyFrameId[mnBackupKFinitialID];
    }

    if(mnBackupKFlowerID != -1)
    {
        mpKFlowerID = mpKeyFrameId[mnBackupKFlowerID];
    }

    mvpKeyFrameOrigins.clear();
    mvpKeyFrameOrigins.reserve(mvBackupKeyFrameOriginsId.size());
    for(int i = 0; i < mvBackupKeyFrameOriginsId.size(); ++i)
    {
        mvpKeyFrameOrigins.push_back(mpKeyFrameId[mvBackupKeyFrameOriginsId[i]]);
    }

    mvpBackupMapPoints.clear();
}


} //namespace ORB_SLAM3

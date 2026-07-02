#include "utils.h"

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/string.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

ORB_SLAM3::ORBVocabulary *vocab = nullptr;

ORB_SLAM3::Atlas* load_atlas_from_file(const std::string &url) {
    ORB_SLAM3::Atlas *atlas = new ORB_SLAM3::Atlas();
    std::ifstream ifs(url, std::ios::binary);
    boost::archive::binary_iarchive ia(ifs);
    std::string strFileVoc, strVocChecksum;
    ia >> strFileVoc;
    ia >> strVocChecksum;
    ia >> atlas;
    return atlas;
}

void alignMap(ORB_SLAM3::Map* map) {
    // compute the best fit rotation between the camera poses and the IMU poses in the map. 
    // This is a complex problem that typically involves solving a Procrustes problem or using 
    // SVD to find the optimal rotation that minimizes the distance between the two sets of orientations. 
    auto kfs = map->GetAllKeyFrames();
    if (kfs.size() < 5) {
        ORB_SLAM3::VerboseStream(ORB_SLAM3::Verbose::VERBOSITY_NORMAL) << "Not enough keyframes to align map (need at least 5)";
        return;
    }
    Eigen::Matrix3Xf cam_orientations(3,kfs.size()*3);
    Eigen::Matrix3Xf imu_orientations(3, kfs.size()*3);
    for (size_t i = 0; i < kfs.size(); i++) {
        auto kf = kfs[i];
        //if (kf->isBad()) continue;
        auto cam_q = kf->GetPose().so3().inverse();
        auto imu_q = kf->GetPoseFromEstimate().so3().inverse();
        cam_orientations.block<3,3>(0,i*3) = cam_q.matrix();
        imu_orientations.block<3,3>(0,i*3) = imu_q.matrix();
    }
    Eigen::Matrix4f T = Eigen::umeyama(cam_orientations, imu_orientations, false);
    Eigen::Matrix3f R = T.block<3,3>(0,0);
    Sophus::SE3f R_se3(R, Eigen::Vector3f::Zero());
    for (ORB_SLAM3::KeyFrame* pKF : map->GetAllKeyFrames()) {
        if (pKF->isBad()) continue;
        auto pose = R_se3 * pKF->GetPose().inverse();
        pKF->SetPose(pose.inverse());
    }
    for (ORB_SLAM3::MapPoint* pMP : map->GetAllMapPoints()) {
        if (pMP->isBad()) continue;
        auto pos = R_se3 * pMP->GetWorldPos();
        pMP->SetWorldPos(pos);
    }
}

void alignAtlas(ORB_SLAM3::Atlas* atlas) {
    for(ORB_SLAM3::Map* map : atlas->GetAllMaps()) {
        alignMap(map);
    }
}


ORB_SLAM3::Atlas* prepare_atlas(std::string url, std::string strVocFile) {
    auto vocab = new ORB_SLAM3::ORBVocabulary();
    if (strVocFile.empty()) {
            strVocFile = ament_index_cpp::get_package_share_directory("orb_slam3") + "/vocab/ORBvoc.txt.bin";
    }
    bool bVocLoad = vocab->loadFromBinFile(strVocFile);
    if (!bVocLoad) {
        return nullptr;
    }
    auto mpKeyFrameDatabase = new ORB_SLAM3::KeyFrameDatabase(*vocab);
    ORB_SLAM3::Atlas *atlas = load_atlas_from_file(url);
    atlas->SetKeyFrameDababase(mpKeyFrameDatabase);
    atlas->SetORBVocabulary(vocab);
    atlas->PostLoad();
    atlas->PreSave();
    return atlas;
}

vector<ORB_SLAM3::Map*> merge_atlas(ORB_SLAM3::Atlas* atlas, std::string url) {
    // load the second atlas from the url, and merge it with the first atlas.
    // need to fix the keyframeIDs and map point IDs in the second atlas to avoid conflicts with the first atlas.
    //return all the new maps in the merged atlas.
    ORB_SLAM3::Atlas *new_atlas = load_atlas_from_file(url);
    auto vocab = atlas->GetORBVocabulary();
    auto mpKeyFrameDatabase = new ORB_SLAM3::KeyFrameDatabase(*vocab);
    new_atlas->SetKeyFrameDababase(mpKeyFrameDatabase);
    new_atlas->SetORBVocabulary(vocab);
    new_atlas->PostLoad();
    new_atlas->PreSave();
    auto new_maps = new_atlas->GetAllMaps();
    for(ORB_SLAM3::Map* map : new_maps) {
        alignMap(map);
    }
    atlas->ImportAtlas(new_atlas);
    return new_maps;
}


ORB_SLAM3::Map* get_biggest_map(ORB_SLAM3::Atlas* atlas) {
    ORB_SLAM3::Map *best_map;
    int map_size = 0;
    for (ORB_SLAM3::Map* map : atlas->GetAllMaps()) {
        if (map->GetAllKeyFrames().size() >= map_size) {
            best_map = map;
            map_size = map->GetAllKeyFrames().size();
        }
    }
    return best_map;
}


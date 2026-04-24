#include "common.h"
#include "Optimizer.h"
#include <rclcpp/serialization.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/string.hpp>

using std::placeholders::_1;
using namespace std;

void printAngles(ORB_SLAM3::Atlas* atlas) {
    auto kfs = atlas->GetAtlasKeyframes();
    for (auto kf : kfs) {
        ORB_SLAM3::VerboseStream(ORB_SLAM3::Verbose::VERBOSITY_NORMAL) << "KF id: " << kf.first << ": " << kf.second->GetPose().so3().unit_quaternion();
    }
}

float setNoise(ORB_SLAM3::Map* map, float noise) {
    auto kfs = map->GetAllKeyFrames();
    bool first = true;
    float first_noise = 0;
    for (auto kf : kfs) {
        if (first) {
            first_noise = kf->GetImuPoseNoise();
            first = false;
        } else {
            kf->SetImuPoseNoise(noise);
        }
    }
    return first_noise;
}

void save_atlas(ORB_SLAM3::Atlas* atlas, std::string url) {
    std::ofstream ofs(url, std::ios::binary);
    rclcpp::Serialization<orb_slam3::msg::Atlas> serializer;
    rclcpp::SerializedMessage serialized_msg;
    auto msg = create_atlas_msg(atlas, rclcpp::Time(0), std::string("world"));

    serializer.serialize_message(&msg, &serialized_msg);

    // Write raw CDR bytes to file
    std::ofstream file(url, std::ios::binary);
    auto & buf = serialized_msg.get_rcl_serialized_message();
    file.write(reinterpret_cast<const char *>(buf.buffer), buf.buffer_length);
    file.close();
}

void bundle_adjustment(ORB_SLAM3::Map* pMap) {
    ORB_SLAM3::Optimizer::GlobalBundleAdjustemnt(pMap);
    for (ORB_SLAM3::KeyFrame* pKF : pMap->GetAllKeyFrames()) {
        if (pKF->isBad()) continue;
        pKF->SetPose(pKF->mTcwGBA);
    }

    for (ORB_SLAM3::MapPoint* pMP : pMap->GetAllMapPoints()) {
        if (pMP->isBad()) continue;
        pMP->SetWorldPos(pMP->mPosGBA);
        pMP->UpdateNormalAndDepth();
    }

    pMap->InformNewBigChange();
    pMap->IncreaseChangeIndex();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
  
    auto node = rclcpp::Node::make_shared("process");
    auto logger = node->get_logger();
    string strFileVoc, strVocChecksum;
    string atlas_url;
    atlas_url = node->declare_parameter("atlas_url", "");
    if (atlas_url == "") {
        RCLCPP_ERROR(logger, "Please provide atlas_url in the launch file");
        rclcpp::shutdown();
        return 1;
    }
    std::string strVocFile = node->declare_parameter("voc_file", "file_not_set");
    auto mpVocabulary = new ORB_SLAM3::ORBVocabulary();
    bool bVocLoad = mpVocabulary->loadFromBinFile(strVocFile);
    if (!bVocLoad) {
        RCLCPP_ERROR(logger, "Wrong path to vocabulary.");
        RCLCPP_ERROR(logger, "Failed to open at: %s", strVocFile.c_str());
        rclcpp::shutdown();
        return 1;
    }
    auto mpKeyFrameDatabase = new ORB_SLAM3::KeyFrameDatabase(*mpVocabulary);
    ORB_SLAM3::Atlas* atlas = new ORB_SLAM3::Atlas();
    ORB_SLAM3::VerboseStream(ORB_SLAM3::Verbose::VERBOSITY_NORMAL) << "Starting to read the save text file " << atlas_url.c_str();
    std::ifstream ifs(atlas_url, std::ios::binary);
    if(!ifs.good())
    {
        ORB_SLAM3::VerboseStream(ORB_SLAM3::Verbose::VERBOSITY_QUIET) << "Load file not found";
        rclcpp::shutdown();
        return 1;
    }
    boost::archive::binary_iarchive ia(ifs);
    ia >> strFileVoc;
    ia >> strVocChecksum;
    ia >> atlas;
    ORB_SLAM3::VerboseStream(ORB_SLAM3::Verbose::VERBOSITY_NORMAL) << "End to load the save text file ";
    atlas->SetKeyFrameDababase(mpKeyFrameDatabase);
    ORB_SLAM3::VerboseStream(ORB_SLAM3::Verbose::VERBOSITY_NORMAL) << "DB Set ";
    atlas->SetORBVocabulary(mpVocabulary);
    ORB_SLAM3::VerboseStream(ORB_SLAM3::Verbose::VERBOSITY_NORMAL) << "Vocabulary Set ";
    atlas->PostLoad();
    atlas->PreSave();
    ORB_SLAM3::VerboseStream(ORB_SLAM3::Verbose::VERBOSITY_NORMAL) << "PostLoad done ";
    ORB_SLAM3::VerboseStream(ORB_SLAM3::Verbose::VERBOSITY_NORMAL) << "There are " << atlas->CountMaps() << " maps in the atlas";
    printAngles(atlas);
    ORB_SLAM3::Map *best_map;
    int map_size = 0;
    std::vector<ORB_SLAM3::Map*> maps = atlas->GetAllMaps();
    ORB_SLAM3::VerboseStream(ORB_SLAM3::Verbose::VERBOSITY_NORMAL) << "There are " << maps.size() << " maps in the atlas";
    for (ORB_SLAM3::Map* map : atlas->GetAllMaps()) {
        ORB_SLAM3::VerboseStream(ORB_SLAM3::Verbose::VERBOSITY_NORMAL) << "There are " << map->GetAllKeyFrames().size() << " keyframes in the map";
        if (map->GetAllKeyFrames().size() >= map_size) {
            best_map = map;
            map_size = map->GetAllKeyFrames().size();
        }
    }
    ORB_SLAM3::VerboseStream(ORB_SLAM3::Verbose::VERBOSITY_NORMAL) << "Biggest map is" << best_map;
    ORB_SLAM3::Optimizer::GlobalBundleAdjustemnt(best_map, 0);
    bundle_adjustment(best_map);
    float noise = setNoise(best_map,-1);
    bundle_adjustment(best_map);
    save_atlas(atlas, node->declare_parameter("atlas_save_url", "atlas_save.bin"));
    //run(node);
    rclcpp::shutdown();
    return 0;
}
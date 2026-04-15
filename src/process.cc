#include "common.h"
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/string.hpp>

using std::placeholders::_1;
using namespace std;

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
    atlas->PostLoad();
    auto kfs = atlas->GetAllKeyFrames();
    ORB_SLAM3::VerboseStream(ORB_SLAM3::Verbose::VERBOSITY_NORMAL) << "There are " << kfs.size() << " keyframes in the atlas";
    for (auto kf : kfs) {
        ORB_SLAM3::VerboseStream(ORB_SLAM3::Verbose::VERBOSITY_NORMAL) << "KF id: " << kf->mnId;
    }
    run(node);
    rclcpp::shutdown();
    return 0;
}
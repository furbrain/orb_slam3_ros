#include "common.h"
#include "Optimizer.h"
#include <rclcpp/serialization.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/string.hpp>

#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

using std::placeholders::_1;
using namespace std;

void printAngles(ORB_SLAM3::Atlas* atlas) {
    auto kfs = atlas->GetAtlasKeyframes();
    for (auto kf : kfs) {
        float angle = kf.second->GetPose().so3().inverse().unit_quaternion().angularDistance(kf.second->GetPoseFromEstimate().so3().inverse().unit_quaternion());
        auto q1 = kf.second->GetPose().so3().inverse().unit_quaternion();
        auto q2 = kf.second->GetPoseFromEstimate().so3().inverse().unit_quaternion();
        auto z_world = q1 * Eigen::Vector3f::UnitZ();
        auto z_world2 = q2 * Eigen::Vector3f::UnitZ();
        // Inclination: angle above/below horizontal plane (-90 to +90 degrees)
        float inclination = std::asin(z_world.z()) * 180.0f / M_PI;

        // Bearing: angle in horizontal plane, measured clockwise from +X (or swap to from +Y for north)
        float bearing = std::atan2(z_world.y(), z_world.x()) * 180.0f / M_PI;
        float bearing2 = std::atan2(z_world2.y(), z_world2.x()) * 180.0f / M_PI;
        ORB_SLAM3::VerboseStream(ORB_SLAM3::Verbose::VERBOSITY_NORMAL) << "KF id: " << kf.first << ", " << angle * 180.0 / M_PI << " , " << inclination << " , " << bearing << " , " << bearing2;
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



void save_atlas(ORB_SLAM3::Atlas* atlas, std::string url) {
    ORB_SLAM3::VerboseStream(ORB_SLAM3::Verbose::VERBOSITY_NORMAL) << "Saving atlas to " << url.c_str();
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

void bundle_adjustment(ORB_SLAM3::Map* pMap, int num_iterations) {
    ORB_SLAM3::Optimizer::GlobalBundleAdjustemnt(pMap, num_iterations);
    // for (ORB_SLAM3::KeyFrame* pKF : pMap->GetAllKeyFrames()) {
    //     if (pKF->isBad()) continue;
    //     std::cout << pKF->mTcwGBA.so3().unit_quaternion() << std::endl;
    //     pKF->SetPose(pKF->mTcwGBA);
    // }

    // for (ORB_SLAM3::MapPoint* pMP : pMap->GetAllMapPoints()) {
    //     if (pMP->isBad()) continue;
    //     pMP->SetWorldPos(pMP->mPosGBA);
    //     pMP->UpdateNormalAndDepth();
    // }

    pMap->InformNewBigChange();
    pMap->IncreaseChangeIndex();
}

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

ORB_SLAM3::Atlas* prepare_atlas(std::string url, ORB_SLAM3::ORBVocabulary* mpVocabulary, ORB_SLAM3::KeyFrameDatabase* mpKeyFrameDatabase) {
    ORB_SLAM3::VerboseStream(ORB_SLAM3::Verbose::VERBOSITY_NORMAL) << "Loading atlas from " << url.c_str();
    ORB_SLAM3::Atlas *atlas = load_atlas_from_file(url);
    atlas->SetKeyFrameDababase(mpKeyFrameDatabase);
    atlas->SetORBVocabulary(mpVocabulary);
    atlas->PostLoad();
    atlas->PreSave();
    ORB_SLAM3::VerboseStream(ORB_SLAM3::Verbose::VERBOSITY_NORMAL) << "There are " << atlas->CountMaps() << " maps in the atlas";
    for (ORB_SLAM3::Map* map : atlas->GetAllMaps()) {
        alignMap(map);
    }
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


void run_full_merge(ORB_SLAM3::Atlas* atlas, vector<ORB_SLAM3::Map*> new_maps) {
    mpTracker = new ORB_SLAM3::Tracking(nullptr, atlas->GetORBVocabulary(), nullptr, atlas,
                            atlas->GetKeyFrameDatabase(), strSettingsFile, mSensor, settings_, strSequence);
    mpLocalMapper = new ORB_SLAM3::LocalMapping(this, atlas, m
                             mpAtlas, mpKeyFrameDatabase, strSettingsFile, mSensor, settings_, strSequence);

    //Initialize the Local Mapping thread and launch
    mpLocalMapper = new LocalMapping(this, mpAtlas, mSensor==MONOCULAR || mSensor==IMU_MONOCULAR,
                                     (mSensor==IMU_MONOCULAR || mSensor==IMU_STEREO || mSensor==IMU_RGBD) && settings_->useImuTrajectory(), strSequence);

    for (ORB_SLAM3::Map* map : new_maps) {
        alignMap(map);
        bundle_adjustment(map, 100);
    }


rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub;
rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_imu_pub;
ORB_SLAM3::Atlas* atlas;
ORB_SLAM3::Map *best_map;
std::vector<Sophus::SE3f> cam_poses;
std::vector<Sophus::SE3f> cam_poses_from_imu;

visualization_msgs::msg::MarkerArray makeCameraMarkerArray(
  const std::vector<Sophus::SE3f> & poses,
  const std_msgs::msg::ColorRGBA & colour,
  const rclcpp::Time & stamp,
  const std::string & ns = "camera_poses",
  bool delete_old = true,
  float size = 0.1f,
  float line_width = 0.01f
  )
{
  // A simple camera glyph in local frame: a box outline + an apex indicating
  // the viewing direction (+Z). All units in metres, scaled by `size`.
  //
  //      1-----2
  //     /|    /|
  //    / |   / |    apex(0) sits in front, lines radiate to the 4 box corners
  //   /  5--/--6
  //  3-----4  /
  //  |  /  | /
  //  | /   |/
  //  7-----8
  //
  // Box corners (local frame, centred slightly behind origin)
  const float h = size * 0.5f;
  const float d = size * 0.8f;   // depth of the box body behind origin

  const std::array<Eigen::Vector3f, 9> pts = {{
    { 0.0f,  0.0f,  0.0f},   // 0 - apex (viewing direction tip)
    {-h,    -h,     d     },  // 1
    { h,    -h,     d     },  // 2
    { h,     h,     d     },  // 3
    {-h,     h,     d     },  // 4
  }};

  constexpr std::array<std::pair<int,int>, 8> edges = {{
    {0,1},{0,2},{0,3},{0,4},  // apex to box corners
    {1,2},{2,3},{3,4},{4,1}   // box rectangle
  }};

  auto toPoint = [](const Eigen::Vector3f & v) {
    geometry_msgs::msg::Point p;
    p.x = v.x(); p.y = v.y(); p.z = v.z();
    return p;
  };

  std_msgs::msg::Header header;
  header.frame_id = world_frame_id;
  header.stamp    = stamp;

  visualization_msgs::msg::MarkerArray out;

  if (delete_old) {
    visualization_msgs::msg::Marker clear;
    clear.header = header;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    out.markers.push_back(clear);
  }

  for (size_t i = 0; i < poses.size(); ++i)
  {
    const Eigen::Vector3f    t = poses[i].translation();
    const Eigen::Quaternionf q(poses[i].rotationMatrix());

    visualization_msgs::msg::Marker m;
    m.header  = header;
    m.ns      = "camera_poses";
    m.id      = static_cast<int>(i);
    m.type    = visualization_msgs::msg::Marker::LINE_LIST;
    m.action  = visualization_msgs::msg::Marker::ADD;
    m.scale.x = line_width;
    m.color   = colour;

    m.pose.position.x    = t.x();
    m.pose.position.y    = t.y();
    m.pose.position.z    = t.z();
    m.pose.orientation.w = q.w();
    m.pose.orientation.x = q.x();
    m.pose.orientation.y = q.y();
    m.pose.orientation.z = q.z();

    for (const auto & [a, b] : edges)
    {
      m.points.push_back(toPoint(pts[a]));
      m.points.push_back(toPoint(pts[b]));
    }

    out.markers.push_back(m);
  }

  return out;
}


void timer_callback() {
  rclcpp::Time msg_time = rclcpp::Clock().now();
  publish_all_points(best_map->GetAllMapPoints(), msg_time);
  std_msgs::msg::ColorRGBA cam_colour = std_msgs::build<std_msgs::msg::ColorRGBA>().r(0.0f).g(1.0f).b(0.0f).a(1.0f);
  std_msgs::msg::ColorRGBA imu_colour = std_msgs::build<std_msgs::msg::ColorRGBA>().r(1.0f).g(0.0f).b(0.0f).a(1.0f);
  auto pose_arr = makeCameraMarkerArray(cam_poses, cam_colour, msg_time, "cam_poses", false);
  marker_array_pub->publish(pose_arr);
  auto pose_arr_imu = makeCameraMarkerArray(cam_poses_from_imu, imu_colour, msg_time, "imu_poses", false);
  marker_array_imu_pub->publish(pose_arr_imu);
}

void fill_poses(ORB_SLAM3::Map* map) {
    cam_poses.clear();
    cam_poses_from_imu.clear();
    for (ORB_SLAM3::KeyFrame* pKF : map->GetAllKeyFrames()) {
        if (pKF->isBad()) continue;
        auto cam_pose = pKF->GetPose().inverse();
        auto imu_pose = pKF->GetPoseFromEstimate().inverse();
        imu_pose.translation() = cam_pose.translation(); // use the same translation for better comparison of rotation
        cam_poses.push_back(cam_pose);
        cam_poses_from_imu.push_back(imu_pose);
    }
}

void reset_orientations(ORB_SLAM3::Map* map) {
    for (ORB_SLAM3::KeyFrame* pKF : map->GetAllKeyFrames()) {
        if (pKF->isBad()) continue;
        auto pose = pKF->GetPose().inverse();
        Eigen::Vector3f t = pose.translation();
        pose.so3() = pKF->GetPoseFromEstimate().inverse().so3(); // set to madgwick rotation
        pose.translation() = t; // keep original translation
        pKF->SetPose(pose.inverse());
    }
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
  
    auto node = rclcpp::Node::make_shared("process");
    setup_publishers(node, "process");
    marker_array_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>("marker_array", rclcpp::QoS(10));
    marker_array_imu_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>("marker_array_imu", rclcpp::QoS(10));
    auto logger = node->get_logger();
    string strFileVoc, strVocChecksum;
    string atlas_url, merge_url;
    world_frame_id = "map";
    atlas_url = node->declare_parameter("atlas_url", "");
    merge_url = node->declare_parameter("merge_url", "");
    int num_iterations = node->declare_parameter("num_iterations", 5);
    float new_noise = node->declare_parameter("new_noise", -1.0);
    bool reset_orientation = node->declare_parameter("reset_orientations", true);
    if (atlas_url == "") {
        RCLCPP_ERROR(logger, "Please provide atlas_url in the launch file");
        rclcpp::shutdown();
        return 1;
    }
    auto default_voc_location = ament_index_cpp::get_package_share_directory("orb_slam3") + "/vocab/ORBvoc.txt.bin";
    std::string strVocFile = node->declare_parameter("voc_file", default_voc_location);
    auto mpVocabulary = new ORB_SLAM3::ORBVocabulary();
    bool bVocLoad = mpVocabulary->loadFromBinFile(strVocFile);
    if (!bVocLoad) {
        RCLCPP_ERROR(logger, "Wrong path to vocabulary.");
        RCLCPP_ERROR(logger, "Failed to open at: %s", strVocFile.c_str());
        rclcpp::shutdown();
        return 1;
    }
    auto mpKeyFrameDatabase = new ORB_SLAM3::KeyFrameDatabase(*mpVocabulary);
    atlas = prepare_atlas(atlas_url, mpVocabulary, mpKeyFrameDatabase);
    best_map = get_biggest_map(atlas);
    atlas->ChangeMap(best_map);
    auto new_maps = merge_atlas(atlas, merge_url);
    ORB_SLAM3::VerboseStream(ORB_SLAM3::Verbose::VERBOSITY_NORMAL) << "There are " << atlas->CountMaps() << " maps in the atlas after merge";
    // DO THE MAIN MERGE HERE...
    //float noise = setNoise(best_map,new_noise);
    //printAngles(atlas);
    //ORB_SLAM3::VerboseStream(ORB_SLAM3::Verbose::VERBOSITY_NORMAL) << "Setting noise to " << new_noise << " (was " << noise << ")";   
    // if (num_iterations > 0) {
    //     ORB_SLAM3::VerboseStream(ORB_SLAM3::Verbose::VERBOSITY_NORMAL) << "Running bundle adjustment with " << num_iterations << " iterations";
    //     bundle_adjustment(best_map, num_iterations);
    //     printAngles(atlas);
    // }
    //save_atlas(atlas, node->declare_parameter("atlas_save_url", "atlas_save.msg"));
    // auto timer = node->create_wall_timer(std::chrono::milliseconds(1000), timer_callback);
    fill_poses(best_map);
    run(node);
    rclcpp::shutdown();
    return 0;
}

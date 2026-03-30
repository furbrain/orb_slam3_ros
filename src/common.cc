/**
 *
 * Common functions and variables across all modes (mono/stereo, with or w/o
 * imu)
 *
 */
#include "common.h"
#include "System.h"
#include <orb_slam3/msg/atlas.hpp> // This file is created automatically, see here http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv#Creating_a_srv
#include <orb_slam3/msg/num_points.hpp>
#include <orb_slam3/msg/state.hpp>

// Variables for ORB-SLAM3
ORB_SLAM3::System *pSLAM;
ORB_SLAM3::System::eSensor sensor_type = ORB_SLAM3::System::NOT_SET;

// Variables for ROS
std::string world_frame_id, cam_frame_id, imu_frame_id;
std::string lost_images_path;
rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr kf_markers_pub;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    tracked_mappoints_pub,
    tracked_keypoints_pub, all_mappoints_pub;
rclcpp::Publisher<orb_slam3::msg::Atlas>::SharedPtr atlas_pub;
rclcpp::Publisher<orb_slam3::msg::NumPoints>::SharedPtr num_points_pub;
rclcpp::Publisher<orb_slam3::msg::State>::SharedPtr state_pub;
image_transport::Publisher tracking_img_pub, kf_pub;
std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

//////////////////////////////////////////////////
// Main functions
//////////////////////////////////////////////////

void save_map_srv(const std::shared_ptr<orb_slam3::srv::SaveMap::Request> req,
                  std::shared_ptr<orb_slam3::srv::SaveMap::Response> res) {
  res->success = pSLAM->SaveMap(req->name);

  if (res->success)
    RCLCPP_INFO(rclcpp::get_logger(""), "Map was saved as %s.osa",
                req->name.c_str());
  else
    RCLCPP_ERROR(rclcpp::get_logger(""), "Map could not be saved.");

  return;
}

void save_colmap_srv(const orb_slam3::srv::SaveMap::Request::SharedPtr req,
                     orb_slam3::srv::SaveMap::Response::SharedPtr res) {
  res->success = pSLAM->SaveCOLMAP(req->name);

  if (res->success)
    RCLCPP_INFO(rclcpp::get_logger(""), "COLMAP was saved as %s.osa",
                req->name.c_str());
  else
    RCLCPP_ERROR(rclcpp::get_logger(""), "COLMAP could not be saved.");

  return;
}

void save_traj_srv(const orb_slam3::srv::SaveMap::Request::SharedPtr req,
                   orb_slam3::srv::SaveMap::Response::SharedPtr res) {
  const string cam_traj_file = req->name + "_cam_traj.txt";
  const string kf_traj_file = req->name + "_kf_traj.txt";

  try {
    pSLAM->SaveTrajectoryEuRoC(cam_traj_file);
    pSLAM->SaveKeyFrameTrajectoryEuRoC(kf_traj_file);
    res->success = true;
  } catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
    res->success = false;
  } catch (...) {
    std::cerr << "Unknows exeption" << std::endl;
    res->success = false;
  }

  if (!res->success)
    RCLCPP_ERROR(rclcpp::get_logger(""),
                 "Estimated trajectory could not be saved.");

  return;
}

void setup_services(rclcpp::Node::SharedPtr node, std::string node_name) {
  static rclcpp::Service<orb_slam3::srv::SaveMap>::SharedPtr save_map_service =
      node->create_service<orb_slam3::srv::SaveMap>(node_name + "/save_map",
                                                    &save_map_srv);
  static rclcpp::Service<orb_slam3::srv::SaveMap>::SharedPtr
      save_colmap_service = node->create_service<orb_slam3::srv::SaveMap>(
          node_name + "/save_colmap", &save_colmap_srv);
  static rclcpp::Service<orb_slam3::srv::SaveMap>::SharedPtr save_traj_service =
      node->create_service<orb_slam3::srv::SaveMap>(node_name + "/save_traj",
                                                    &save_traj_srv);
}

void setup_clients(rclcpp::Node::SharedPtr node, std::string node_name) {
  // Currently no clients are needed

}

void setup_publishers(rclcpp::Node::SharedPtr node, std::string node_name) {
  static image_transport::ImageTransport image_transport(node);

  pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
      node_name + "/camera_pose", 1);

  tracked_mappoints_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>(
      node_name + "/tracked_points", 1);

  tracked_keypoints_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>(
      node_name + "/tracked_key_points", 1);

  all_mappoints_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>(
      node_name + "/all_points", 1);

  tracking_img_pub =
      image_transport.advertise(node_name + "/tracking_image", 1);

  kf_markers_pub = node->create_publisher<visualization_msgs::msg::Marker>(
      node_name + "/kf_markers", 1000);

  kf_pub = image_transport.advertise(node_name + "/keyframes", 1);

  atlas_pub =
      node->create_publisher<orb_slam3::msg::Atlas>(node_name + "/atlas", 3);

  if (sensor_type == ORB_SLAM3::System::IMU_MONOCULAR ||
      sensor_type == ORB_SLAM3::System::IMU_STEREO ||
      sensor_type == ORB_SLAM3::System::IMU_RGBD) {
    odom_pub = node->create_publisher<nav_msgs::msg::Odometry>(
        node_name + "/body_odom", 1);
  }
  num_points_pub = node->create_publisher<orb_slam3::msg::NumPoints>(
      node_name + "/num_points", 1);
  tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*node);

  state_pub = node->create_publisher<orb_slam3::msg::State>(
      node_name + "/state", 1);
}

void publish_topics(rclcpp::Time msg_time, Eigen::Vector3f Wbb) {
  Sophus::SE3f Twc = pSLAM->GetCamTwc();

  if (Twc.translation().array().isNaN()[0] ||
      Twc.rotationMatrix().array().isNaN()(0, 0)) // avoid publishing NaN
    return;

  // Common topics
  publish_camera_pose(Twc, msg_time);
  publish_tf_transform(Twc, world_frame_id, cam_frame_id, msg_time);

  publish_tracking_img(pSLAM->GetCurrentFrame(), msg_time);

  auto state = pSLAM->GetTrackingState();
  auto state_msg = orb_slam3::msg::State();
  if (state == ORB_SLAM3::Tracking::RECENTLY_LOST &&
      !lost_images_path.empty()) {
    ORB_SLAM3::Verbose::PrintMess(
        "Saving lost image " + std::to_string(msg_time.nanoseconds()) + ".png",
        ORB_SLAM3::Verbose::VERBOSITY_NORMAL);
    std::string img_name =
        lost_images_path + std::to_string(msg_time.nanoseconds()) + ".png";
    cv::imwrite(img_name, pSLAM->GetCurrentFrame());
  }
  switch (state) {
  case ORB_SLAM3::Tracking::OK:
    state_msg.state = orb_slam3::msg::State::OK;
    break;  
  case ORB_SLAM3::Tracking::RECENTLY_LOST:
    state_msg.state = orb_slam3::msg::State::RECENTLY_LOST;
    break;
  case ORB_SLAM3::Tracking::LOST:
    state_msg.state = orb_slam3::msg::State::LOST;
    break;
  default:
    state_msg.state = orb_slam3::msg::State::OTHER;
    break;
  }
  state_pub->publish(state_msg);

  publish_keypoints(pSLAM->GetTrackedMapPoints(), pSLAM->GetTrackedKeyPoints(),
                    msg_time);

  publish_tracked_points(pSLAM->GetTrackedMapPoints(), msg_time);
  publish_all_points(pSLAM->GetAllMapPoints(), msg_time);
  publish_kf_markers(pSLAM->GetAllKeyframePoses(), msg_time);

  // IMU-specific topics
  if (sensor_type == ORB_SLAM3::System::IMU_MONOCULAR ||
      sensor_type == ORB_SLAM3::System::IMU_STEREO ||
      sensor_type == ORB_SLAM3::System::IMU_RGBD) {
    // Body pose and translational velocity can be obtained from ORB-SLAM3
    Sophus::SE3f Twb = pSLAM->GetImuTwb();
    Eigen::Vector3f Vwb = pSLAM->GetImuVwb();

    // IMU provides body angular velocity in body frame (Wbb) which is
    // transformed to world frame (Wwb)
    Sophus::Matrix3f Rwb = Twb.rotationMatrix();
    Eigen::Vector3f Wwb = Rwb * Wbb;

    publish_tf_transform(Twb, world_frame_id, imu_frame_id, msg_time);
    publish_body_odom(Twb, Vwb, Wwb, msg_time);
  }
}

void publish_body_odom(Sophus::SE3f Twb_SE3f, Eigen::Vector3f Vwb_E3f,
                       Eigen::Vector3f ang_vel_body, rclcpp::Time msg_time) {
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.child_frame_id = imu_frame_id;
  odom_msg.header.frame_id = world_frame_id;
  odom_msg.header.stamp = msg_time;

  odom_msg.pose.pose.position.x = Twb_SE3f.translation().x();
  odom_msg.pose.pose.position.y = Twb_SE3f.translation().y();
  odom_msg.pose.pose.position.z = Twb_SE3f.translation().z();

  odom_msg.pose.pose.orientation.w = Twb_SE3f.unit_quaternion().coeffs().w();
  odom_msg.pose.pose.orientation.x = Twb_SE3f.unit_quaternion().coeffs().x();
  odom_msg.pose.pose.orientation.y = Twb_SE3f.unit_quaternion().coeffs().y();
  odom_msg.pose.pose.orientation.z = Twb_SE3f.unit_quaternion().coeffs().z();

  odom_msg.twist.twist.linear.x = Vwb_E3f.x();
  odom_msg.twist.twist.linear.y = Vwb_E3f.y();
  odom_msg.twist.twist.linear.z = Vwb_E3f.z();

  odom_msg.twist.twist.angular.x = ang_vel_body.x();
  odom_msg.twist.twist.angular.y = ang_vel_body.y();
  odom_msg.twist.twist.angular.z = ang_vel_body.z();

  odom_pub->publish(odom_msg);
}

void publish_camera_pose(Sophus::SE3f Tcw_SE3f, rclcpp::Time msg_time) {
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.frame_id = world_frame_id;
  pose_msg.header.stamp = msg_time;

  pose_msg.pose.position.x = Tcw_SE3f.translation().x();
  pose_msg.pose.position.y = Tcw_SE3f.translation().y();
  pose_msg.pose.position.z = Tcw_SE3f.translation().z();

  pose_msg.pose.orientation.w = Tcw_SE3f.unit_quaternion().coeffs().w();
  pose_msg.pose.orientation.x = Tcw_SE3f.unit_quaternion().coeffs().x();
  pose_msg.pose.orientation.y = Tcw_SE3f.unit_quaternion().coeffs().y();
  pose_msg.pose.orientation.z = Tcw_SE3f.unit_quaternion().coeffs().z();

  pose_pub->publish(pose_msg);
}

void publish_tf_transform(Sophus::SE3f T_SE3f, string frame_id,
                          string child_frame_id, rclcpp::Time msg_time) {
  geometry_msgs::msg::TransformStamped t;
  t.transform = SE3f_to_tfTransform(T_SE3f);
  t.header.stamp = msg_time;
  t.header.frame_id = frame_id;
  t.child_frame_id = child_frame_id;
  tf_broadcaster->sendTransform(t);
}

void publish_tracking_img(cv::Mat image, rclcpp::Time msg_time) {
  std_msgs::msg::Header header;

  header.stamp = msg_time;

  header.frame_id = world_frame_id;

  const sensor_msgs::msg::Image::SharedPtr rendered_image_msg =
      cv_bridge::CvImage(header, "bgr8", image).toImageMsg();

  tracking_img_pub.publish(rendered_image_msg);
}

void publish_keypoints(std::vector<ORB_SLAM3::MapPoint *> tracked_map_points,
                       std::vector<cv::KeyPoint> tracked_keypoints,
                       rclcpp::Time msg_time) {
  std::vector<cv::KeyPoint> finalKeypoints;

  if (tracked_keypoints.empty())
    return;

  for (size_t i = 0; i < tracked_map_points.size(); i++) {
    if (tracked_map_points[i]) { // if the MapPoint pointer is not nullptr
      finalKeypoints.push_back(tracked_keypoints[i]);
    }
  }

  // Create a blank image. Adjust dimensions as per your requirement.
  // int width = 640;  // Assuming a standard 640x480 size. Change as needed.
  // int height = 480;
  // cv::Mat blankImg = cv::Mat::zeros(height, width, CV_8UC3);  // Black image

  // Draw keypoints on the blank image.
  // cv::drawKeypoints(blankImg, finalKeypoints, blankImg, cv::Scalar(0, 255,
  // 0), cv::DrawMatchesFlags::DEFAULT);

  // Display the image (optional)
  // cv::imshow("Keypoints", blankImg);
  // cv::waitKey(1);

  sensor_msgs::msg::PointCloud2 cloud =
      keypoints_to_pointcloud(finalKeypoints, msg_time);

  tracked_keypoints_pub->publish(cloud);
}

void publish_tracked_points(std::vector<ORB_SLAM3::MapPoint *> tracked_points,
                            rclcpp::Time msg_time) {
  sensor_msgs::msg::PointCloud2 cloud =
      mappoint_to_pointcloud(tracked_points, msg_time);
  orb_slam3::msg::NumPoints num_points_msg;
  num_points_msg.count =
      std::count_if(tracked_points.begin(), tracked_points.end(),
                    [](ORB_SLAM3::MapPoint *ptr) { return ptr != nullptr; });
  num_points_pub->publish(num_points_msg);
  tracked_mappoints_pub->publish(cloud);
}

void publish_all_points(std::vector<ORB_SLAM3::MapPoint *> map_points,
                        rclcpp::Time msg_time) {
  sensor_msgs::msg::PointCloud2 cloud =
      mappoint_to_pointcloud(map_points, msg_time);

  all_mappoints_pub->publish(cloud);
}

// More details:
// http://docs.ros.org/en/api/visualization_msgs/html/msg/Marker.html
void publish_kf_markers(std::vector<Sophus::SE3f> vKFposes,
                        rclcpp::Time msg_time) {
  int numKFs = vKFposes.size();
  if (numKFs == 0)
    return;

  visualization_msgs::msg::Marker kf_markers;
  kf_markers.header.frame_id = world_frame_id;
  kf_markers.ns = "kf_markers";
  kf_markers.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  kf_markers.action = visualization_msgs::msg::Marker::ADD;
  kf_markers.pose.orientation.w = 1.0;
  kf_markers.lifetime = rclcpp::Duration::from_nanoseconds(0);

  kf_markers.id = 0;
  kf_markers.scale.x = 0.05;
  kf_markers.scale.y = 0.05;
  kf_markers.scale.z = 0.05;
  kf_markers.color.g = 1.0;
  kf_markers.color.a = 1.0;

  for (int i = 0; i <= numKFs; i++) {
    geometry_msgs::msg::Point kf_marker;
    kf_marker.x = vKFposes[i].translation().x();
    kf_marker.y = vKFposes[i].translation().y();
    kf_marker.z = vKFposes[i].translation().z();
    kf_markers.points.push_back(kf_marker);
  }

  kf_markers_pub->publish(kf_markers);
}

void publish_kf(cv::Mat image, rclcpp::Time msg_time) {
  std_msgs::msg::Header header;
  header.stamp = msg_time;
  header.frame_id = world_frame_id;
  const sensor_msgs::msg::Image::SharedPtr rendered_image_msg =
      cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
  kf_pub.publish(rendered_image_msg);
}

void publish_atlas(ORB_SLAM3::Atlas *atlas, rclcpp::Time msg_time) {
  std_msgs::msg::Header header;
  header.stamp = msg_time;
  header.frame_id = world_frame_id;

  orb_slam3::msg::Atlas atlas_msg;
  atlas_msg.header = header;
  auto vpMaps = atlas->GetAllMaps();
  std::set<decltype(ORB_SLAM3::MapPoint::mnId)> sPoints;
  for (ORB_SLAM3::Map *pMap : vpMaps) {
    orb_slam3::msg::Map map_msg;
    for (ORB_SLAM3::KeyFrame *pKF : pMap->GetAllKeyFrames()) {
      orb_slam3::msg::KeyFrame kf_msg;
      Sophus::SE3f Twb = pKF->GetPose();
      Eigen::Quaternionf q = Twb.unit_quaternion();
      Sophus::Vector3f tr = Twb.translation();
      kf_msg.id = pKF->mnId;
      kf_msg.stamp = rclcpp::Time(int64(pKF->mTimeStamp * 1e9));
      kf_msg.pose.orientation.w = q.w();
      kf_msg.pose.orientation.x = q.x();
      kf_msg.pose.orientation.y = q.y();
      kf_msg.pose.orientation.z = q.z();
      kf_msg.pose.position.x = tr.x();
      kf_msg.pose.position.y = tr.y();
      kf_msg.pose.position.z = tr.z();
      for (size_t i = 0; i < pKF->mvKeys.size(); i++) {
        cv::KeyPoint kp = pKF->mvKeys[i];
        orb_slam3::msg::KeyPoint kp_msg;
        auto mp = pKF->GetMapPoint(i);
        if (mp) {
          kp_msg.point3d_id = mp->mnId;
          kp_msg.x = kp.pt.x;
          kp_msg.y = kp.pt.y;
          kf_msg.points.push_back(kp_msg);
          if (sPoints.insert(mp->mnId).second) // insert returns second part
                                               // true if actually inserted
          { // therefore we need to add this point to the main points list
            auto pos = mp->GetWorldPos();
            orb_slam3::msg::Point3D pt_msg;
            pt_msg.x = pos.x();
            pt_msg.y = pos.y();
            pt_msg.z = pos.z();
            pt_msg.id = mp->mnId;
            atlas_msg.points.push_back(pt_msg);
          }
        }
      }
      map_msg.frames.push_back(kf_msg);
    }
    atlas_msg.maps.push_back(map_msg);
  }
  atlas_pub->publish(atlas_msg);
}

//////////////////////////////////////////////////
// Miscellaneous functions
//////////////////////////////////////////////////

sensor_msgs::msg::PointCloud2
keypoints_to_pointcloud(std::vector<cv::KeyPoint> &keypoints,
                        rclcpp::Time msg_time) {
  const int num_channels = 3; // x y z

  sensor_msgs::msg::PointCloud2 cloud;

  cloud.header.stamp = msg_time;
  cloud.header.frame_id = world_frame_id;
  cloud.height = 1;
  cloud.width = keypoints.size();
  cloud.is_bigendian = false;
  cloud.is_dense = true;
  cloud.point_step = num_channels * sizeof(float);
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.fields.resize(num_channels);

  std::string channel_id[] = {"x", "y", "z"};

  for (int i = 0; i < num_channels; i++) {
    cloud.fields[i].name = channel_id[i];
    cloud.fields[i].offset = i * sizeof(float);
    cloud.fields[i].count = 1;
    cloud.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
  }

  cloud.data.resize(cloud.row_step * cloud.height);

  unsigned char *cloud_data_ptr = &(cloud.data[0]);

  for (unsigned int i = 0; i < cloud.width; i++) {
    float data_array[num_channels] = {
        keypoints[i].pt.x, keypoints[i].pt.y,
        0.0f // Z value is 0 for 2D keypoints
    };

    memcpy(cloud_data_ptr + (i * cloud.point_step), data_array,
           num_channels * sizeof(float));
  }
  return cloud;
}

sensor_msgs::msg::PointCloud2
mappoint_to_pointcloud(std::vector<ORB_SLAM3::MapPoint *> map_points,
                       rclcpp::Time msg_time) {
  const int num_channels = 3; // x y z

  if (map_points.size() == 0) {
    std::cout << "Map point vector is empty!" << std::endl;
  }

  sensor_msgs::msg::PointCloud2 cloud;

  cloud.header.stamp = msg_time;
  cloud.header.frame_id = world_frame_id;
  cloud.height = 1;
  cloud.width = map_points.size();
  cloud.is_bigendian = false;
  cloud.is_dense = true;
  cloud.point_step = num_channels * sizeof(float);
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.fields.resize(num_channels);

  std::string channel_id[] = {"x", "y", "z"};

  for (int i = 0; i < num_channels; i++) {
    cloud.fields[i].name = channel_id[i];
    cloud.fields[i].offset = i * sizeof(float);
    cloud.fields[i].count = 1;
    cloud.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
  }

  cloud.data.resize(cloud.row_step * cloud.height);

  unsigned char *cloud_data_ptr = &(cloud.data[0]);

  for (unsigned int i = 0; i < cloud.width; i++) {
    if (map_points[i]) {
      Eigen::Vector3f P3Dw = map_points[i]->GetWorldPos().cast<float>();

      float data_array[num_channels] = {P3Dw.x(), P3Dw.y(), P3Dw.z()};

      memcpy(cloud_data_ptr + (i * cloud.point_step), data_array,
             num_channels * sizeof(float));
    }
  }
  return cloud;
}

cv::Mat SE3f_to_cvMat(Sophus::SE3f T_SE3f) {
  cv::Mat T_cvmat;

  Eigen::Matrix4f T_Eig3f = T_SE3f.matrix();
  cv::eigen2cv(T_Eig3f, T_cvmat);

  return T_cvmat;
}

geometry_msgs::msg::Transform SE3f_to_tfTransform(Sophus::SE3f T_SE3f) {
  geometry_msgs::msg::Transform t;
  auto quat = T_SE3f.so3().unit_quaternion();
  auto tf = T_SE3f.translation();
  t.rotation.x = quat.x();
  t.rotation.y = quat.y();
  t.rotation.z = quat.z();
  t.rotation.w = quat.w();
  t.translation.x = tf.x();
  t.translation.y = tf.y();
  t.translation.z = tf.z();
  return t;
}

void ROS2Printer(const std::string &msg, ORB_SLAM3::Verbose::eLevel lev) {
  auto logger = rclcpp::get_logger("ORB");
  switch (lev) {
  case ORB_SLAM3::Verbose::VERBOSITY_QUIET:
    RCLCPP_WARN(logger, "%s", msg.c_str());
    break;
  case ORB_SLAM3::Verbose::VERBOSITY_NORMAL:
    RCLCPP_INFO(logger, "%s", msg.c_str());
    break;
  case ORB_SLAM3::Verbose::VERBOSITY_DEBUG:
  default:
    RCLCPP_DEBUG(logger, "%s", msg.c_str());
    break;
  }
}

rclcpp::Node::SharedPtr init(int argc, char **argv, std::string name,
                             ORB_SLAM3::System::eSensor sensor) {
  rclcpp::init(argc, argv);
  ORB_SLAM3::Verbose::customPrint = ROS2Printer;
  auto node = rclcpp::Node::make_shared(name);
  auto logger = node->get_logger();
  if (argc > 1) {
    RCLCPP_WARN(logger, "Arguments supplied via command line are ignored.");
  }

  std::string voc_file = node->declare_parameter("voc_file", "file_not_set");
  std::string settings_file =
      node->declare_parameter("settings_file", "file_not_set");

  if (voc_file == "file_not_set" || settings_file == "file_not_set") {
    RCLCPP_ERROR(
        logger, "Please provide voc_file and settings_file in the launch file");
    rclcpp::shutdown();
    return NULL;
  }

  world_frame_id = node->declare_parameter("world_frame_id", "map");
  cam_frame_id = node->declare_parameter("cam_frame_id", "camera");
  lost_images_path = node->declare_parameter("lost_images_path", "");

  sensor_type = sensor;
  if (sensor_type == ORB_SLAM3::System::IMU_MONOCULAR ||
      sensor_type == ORB_SLAM3::System::IMU_STEREO ||
      sensor_type == ORB_SLAM3::System::IMU_RGBD) {
    imu_frame_id = node->declare_parameter("imu_frame_id", "imu");
  }
  // Create SLAM system. It initializes all system threads and gets ready to
  // process frames.
  pSLAM = new ORB_SLAM3::System(voc_file, settings_file, sensor_type);
  RCLCPP_INFO(logger, "pSLAM created");
  return node;
}

void run(rclcpp::Node::SharedPtr node) {
  auto logger = node->get_logger();
  RCLCPP_INFO(logger, "Spinning");
  rclcpp::spin(node);
  RCLCPP_INFO(logger, "Finished spin");

  // Stop all threads
  pSLAM->Shutdown();
  RCLCPP_INFO(logger, "mid shutdwon");
  rclcpp::shutdown();
  RCLCPP_INFO(logger, "can this work?");
}

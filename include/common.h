#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <image_transport/image_transport.hpp>

#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <orb_slam3/srv/save_map.hpp> // This file is created automatically, see here http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv#Creating_a_srv
#include <orb_slam3/msg/atlas.hpp> // This file is created automatically, see here http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv#Creating_a_srv

// ORB-SLAM3-specific libraries
#include "System.h"
#include "ImuTypes.h"
#include "Atlas.h"

extern ORB_SLAM3::System* pSLAM;
extern ORB_SLAM3::System::eSensor sensor_type;

extern std::string world_frame_id, cam_frame_id, imu_frame_id;

extern rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;
extern rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
extern rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr kf_markers_pub;
extern rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr tracked_mappoints_pub, all_mappoints_pub;
extern image_transport::Publisher tracking_img_pub;

void setup_services(rclcpp::Node::SharedPtr, std::string);
void setup_publishers(rclcpp::Node::SharedPtr, std::string);
void publish_topics(rclcpp::Time, Eigen::Vector3f = Eigen::Vector3f::Zero());

void publish_camera_pose(Sophus::SE3f, rclcpp::Time);
void publish_tracking_img(cv::Mat, rclcpp::Time);
void publish_tracked_points(std::vector<ORB_SLAM3::MapPoint*>, rclcpp::Time);
void publish_keypoints(std::vector<ORB_SLAM3::MapPoint*>, std::vector<cv::KeyPoint>, rclcpp::Time);
sensor_msgs::msg::PointCloud2 keypoints_to_pointcloud(std::vector<cv::KeyPoint>&, rclcpp::Time);

void publish_all_points(std::vector<ORB_SLAM3::MapPoint*>, rclcpp::Time);
void publish_tf_transform(Sophus::SE3f, string, string, rclcpp::Time);
void publish_body_odom(Sophus::SE3f, Eigen::Vector3f, Eigen::Vector3f, rclcpp::Time);
void publish_kf_markers(std::vector<Sophus::SE3f>, rclcpp::Time);
void publish_kf(cv::Mat, rclcpp::Time);
void publish_kf_right(cv::Mat, rclcpp::Time);
orb_slam3::msg::Atlas create_atlas_msg(ORB_SLAM3::Atlas *atlas, rclcpp::Time msg_time, std::string frame_id);
void publish_atlas(ORB_SLAM3::Atlas*, rclcpp::Time);

void save_map_srv(const orb_slam3::srv::SaveMap::Request::SharedPtr, orb_slam3::srv::SaveMap::Response::SharedPtr);
void save_traj_srv(const orb_slam3::srv::SaveMap::Request::SharedPtr, orb_slam3::srv::SaveMap::Response::SharedPtr);

cv::Mat SE3f_to_cvMat(Sophus::SE3f);
geometry_msgs::msg::Transform SE3f_to_tfTransform(Sophus::SE3f);
sensor_msgs::msg::PointCloud2 mappoint_to_pointcloud(std::vector<ORB_SLAM3::MapPoint*>, rclcpp::Time);

rclcpp::Node::SharedPtr init(int argc, char **argv, std::string name, ORB_SLAM3::System::eSensor sensor);
void run(rclcpp::Node::SharedPtr node);

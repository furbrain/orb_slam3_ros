/**
* 
* Adapted from ORB-SLAM3: Examples/ROS/src/ros_mono.cc
*
*/

#include "common.h"

using std::placeholders::_1;
using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(){};

    void GrabImage(const sensor_msgs::msg::Image::SharedPtr msg);
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("mono");
    auto logger = node->get_logger();
    if (argc > 1)
    {
        RCLCPP_WARN(logger, "Arguments supplied via command line are ignored.");
    }

    std::string node_name = node->get_name();

    image_transport::ImageTransport image_transport(node);

    std::string voc_file = node->declare_parameter("voc_file", "file_not_set");
    std::string settings_file = node->declare_parameter("settings_file", "file_not_set");

    if (voc_file == "file_not_set" || settings_file == "file_not_set")
    {
        RCLCPP_ERROR(logger, "Please provide voc_file and settings_file in the launch file");       
        rclcpp::shutdown();
        return 1;
    }

    std::string world_frame_id = node->declare_parameter("world_frame_id", "map");
    std::string cam_frame_id = node->declare_parameter("cam_frame_id", "camera");

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    sensor_type = ORB_SLAM3::System::MONOCULAR;
    pSLAM = new ORB_SLAM3::System(voc_file, settings_file, sensor_type);
    ImageGrabber igb;

    auto sub_img = node->create_subscription<sensor_msgs::msg::Image>("/camera/image_raw", 1, std::bind(&ImageGrabber::GrabImage, &igb,_1));

    setup_publishers(node, image_transport, node_name);
    setup_services(node, node_name);

    rclcpp::spin_some(node);

    // Stop all threads
    pSLAM->Shutdown();
    rclcpp::shutdown();

    return 0;
}

//////////////////////////////////////////////////
// Functions
//////////////////////////////////////////////////

void ImageGrabber::GrabImage(const sensor_msgs::msg::Image::SharedPtr msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger(""),"cv_bridge exception: %s", e.what());
        return;
    }

    // ORB-SLAM3 runs in TrackMonocular()
    Sophus::SE3f Tcw = pSLAM->TrackMonocular(cv_ptr->image, rclcpp::Time(cv_ptr->header.stamp).seconds());

    rclcpp::Time msg_time = msg->header.stamp;

    publish_topics(msg_time);
}

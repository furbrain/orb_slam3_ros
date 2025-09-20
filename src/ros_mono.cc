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
    auto node = init(argc, argv, "mono", ORB_SLAM3::System::MONOCULAR);
    if (node==NULL) return 1;
    std::string node_name = node->get_name();

    ImageGrabber igb;

    auto sub_img = node->create_subscription<sensor_msgs::msg::Image>("/camera/image_raw", 1, std::bind(&ImageGrabber::GrabImage, &igb,_1));
    

    setup_publishers(node, node_name);
    setup_services(node, node_name);

    run(node);
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

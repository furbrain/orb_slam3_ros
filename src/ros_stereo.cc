/**
* 
* Adapted from ORB-SLAM3: Examples/ROS/src/ros_stereo.cc
*
*/

#include "common.h"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(){};

    void GrabStereo(const sensor_msgs::msg::Image::ConstSharedPtr msgLeft, const sensor_msgs::msg::Image::ConstSharedPtr msgRight);
};

int main(int argc, char **argv)
{
    auto node = init(argc, argv, "Stereo", ORB_SLAM3::System::STEREO);
    if (node==NULL) return 1;
    std::string node_name = node->get_name();

    ImageGrabber igb;

    message_filters::Subscriber<sensor_msgs::msg::Image> sub_img_left(node, "/camera/left/image_raw");
    message_filters::Subscriber<sensor_msgs::msg::Image> sub_img_right(node, "/camera/right/image_raw");
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), sub_img_left, sub_img_right);
    sync.registerCallback(std::bind(&ImageGrabber::GrabStereo, &igb, _1, _2));

    setup_publishers(node, node_name);
    setup_services(node, node_name);

    run(node);

    return 0;
}

//////////////////////////////////////////////////
// Functions
//////////////////////////////////////////////////

void ImageGrabber::GrabStereo(const sensor_msgs::msg::Image::ConstSharedPtr msgLeft,const sensor_msgs::msg::Image::ConstSharedPtr msgRight)
{
    rclcpp::Time msg_time(msgLeft->header.stamp);

    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrLeft, cv_ptrRight;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger(""), "cv_bridge exception: %s", e.what());
        return;
    }

    // ORB-SLAM3 runs in TrackStereo()
    Sophus::SE3f Tcw = pSLAM->TrackStereo(cv_ptrLeft->image, cv_ptrRight->image, msg_time.seconds());
    if (pSLAM->GetLastFrameIsKF()) 
    {
        publish_atlas(pSLAM->GetAtlas(), msg_time);
        publish_kf(cv_ptrLeft->image, msg_time);
    }
    publish_topics(msg_time);
}

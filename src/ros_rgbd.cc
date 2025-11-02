/**
* 
* Adapted from ORB-SLAM3: Examples/ROS/src/ros_rgbd.cc
*
*/

#include "common.h"
#include "System.h"
using std::placeholders::_1;
using std::placeholders::_2;
using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(){};

    void GrabRGBD(const sensor_msgs::msg::Image::ConstSharedPtr msgRGB,const sensor_msgs::msg::Image::ConstSharedPtr msgD);
private:
    unsigned int mframeCount = 0;
};

int main(int argc, char **argv)
{
    auto node = init(argc, argv, "RGBD", ORB_SLAM3::System::RGBD);
    if (node==NULL) return 1;
    std::string node_name = node->get_name();

    ImageGrabber igb;

    message_filters::Subscriber<sensor_msgs::msg::Image> sub_rgb_img(node, "/camera/rgb/image_raw");
    message_filters::Subscriber<sensor_msgs::msg::Image> sub_depth_img(node, "/camera/depth_registered/image_raw");
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), sub_rgb_img, sub_depth_img);
    sync.registerCallback(std::bind(&ImageGrabber::GrabRGBD, &igb, _1, _2));

    setup_publishers(node, node_name);
    setup_services(node, node_name);

    run(node);

    return 0;
}

//////////////////////////////////////////////////
// Functions
//////////////////////////////////////////////////

void ImageGrabber::GrabRGBD(const sensor_msgs::msg::Image::ConstSharedPtr msgRGB,const sensor_msgs::msg::Image::ConstSharedPtr msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger(""),"cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger(""),"cv_bridge exception: %s", e.what());
        return;
    }
    // ORB-SLAM3 runs in TrackRGBD()
    Sophus::SE3f Tcw = pSLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, rclcpp::Time(cv_ptrRGB->header.stamp).seconds());

    rclcpp::Time msg_time(cv_ptrRGB->header.stamp);

    publish_topics(msg_time);
    if (mframeCount++ % 20 == 0) 
    {
        publish_atlas(pSLAM->GetAtlas(), msg_time);
    }
}

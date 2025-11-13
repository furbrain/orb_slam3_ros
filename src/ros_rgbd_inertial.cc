/**
* 
* Adapted from ORB-SLAM3: Examples/ROS/src/ros_mono_inertial.cc and ros_rgbd.cc
*
*/

#include "common.h"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std;

class ImuGrabber
{
public:
    ImuGrabber(){};
    void GrabImu(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg);

    queue<sensor_msgs::msg::Imu::ConstSharedPtr> imuBuf;
    std::mutex mBufMutex;
};

class ImageGrabber
{
public:
    ImageGrabber(ImuGrabber *pImuGb): mpImuGb(pImuGb){}

    void GrabRGBD(const sensor_msgs::msg::Image::ConstSharedPtr msgRGB, const sensor_msgs::msg::Image::ConstSharedPtr msgD);
    cv::Mat GetImage(const sensor_msgs::msg::Image::ConstSharedPtr img_msg);
    void SyncWithImu();
    void ShowStats();
    queue<sensor_msgs::msg::Image::ConstSharedPtr> imgRGBBuf, imgDBuf;
    std::mutex mBufMutex;
    ImuGrabber *mpImuGb;
private:
    unsigned int mframeCount = 0;
};


int main(int argc, char **argv)
{
    auto node = init(argc, argv, "RGBD_Inertial", ORB_SLAM3::System::IMU_RGBD);
    if (node==NULL) return 1;
    std::string node_name = node->get_name();

    ImuGrabber imugb;
    ImageGrabber igb(&imugb);

    auto sub_imu = node->create_subscription<sensor_msgs::msg::Imu>("/imu", 1, std::bind(&ImuGrabber::GrabImu, &imugb,_1));

    message_filters::Subscriber<sensor_msgs::msg::Image> sub_rgb_img(node, "/camera/rgb/image_raw");
    message_filters::Subscriber<sensor_msgs::msg::Image> sub_depth_img(node, "/camera/depth_registered/image_raw");

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(5), sub_rgb_img, sub_depth_img);
    sync.registerCallback(std::bind(&ImageGrabber::GrabRGBD, &igb, _1, _2));

    setup_publishers(node, node_name);
    setup_services(node, node_name);

    std::thread sync_thread(&ImageGrabber::SyncWithImu, &igb);

    run(node);

    return 0;
}

//////////////////////////////////////////////////
// Functions
//////////////////////////////////////////////////

void ImageGrabber::GrabRGBD(const sensor_msgs::msg::Image::ConstSharedPtr msgRGB,const sensor_msgs::msg::Image::ConstSharedPtr msgD)
{
    mBufMutex.lock();

    if (imgRGBBuf.size()>5)
        imgRGBBuf.pop();
    imgRGBBuf.push(msgRGB);

    if (imgDBuf.size()>5)
        imgDBuf.pop();
    imgDBuf.push(msgD);

    mBufMutex.unlock();
}

void ImageGrabber::ShowStats()
{

    mBufMutex.lock();
    mBufMutex.unlock();

}

cv::Mat ImageGrabber::GetImage(const sensor_msgs::msg::Image::ConstSharedPtr img_msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(img_msg);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger(""), "cv_bridge exception: %s", e.what());
    }

    return cv_ptr->image.clone();
}

void ImageGrabber::SyncWithImu()
{
    while(1)
    {
        if (!imgRGBBuf.empty() && !mpImuGb->imuBuf.empty())
        {
            cv::Mat im, depth;

            rclcpp::Time tIm(imgRGBBuf.front()->header.stamp);
            if (tIm > rclcpp::Time(mpImuGb->imuBuf.back()->header.stamp))
                continue;
            
            this->mBufMutex.lock();
            rclcpp::Time msg_time(imgRGBBuf.front()->header.stamp);
            im = GetImage(imgRGBBuf.front());
            imgRGBBuf.pop();
            depth = GetImage(imgDBuf.front());
            imgDBuf.pop();
            this->mBufMutex.unlock();

            vector<ORB_SLAM3::IMU::Point> vImuMeas;
            vImuMeas.clear();
            Eigen::Vector3f Wbb;
            mpImuGb->mBufMutex.lock();
            if (!mpImuGb->imuBuf.empty())
            {
                // Load imu measurements from buffer
                while(!mpImuGb->imuBuf.empty() && rclcpp::Time(mpImuGb->imuBuf.front()->header.stamp) <= tIm)
                {
                    rclcpp::Time t(mpImuGb->imuBuf.front()->header.stamp);

                    cv::Point3f acc(mpImuGb->imuBuf.front()->linear_acceleration.x, mpImuGb->imuBuf.front()->linear_acceleration.y, mpImuGb->imuBuf.front()->linear_acceleration.z);
                    
                    cv::Point3f gyr(mpImuGb->imuBuf.front()->angular_velocity.x, mpImuGb->imuBuf.front()->angular_velocity.y, mpImuGb->imuBuf.front()->angular_velocity.z);

                    Eigen::Quaternionf quat(mpImuGb->imuBuf.front()->orientation.w, mpImuGb->imuBuf.front()->orientation.x,
                                                mpImuGb->imuBuf.front()->orientation.y, mpImuGb->imuBuf.front()->orientation.z);
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t.seconds(), quat));
                    
                    Wbb << mpImuGb->imuBuf.front()->angular_velocity.x, mpImuGb->imuBuf.front()->angular_velocity.y, mpImuGb->imuBuf.front()->angular_velocity.z;

                    mpImuGb->imuBuf.pop();
                }
            }
            mpImuGb->mBufMutex.unlock();

            // ORB-SLAM3 runs in TrackRGBD()
            Sophus::SE3f Tcw = pSLAM->TrackRGBD(im, depth, tIm.seconds(), vImuMeas);
            
            publish_topics(msg_time, Wbb);
            if (mframeCount++ % 20 == 0) 
            {
                publish_atlas(pSLAM->GetAtlas(), msg_time);
            }

            //if (pSLAM->GetLastFrameIsKF()) 
            //{
            //    publish_atlas(pSLAM->GetAtlas(), msg_time);
            //    publish_kf(im, msg_time);
            //}
        }

        std::chrono::milliseconds tSleep(1);
        std::this_thread::sleep_for(tSleep);
    }
}

void ImuGrabber::GrabImu(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg)
{
    mBufMutex.lock();
    imuBuf.push(imu_msg);
    mBufMutex.unlock();

    return;
}

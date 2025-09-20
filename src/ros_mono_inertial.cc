/**
* 
* Adapted from ORB-SLAM3: Examples/ROS/src/ros_mono_inertial.cc
*
*/

#include "common.h"

using std::placeholders::_1;
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

    void GrabImage(const sensor_msgs::msg::Image::ConstSharedPtr msg);
    cv::Mat GetImage(const sensor_msgs::msg::Image::ConstSharedPtr img_msg);
    void SyncWithImu();

    queue<sensor_msgs::msg::Image::ConstSharedPtr> img0Buf;
    std::mutex mBufMutex;
    ImuGrabber *mpImuGb;
};


int main(int argc, char **argv)
{
    auto node = init(argc, argv, "Mono_Inertial", ORB_SLAM3::System::IMU_MONOCULAR);
    if (node==NULL) return 1;

    std::string node_name = node->get_name();
    ImuGrabber imugb;
    ImageGrabber igb(&imugb);


    auto sub_imu = node->create_subscription<sensor_msgs::msg::Imu>("/imu", 1, std::bind(&ImuGrabber::GrabImu, &imugb,_1));
    auto sub_img = node->create_subscription<sensor_msgs::msg::Image>("/camera/image_raw", 1, std::bind(&ImageGrabber::GrabImage, &igb,_1));

    setup_publishers(node, node_name);
    setup_services(node, node_name);
    
    std::thread sync_thread(&ImageGrabber::SyncWithImu, &igb);

    run(node);

    return 0;
}

//////////////////////////////////////////////////
// Functions
//////////////////////////////////////////////////

void ImageGrabber::GrabImage(const sensor_msgs::msg::Image::ConstSharedPtr img_msg)
{
    mBufMutex.lock();
    if (!img0Buf.empty())
        img0Buf.pop();
    img0Buf.push(img_msg);
    mBufMutex.unlock();
}

cv::Mat ImageGrabber::GetImage(const sensor_msgs::msg::Image::ConstSharedPtr img_msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger(""), "cv_bridge exception: %s", e.what());
    }
    
    if(cv_ptr->image.type()==0)
    {
        return cv_ptr->image.clone();
    }
    else
    {
        std::cout << "Error type" << std::endl;
        return cv_ptr->image.clone();
    }
}

void ImageGrabber::SyncWithImu()
{
    while(1)
    {
        if (!img0Buf.empty()&&!mpImuGb->imuBuf.empty())
        {
            cv::Mat im;
            rclcpp::Time tIm(img0Buf.front()->header.stamp);
            if(tIm > rclcpp::Time(mpImuGb->imuBuf.back()->header.stamp))
                continue;
            
            this->mBufMutex.lock();
            im = GetImage(img0Buf.front());
            rclcpp::Time msg_time = img0Buf.front()->header.stamp;
            img0Buf.pop();
            this->mBufMutex.unlock();

            vector<ORB_SLAM3::IMU::Point> vImuMeas;
            Eigen::Vector3f Wbb;
            mpImuGb->mBufMutex.lock();
            if (!mpImuGb->imuBuf.empty())
            {
                // Load imu measurements from buffer
                vImuMeas.clear();
                while(!mpImuGb->imuBuf.empty() && rclcpp::Time(mpImuGb->imuBuf.front()->header.stamp) <= tIm)
                {
                    rclcpp::Time t(mpImuGb->imuBuf.front()->header.stamp);

                    cv::Point3f acc(mpImuGb->imuBuf.front()->linear_acceleration.x, mpImuGb->imuBuf.front()->linear_acceleration.y, mpImuGb->imuBuf.front()->linear_acceleration.z);
                    
                    cv::Point3f gyr(mpImuGb->imuBuf.front()->angular_velocity.x, mpImuGb->imuBuf.front()->angular_velocity.y, mpImuGb->imuBuf.front()->angular_velocity.z);

                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t.seconds()));
                    
                    Wbb << mpImuGb->imuBuf.front()->angular_velocity.x, mpImuGb->imuBuf.front()->angular_velocity.y, mpImuGb->imuBuf.front()->angular_velocity.z;

                    mpImuGb->imuBuf.pop();
                }
            }
            mpImuGb->mBufMutex.unlock();

            // ORB-SLAM3 runs in TrackMonocular()
            Sophus::SE3f Tcw = pSLAM->TrackMonocular(im, tIm.seconds(), vImuMeas);
            
            publish_topics(msg_time, Wbb);
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

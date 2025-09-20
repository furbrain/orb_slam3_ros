/**
* 
* Adapted from ORB-SLAM3: Examples/ROS/src/ros_stereo_inertial.cc
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

    void GrabImageLeft(const sensor_msgs::msg::Image::ConstSharedPtr msg);
    void GrabImageRight(const sensor_msgs::msg::Image::ConstSharedPtr msg);
    cv::Mat GetImage(const sensor_msgs::msg::Image::ConstSharedPtr img_msg);
    void SyncWithImu();

    queue<sensor_msgs::msg::Image::ConstSharedPtr> imgLeftBuf, imgRightBuf;
    std::mutex mBufMutexLeft,mBufMutexRight;
    ImuGrabber *mpImuGb;
};

int main(int argc, char **argv)
{
    auto node = init(argc, argv, "Stereo_Inertion", ORB_SLAM3::System::IMU_STEREO);
    if (node==NULL) return 1;
    std::string node_name = node->get_name();

    ImuGrabber imugb;
    ImageGrabber igb(&imugb);

    // Maximum delay, 5 seconds * 200Hz = 1000 samples
    auto sub_imu = node->create_subscription<sensor_msgs::msg::Imu>("/imu", 1, std::bind(&ImuGrabber::GrabImu, &imugb,_1));
    auto sub_img_left = node->create_subscription<sensor_msgs::msg::Image>("/camera/left/image_raw", 1, std::bind(&ImageGrabber::GrabImageLeft, &igb,_1));
    auto sub_img_right = node->create_subscription<sensor_msgs::msg::Image>("/camera/right/image_raw", 1, std::bind(&ImageGrabber::GrabImageRight, &igb,_1));

    setup_publishers(node, node_name);
    setup_services(node, node_name);

    std::thread sync_thread(&ImageGrabber::SyncWithImu, &igb);

    run(node);

    return 0;
}

//////////////////////////////////////////////////
// Functions
//////////////////////////////////////////////////

void ImageGrabber::GrabImageLeft(const sensor_msgs::msg::Image::ConstSharedPtr img_msg)
{
    mBufMutexLeft.lock();
    if (!imgLeftBuf.empty())
        imgLeftBuf.pop();
    imgLeftBuf.push(img_msg);
    mBufMutexLeft.unlock();
}

void ImageGrabber::GrabImageRight(const sensor_msgs::msg::Image::ConstSharedPtr img_msg)
{
    mBufMutexRight.lock();
    if (!imgRightBuf.empty())
        imgRightBuf.pop();
    imgRightBuf.push(img_msg);
    mBufMutexRight.unlock();
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
    const rclcpp::Duration maxTimeDiff = rclcpp::Duration(0, 10'000'000); //10 ms
    while(1)
    {
        cv::Mat imLeft, imRight;
        rclcpp::Time tImLeft, tImRight;
        if (!imgLeftBuf.empty()&&!imgRightBuf.empty()&&!mpImuGb->imuBuf.empty())
        {
            tImLeft = imgLeftBuf.front()->header.stamp;
            tImRight = imgRightBuf.front()->header.stamp;

            this->mBufMutexRight.lock();
            while((tImLeft-tImRight)>maxTimeDiff && imgRightBuf.size()>1)
            {
                imgRightBuf.pop();
                tImRight = imgRightBuf.front()->header.stamp;
            }
            this->mBufMutexRight.unlock();

            this->mBufMutexLeft.lock();
            while((tImRight-tImLeft)>maxTimeDiff && imgLeftBuf.size()>1)
            {
                imgLeftBuf.pop();
                tImLeft = imgLeftBuf.front()->header.stamp;
            }
            this->mBufMutexLeft.unlock();

            if((tImLeft-tImRight)>maxTimeDiff || (tImRight-tImLeft)>maxTimeDiff)
            {
                // std::cout << "big time difference" << std::endl;
                continue;
            }
            if(tImLeft > mpImuGb->imuBuf.back()->header.stamp)
                continue;

            this->mBufMutexLeft.lock();
            imLeft = GetImage(imgLeftBuf.front());
            rclcpp::Time msg_time = imgLeftBuf.front()->header.stamp;
            imgLeftBuf.pop();
            this->mBufMutexLeft.unlock();

            this->mBufMutexRight.lock();
            imRight = GetImage(imgRightBuf.front());
            imgRightBuf.pop();
            this->mBufMutexRight.unlock();

            vector<ORB_SLAM3::IMU::Point> vImuMeas;
            Eigen::Vector3f Wbb;
            mpImuGb->mBufMutex.lock();
            if(!mpImuGb->imuBuf.empty())
            {
                // Load imu measurements from buffer
                vImuMeas.clear();
                while(!mpImuGb->imuBuf.empty() && rclcpp::Time(mpImuGb->imuBuf.front()->header.stamp) <= tImLeft)
                {
                    rclcpp::Time t = mpImuGb->imuBuf.front()->header.stamp;

                    cv::Point3f acc(mpImuGb->imuBuf.front()->linear_acceleration.x, mpImuGb->imuBuf.front()->linear_acceleration.y, mpImuGb->imuBuf.front()->linear_acceleration.z);

                    cv::Point3f gyr(mpImuGb->imuBuf.front()->angular_velocity.x, mpImuGb->imuBuf.front()->angular_velocity.y, mpImuGb->imuBuf.front()->angular_velocity.z);
                    
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t.seconds()));

                    Wbb << mpImuGb->imuBuf.front()->angular_velocity.x, mpImuGb->imuBuf.front()->angular_velocity.y, mpImuGb->imuBuf.front()->angular_velocity.z;

                    mpImuGb->imuBuf.pop();
                }
            }
            mpImuGb->mBufMutex.unlock();
            
            // ORB-SLAM3 runs in TrackStereo()
            Sophus::SE3f Tcw = pSLAM->TrackStereo(imLeft,imRight,tImLeft.seconds(),vImuMeas);

            publish_topics(msg_time, Wbb);
            if (pSLAM->GetLastFrameIsKF()) 
            {
                publish_atlas(pSLAM->GetAtlas(), msg_time);
                publish_kf(imLeft, msg_time);
            }
            std::chrono::milliseconds tSleep(1);
            std::this_thread::sleep_for(tSleep);
        }
    }
}

void ImuGrabber::GrabImu(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg)
{
    mBufMutex.lock();
    imuBuf.push(imu_msg);
    mBufMutex.unlock();
    
    return;
}

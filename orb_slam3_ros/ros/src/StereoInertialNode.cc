/**
 * @file StereoInertialNode.cc
 * @author LauZanMo (LauZanMo@whu.edu.cn)
 * @brief 
 * @version 1.0
 * @date 2020-12-28
 * 
 * MIT License
 * 
 * @copyright Copyright (c) 2020 WHU-Drones
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 */

#include "StereoInertialNode.h"

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "StereoInertial");
    ros::start();

    if(argc > 1) {
        ROS_WARN ("Arguments supplied via command line are neglected.");
    }

    ros::NodeHandle node_handle;
    image_transport::ImageTransport image_transport (node_handle);

    // initialize
    StereoInertialNode node (ORB_SLAM3::System::IMU_STEREO, node_handle, image_transport);

    node.Init();
    std::thread sync_thread(&StereoInertialNode::SyncWithImu, &node);

    ros::spin();

    return 0;
}

/**
 * @brief Construct a new Stereo Inertial Node:: Stereo Inertial Node object
 * 
 * @param sensor 
 * @param node_handle 
 * @param image_transport 
 */
StereoInertialNode::StereoInertialNode(const ORB_SLAM3::System::eSensor sensor, ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport):
    CommonNode (sensor, node_handle, image_transport)
{
    name_of_node_ = ros::this_node::getName();
    left_sub_ = node_handle.subscribe<sensor_msgs::Image>("image_left/image_color_rect", 100, &StereoInertialNode::GrabImageLeft, this);
    right_sub_ = node_handle.subscribe<sensor_msgs::Image>("image_right/image_color_rect", 100, &StereoInertialNode::GrabImageRight, this);
    imu_sub_ = node_handle.subscribe<sensor_msgs::Imu>("/slam_imu", 1000, &StereoInertialNode::GrabImu, this);
    node_handle.param(name_of_node_+ "/use_clahe", clahe_param_, true);
}

/**
 * @brief Destroy the Stereo Inertial Node:: Stereo Inertial Node object
 * 
 */
StereoInertialNode::~StereoInertialNode() {}

/**
 * @brief sync imu and stereo image and call trackstereo function
 * 
 */
void StereoInertialNode::SyncWithImu()
{
    const double maxTimeDiff = 0.01;
    while (1)
    {
        cv::Mat imLeft, imRight;
        double tImLeft = 0, tImRight = 0;
        if (!left_buf_.empty() && !right_buf_.empty() && !imu_buf_.empty())
        {
            tImLeft = left_buf_.front()->header.stamp.toSec();
            tImRight = right_buf_.front()->header.stamp.toSec();

            this->left_mutex_.lock();
            while((tImLeft - tImRight) > maxTimeDiff && right_buf_.size() > 1)
            {
                right_buf_.pop();
                tImRight = right_buf_.front()->header.stamp.toSec();
            }
            this->left_mutex_.unlock();

            this->right_mutex_.lock();
            while((tImRight - tImLeft) > maxTimeDiff && left_buf_.size() > 1)
            {
                left_buf_.pop();
                tImLeft = left_buf_.front()->header.stamp.toSec();
            }
            this->right_mutex_.unlock();

            if((tImLeft-tImRight)>maxTimeDiff || (tImRight-tImLeft)>maxTimeDiff)
                continue;
            if(tImLeft>imu_buf_.back()->header.stamp.toSec())
                continue;
            
            this->left_mutex_.lock();
            imLeft = GetImage(left_buf_.front());
            left_buf_.pop();
            this->left_mutex_.unlock();

            this->right_mutex_.lock();
            imRight = GetImage(right_buf_.front());
            right_buf_.pop();
            this->right_mutex_.unlock();

            vector<ORB_SLAM3::IMU::Point> vImuMeas;
            imu_mutex_.lock();
            if(!imu_buf_.empty())
            {
                // Load imu measurements from buffer
                vImuMeas.clear();
                while(!imu_buf_.empty() && imu_buf_.front()->header.stamp.toSec() <= tImLeft)
                {
                    double t = imu_buf_.front()->header.stamp.toSec();
                    cv::Point3f acc(imu_buf_.front()->linear_acceleration.x, imu_buf_.front()->linear_acceleration.y, imu_buf_.front()->linear_acceleration.z);
                    cv::Point3f gyr(imu_buf_.front()->angular_velocity.x, imu_buf_.front()->angular_velocity.y, imu_buf_.front()->angular_velocity.z);
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc,gyr,t));
                    imu_buf_.pop();
                }
            }
            imu_mutex_.unlock();

            if (clahe_param_)
            {
                clahe_->apply(imLeft, imLeft);
                clahe_->apply(imRight,imRight);
            }

            orb_slam_->TrackStereo(imLeft,imRight,tImLeft,vImuMeas);
            Update ();

            std::chrono::milliseconds tSleep(1);
            std::this_thread::sleep_for(tSleep);
        } 
    } 
}

/**
 * @brief transform image from ros type to cv type in deep copy 
 * 
 * @param img_msg 
 * @return cv::Mat 
 */
cv::Mat StereoInertialNode::GetImage(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(img_msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    
    return cv_ptr->image.clone();
}

/**
 * @brief imu message callback
 * 
 * @param imu_msg 
 */
void StereoInertialNode::GrabImu(const sensor_msgs::ImuConstPtr &imu_msg)
{
    imu_mutex_.lock();
    imu_buf_.push(imu_msg);
    imu_mutex_.unlock();
}

/**
 * @brief left image callback
 * 
 * @param img_msg 
 */
void StereoInertialNode::GrabImageLeft(const sensor_msgs::ImageConstPtr& img_msg)
{
    left_mutex_.lock();
    if(!left_buf_.empty())
        left_buf_.pop();
    left_buf_.push(img_msg);
    left_mutex_.unlock();
}

/**
 * @brief right image callback
 * 
 * @param img_msg 
 */
void StereoInertialNode::GrabImageRight(const sensor_msgs::ImageConstPtr& img_msg)
{
    right_mutex_.lock();
    if(!right_buf_.empty())
        right_buf_.pop();
    right_buf_.push(img_msg);
    right_mutex_.unlock();
}
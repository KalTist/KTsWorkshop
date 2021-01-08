/**
 * @file StereoNode.cc
 * @author LauZanMo (LauZanMo@whu.edu.cn)
 * @brief 
 * @version 1.0
 * @date 2020-12-30
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
#include "StereoNode.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Stereo");
    ros::start();

    if(argc > 1) {
        ROS_WARN ("Arguments supplied via command line are neglected.");
    }

    ros::NodeHandle node_handle;
    image_transport::ImageTransport image_transport (node_handle);

    // initialize
    StereoNode node (ORB_SLAM3::System::STEREO, node_handle, image_transport);

    node.Init();

    ros::spin();

    return 0;
}

/**
 * @brief Construct a new Stereo Node:: Stereo Node object
 * 
 * @param sensor 
 * @param node_handle 
 * @param image_transport 
 */
StereoNode::StereoNode (const ORB_SLAM3::System::eSensor sensor, ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport):
    CommonNode (sensor, node_handle, image_transport) {
    left_sub_ = new message_filters::Subscriber<sensor_msgs::Image> (node_handle, "image_left/image_color_rect", 10);
    right_sub_ = new message_filters::Subscriber<sensor_msgs::Image> (node_handle, "image_right/image_color_rect", 10);
    camera_info_topic_ = "image_left/camera_info";
    test_pub_ = node_handle.advertise<std_msgs::Float64>("test", 100, true);

    sync_ = new message_filters::Synchronizer<sync_pol> (sync_pol(10), *left_sub_, *right_sub_);
    sync_->registerCallback(boost::bind(&StereoNode::ImageCallback, this, _1, _2));
}

/**
 * @brief Destroy the Stereo Node:: Stereo Node object
 * 
 */
StereoNode::~StereoNode () {
    delete left_sub_;
    delete right_sub_;
    delete sync_;
}

/**
 * @brief sync stereo image and call trackstereo function
 * 
 * @param msgLeft 
 * @param msgRight 
 */
void StereoNode::ImageCallback (const sensor_msgs::ImageConstPtr& msgLeft, const sensor_msgs::ImageConstPtr& msgRight) {
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    current_frame_time_ = msgLeft->header.stamp;
    test_pub_.publish(cv_ptrLeft->header.stamp.toSec());
    orb_slam_->TrackStereo(cv_ptrLeft->image, cv_ptrRight->image, cv_ptrLeft->header.stamp.toSec());

    Update ();
}

/**
 * @file StereoInertialNode.h
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
#pragma once

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Imu.h>
#include <image_transport/image_transport.h>

#include <opencv2/core/core.hpp>

#include "System.h"
#include "ImuTypes.h"
#include "CommonNode.h"

class StereoInertialNode : public CommonNode
{
public:
    StereoInertialNode(const ORB_SLAM3::System::eSensor sensor, ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport);
    ~StereoInertialNode();
    void SyncWithImu();
    cv::Mat GetImage(const sensor_msgs::ImageConstPtr &img_msg);
    void GrabImu(const sensor_msgs::ImuConstPtr &imu_msg);
    void GrabImageLeft(const sensor_msgs::ImageConstPtr& img_msg);
    void GrabImageRight(const sensor_msgs::ImageConstPtr& img_msg);
private:
    ros::Subscriber left_sub_;
    ros::Subscriber right_sub_;
    ros::Subscriber imu_sub_;

    std::string name_of_node_;

    mutex imu_mutex_, left_mutex_, right_mutex_;
    queue<sensor_msgs::ImuConstPtr> imu_buf_;
    queue<sensor_msgs::ImageConstPtr> left_buf_, right_buf_;

    bool clahe_param_;
    cv::Ptr<cv::CLAHE> clahe_ = cv::createCLAHE(3.0, cv::Size(8, 8));
};

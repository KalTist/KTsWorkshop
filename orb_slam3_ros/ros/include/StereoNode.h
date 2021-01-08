/**
 * @file StereoNode.h
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
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/core/types.hpp>

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float64.h>

#include "System.h"
#include "CommonNode.h"

class StereoNode : public CommonNode
{
public:
    StereoNode (const ORB_SLAM3::System::eSensor sensor, ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport);
    ~StereoNode ();
    void ImageCallback (const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight);

private:
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Subscriber<sensor_msgs::Image> *left_sub_;
    message_filters::Subscriber<sensor_msgs::Image> *right_sub_;
    message_filters::Synchronizer<sync_pol> *sync_;
    ros::Publisher test_pub_;
};


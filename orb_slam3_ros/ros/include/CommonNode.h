/**
 * @file CommonNode.h
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
#ifndef ORBSLAM3_ROS_NODE_H_
#define ORBSLAM3_ROS_NODE_H_

#include <vector>
#include <ros/ros.h>
#include <ros/time.h>
#include <image_transport/image_transport.h>
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>

#include <dynamic_reconfigure/server.h>
#include <orb_slam3_ros/dynamic_reconfigureConfig.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/CameraInfo.h>

#include "System.h"

class CommonNode
{
  public:
    CommonNode (ORB_SLAM3::System::eSensor sensor, ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport);
    ~CommonNode ();
    void Init ();

  protected:
    void Update ();
    ORB_SLAM3::System* orb_slam_;
    ros::Time current_frame_time_;
    std::string camera_info_topic_;

  private:
    void PublishMapPoints (std::vector<ORB_SLAM3::MapPoint*> map_points);
    void PublishPositionAsTransform (cv::Mat position);
    void PublishPositionAsPoseStamped(cv::Mat position);
    void PublishRenderedImage (cv::Mat image);
    void ParamsChangedCallback(orb_slam3_ros::dynamic_reconfigureConfig &config, uint32_t level);

    tf::Transform TransformFromMat (cv::Mat position_mat);
    sensor_msgs::PointCloud2 MapPointsToPointCloud (std::vector<ORB_SLAM3::MapPoint*> map_points);

    dynamic_reconfigure::Server<orb_slam3_ros::dynamic_reconfigureConfig> dynamic_param_server_;

    image_transport::Publisher rendered_image_publisher_;
    ros::Publisher map_points_publisher_;
    ros::Publisher pose_publisher_;
    ros::Publisher path_publisher_;

    nav_msgs::Path path_;

    std::string name_of_node_;
    ros::NodeHandle node_handle_;
    image_transport::ImageTransport image_transport_;

    ORB_SLAM3::System::eSensor sensor_;

    std::string map_frame_id_param_;
    std::string camera_frame_id_param_;
    std::string voc_file_name_param_;
    std::string setting_file_name_param_;
    bool publish_pointcloud_param_;
    bool publish_tf_param_;
    bool publish_pose_param_;
    bool publish_path_param_;
};

#endif //ORBSLAM3_ROS_NODE_H_

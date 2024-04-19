/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024, Metro Robots
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Metro Robots nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: David V. Lu!! */

#include "ped_tracker.hpp"

namespace stereolabs
{
PedTracker::PedTracker(rclcpp::Node& node, const tf2_ros::Buffer& tf_buffer, const std::string& source_frame)
  : tf_buffer_(tf_buffer), logger_(node.get_logger().get_child("ped_tracker")), source_frame_(source_frame)
{
  node.declare_parameter("pos_tracking.pedestrian_frame", "odom");
  node.get_parameter("pos_tracking.pedestrian_frame", target_frame_);
}

void PedTracker::update(const sl::Objects& objects, const rclcpp::Time& t)
{
  double deltaT;
  if (time_initialized_)
  {
    deltaT = (t - cached_stamp_).seconds();
  }
  else
  {
    deltaT = 0.1;
  }

  std::unordered_map<std::string, social_nav_msgs::msg::PedestrianWithCovariance> peopleLocationsMap;
  size_t idx = 0;
  for (auto data : objects.object_list)
  {
    if (data.label != sl::OBJECT_CLASS::PERSON)
    {
      idx++;
      continue;
    }

    std::string ident = "Person" + std::to_string(data.id);

    geometry_msgs::msg::PointStamped cam_point, odom_point;
    cam_point.header.stamp = t;
    cam_point.header.frame_id = source_frame_;
    cam_point.point.x = data.position[0];
    cam_point.point.y = data.position[1];
    cam_point.point.z = data.position[2];

    /* geometry_msgs::msg::Vector3Stamped velocity_v, new_v;
velocity_v.header = objMsg->header;
velocity_v.vector.x = data.velocity[0];
velocity_v.vector.y = data.velocity[1];
velocity_v.vector.z = data.velocity[2]; */

    try
    {
      odom_point = tf_buffer_.transform(cam_point, target_frame_);
      // new_v = tf_buffer_.transform(velocity_v, target_frame_);
    }
    catch (tf2::TransformException& ex)
    {
      rclcpp::Clock steady_clock(RCL_STEADY_TIME);
      RCLCPP_WARN_THROTTLE(logger_, steady_clock, 1.0, "The tf from '%s' to '%s' is not available.",
                           source_frame_.c_str(), target_frame_.c_str());
      idx++;
      continue;
    }

    social_nav_msgs::msg::PedestrianWithCovariance pedMsg;
    geometry_msgs::msg::Pose2D pose;
    pedMsg.pedestrian.pose.x = odom_point.point.x;
    pedMsg.pedestrian.pose.y = odom_point.point.y;
    pedMsg.pedestrian.pose.theta = atan2(data.position[1], data.position[0]);

    const auto& match = ped_map_.find(pedMsg.pedestrian.identifier);
    if (match != ped_map_.end())
    {
      const auto& cachePose = match->second.pedestrian.pose;
      pedMsg.pedestrian.velocity.x = (pedMsg.pedestrian.pose.x - cachePose.x) / deltaT;
      pedMsg.pedestrian.velocity.y = (pedMsg.pedestrian.pose.y - cachePose.y) / deltaT;
      pedMsg.pedestrian.velocity.theta = (pedMsg.pedestrian.pose.theta - cachePose.theta) / deltaT;
    }
    else
    {
      pedMsg.pedestrian.velocity.x = 0.0;
      pedMsg.pedestrian.velocity.y = 0.0;
      pedMsg.pedestrian.velocity.theta = 0.0;
    }

    pedMsg.covariance[0] = data.position_covariance[0];  // xx is index 0
    pedMsg.covariance[1] = data.position_covariance[1];  // xy is index 1.
    // xz is index 2, skipping.
    pedMsg.covariance[2] = data.position_covariance[1];  // yx is the same as xy.
    pedMsg.covariance[3] = data.position_covariance[3];  // yy is index 3

    peopleLocationsMap[ident] = pedMsg;

    idx++;
  }

  ped_map_.swap(peopleLocationsMap);
  cached_stamp_ = t;
  time_initialized_ = true;
}

social_nav_msgs::msg::PedestriansWithCovariance PedTracker::getMsg()
{
  social_nav_msgs::msg::PedestriansWithCovariance pedsMsg;
  pedsMsg.header.stamp = cached_stamp_;
  pedsMsg.header.frame_id = target_frame_;

  for (const auto& pair : ped_map_)
  {
    social_nav_msgs::msg::PedestrianWithCovariance pedMsg;
    pedMsg.pedestrian.identifier = pair.first;
    pedsMsg.pedestrians.push_back(pedMsg);
  }

  return pedsMsg;
}

}  // namespace stereolabs

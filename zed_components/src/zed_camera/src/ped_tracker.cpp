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

  size_t idx = 0;
  for (auto data : objects.object_list)
  {
    if (data.label != sl::OBJECT_CLASS::PERSON)
    {
      idx++;
      continue;
    }

    auto track = ped_map_.find(data.id);
    if (track == ped_map_.end())
    {
      ped_map_.insert(std::make_pair(data.id, TrackedPed(*this, data.id)));
    }

    geometry_msgs::msg::PointStamped cam_point;
    cam_point.header.stamp = t;
    cam_point.header.frame_id = source_frame_;
    cam_point.point.x = data.position[0];
    cam_point.point.y = data.position[1];
    cam_point.point.z = data.position[2];

    track->second.update(cam_point);
    idx++;
  }

  cached_stamp_ = t;
}

social_nav_msgs::msg::PedestriansWithCovariance PedTracker::getMsg()
{
  social_nav_msgs::msg::PedestriansWithCovariance pedsMsg;
  pedsMsg.header.stamp = cached_stamp_;
  pedsMsg.header.frame_id = target_frame_;

  for (auto it = ped_map_.begin(); it != ped_map_.end();)
  {
    auto& track = it->second;
    if (track.isCurrent(cached_stamp_))
    {
      pedsMsg.pedestrians.push_back(track.getMsg());
      ++it;
    }
    else
    {
      it = ped_map_.erase(it);
    }
  }

  return pedsMsg;
}

PedTracker::TrackedPed::TrackedPed(PedTracker& parent, int label_id) : parent_(parent)
{
  label_ = "Person" + std::to_string(label_id);
}

void PedTracker::TrackedPed::update(const geometry_msgs::msg::PointStamped& point)
{
  geometry_msgs::msg::PointStamped target_point;
  try
  {
    target_point = parent_.tf_buffer_.transform(point, parent_.target_frame_);
  }
  catch (tf2::TransformException& ex)
  {
    rclcpp::Clock steady_clock(RCL_STEADY_TIME);
    RCLCPP_WARN_THROTTLE(parent_.logger_, steady_clock, 1.0, "The tf from '%s' to '%s' is not available.",
                         point.header.frame_id.c_str(), parent_.target_frame_.c_str());
    return;
  }
  points_.push(target_point);

  while (points_.size() > 2)
  {
    points_.pop();
  }
}

nav_2d_msgs::msg::Twist2D PedTracker::TrackedPed::getVelocity() const
{
  nav_2d_msgs::msg::Twist2D twist;
  if (points_.size() == 1)
  {
    return twist;
  }

  const auto& cachePoint0 = points_.front();
  const auto& cachePoint1 = points_.back();
  double t0 = cachePoint0.header.stamp.sec + cachePoint0.header.stamp.nanosec / 1e9;
  double t1 = cachePoint1.header.stamp.sec + cachePoint1.header.stamp.nanosec / 1e9;
  double deltaT = t1 - t0;
  twist.x = (cachePoint1.point.x - cachePoint0.point.x) / deltaT;
  twist.y = (cachePoint1.point.y - cachePoint0.point.y) / deltaT;
  // twist.theta = (cachePoint1.point.theta - cachePoint0.point.theta) / deltaT;

  return twist;
}

social_nav_msgs::msg::PedestrianWithCovariance PedTracker::TrackedPed::getMsg() const
{
  social_nav_msgs::msg::PedestrianWithCovariance pedMsg;
  pedMsg.pedestrian.identifier = label_;
  pedMsg.pedestrian.pose.x = points_.back().point.x;
  pedMsg.pedestrian.pose.y = points_.back().point.y;
  // pedMsg.pedestrian.pose.theta = atan2(data.position[1], data.position[0]);

  pedMsg.pedestrian.velocity = getVelocity();

  /*
    pedMsg.covariance[0] = data.position_covariance[0];  // xx is index 0
    pedMsg.covariance[1] = data.position_covariance[1];  // xy is index 1.
    // xz is index 2, skipping.
    pedMsg.covariance[2] = data.position_covariance[1];  // yx is the same as xy.
    pedMsg.covariance[3] = data.position_covariance[3];  // yy is index 3*/

  return pedMsg;
}

}  // namespace stereolabs

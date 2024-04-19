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

#pragma once
#include "sl_tools.hpp"
#include "sl_types.hpp"
#include <queue>

namespace stereolabs
{
class PedTracker
{
public:
  PedTracker(rclcpp::Node& node, const tf2_ros::Buffer& tf_buffer, const std::string& source_frame);

  void update(const sl::Objects& objects, const rclcpp::Time& t);

  social_nav_msgs::msg::PedestriansWithCovariance getMsg();

protected:
  const tf2_ros::Buffer& tf_buffer_;
  rclcpp::Logger logger_;
  rclcpp::Time cached_stamp_;
  bool time_initialized_{false};

  // Params
  std::string source_frame_, target_frame_;

  class TrackedPed
  {
  public:
    TrackedPed(PedTracker& parent, int label_id);
    void update(const geometry_msgs::msg::PointStamped& point);
    social_nav_msgs::msg::PedestrianWithCovariance getMsg() const;

    nav_2d_msgs::msg::Twist2D getVelocity() const;

    bool isCurrent(const rclcpp::Time& t) const
    {
      return points_.size() > 0 && points_.back().header.stamp == t;
    }

  protected:
    PedTracker& parent_;
    std::string label_;
    std::queue<geometry_msgs::msg::PointStamped> points_;
  };

  std::unordered_map<int, TrackedPed> ped_map_;
};
}  // namespace stereolabs

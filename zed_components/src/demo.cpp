#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "ped_tracker.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher() : Node("minimal_publisher")
  {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    ped_ = new stereolabs::PedTracker(*this, *tf_buffer_, "odom");

    ped_pub_ = create_publisher<social_nav_msgs::msg::PedestriansWithCovariance>("/peds2", 1);
    obj_sub_ = create_subscription<zed_interfaces::msg::ObjectsStamped>(
        "/top/zed/obj_det/objects", 1, std::bind(&MinimalPublisher::objCB, this, std::placeholders::_1));
  }

private:
  void objCB(const zed_interfaces::msg::ObjectsStamped::SharedPtr msg)
  {
    for (const auto& obj : msg->objects)
    {
      geometry_msgs::msg::PointStamped cam_point;
      cam_point.header = msg->header;
      cam_point.point.x = obj.position[0];
      cam_point.point.y = obj.position[1];
      cam_point.point.z = obj.position[2];

      ped_->updateSingle(obj.label_id, cam_point);
    }

    auto omsg = ped_->getMsg();
    ped_pub_->publish(omsg);
  }
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  stereolabs::PedTracker* ped_;
  rclcpp::Subscription<zed_interfaces::msg::ObjectsStamped>::SharedPtr obj_sub_;
  rclcpp::Publisher<social_nav_msgs::msg::PedestriansWithCovariance>::SharedPtr ped_pub_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
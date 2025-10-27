//	MTRX3760 2025 Project 2: Warehouse Robot DevKit
//	File: delivery.hpp
//	Author(s):
//
//	Delivery node for warehouse robot navigation and delivery tasks.
//	Manages delivery actions to specified poses using Nav2 action server.

#ifndef DELIVERY__DELIVERY_HPP
#define DELIVERY__DELIVERY_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/deliver_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

using DeliverToPose = nav2_msgs::action::DeliverToPose;
using DeliverToPoseGoalHandle = rclcpp_action::ClientGoalHandle<DeliverToPose>;

class Deliverer : public rclcpp::Node
{
public:
  Deliverer();

private:
  void sendGoal();
  void goalResultCallback(const DeliverToPoseGoalHandle::WrappedResult& result);

  rclcpp_action::Client<DeliverToPose>::SharedPtr client;
};

#endif // DELIVERY__DELIVERY_HPP

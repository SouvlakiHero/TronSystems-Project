#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/deliver_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

using DeliverToPose = nav2_msgs::action::DeliverToPose;
using DeliverToPoseGoalHandle = rclcpp_action::ClientGoalHandle<DeliverToPose>;

class Deliverer : public rclcpp::Node
{
public:
  Deliverer()
    : Node("deliverer_node")
  {
    RCLCPP_INFO(get_logger(), "Starting Deliverer Node");
    client = rclcpp_action::create_client<DeliverToPose>(this, "deliver_to_pose");
    sendGoal();
  }

private:
  void sendGoal()
  {
    if (!client->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(get_logger(), "Action server not available after waiting");
      return;
    }

    auto goal_msg = DeliverToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = now();

    goal_msg.pose.pose.position.x = 2.0;
    goal_msg.pose.pose.position.y = 3.0;
    goal_msg.pose.pose.orientation.w = 1.0;

    RCLCPP_INFO(get_logger(), "Sending goal to deliver to pose (2.0, 3.0)");

    auto options = rclcpp_action::Client<DeliverToPose>::SendGoalOptions();
    options.result_callback = std::bind(&Deliverer::goalResultCallback, this, std::placeholders::_1);

    client->async_send_goal(goal_msg, options);
  }

  void goalResultCallback(const DeliverToPoseGoalHandle::WrappedResult& result)
  {
    (void)result;
    RCLCPP_INFO(get_logger(), "Delivery action completed");
  }
    rclcpp_action::Client<DeliverToPose>::SharedPtr client;

};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Deliverer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
}
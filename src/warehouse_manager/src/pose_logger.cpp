#include "warehouse_manager/pose_logger.hpp"
#include <fstream>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

namespace wh {

// on instantiation, handle the ros sub + timer stuff
PoseLogger::PoseLogger(rclcpp::Node & node, const std::string & log_path)
: node_(node), log_path_(log_path)
{
	sub_ = node_.create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
		"/amcl_pose", rclcpp::QoS(10),
		std::bind(&PoseLogger::pose_cb, this, std::placeholders::_1));

	timer_ = node_.create_wall_timer(5s, std::bind(&PoseLogger::on_timer, this));

	RCLCPP_INFO(node_.get_logger(), "PoseLogger writing to: %s", log_path_.c_str());
}

// record the pose received from the subscription
void PoseLogger::pose_cb(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
	last_pose_ = *msg;
}

// on timer, log the pose of the bot
void PoseLogger::on_timer() {
	if (!last_pose_) return;

	// extract the coords and yaw
	const auto &p = last_pose_->pose.pose.position;
	const auto &q = last_pose_->pose.pose.orientation;

	double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
	double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
	double yaw = std::atan2(siny_cosp, cosy_cosp);

	// record them
	std::ofstream ofs(log_path_, std::ios::app);
	if (ofs) {
		ofs << std::fixed;
		ofs << rclcpp::Clock().now().seconds() << ", "
			<< p.x << ", " << p.y << ", " << yaw << "\n";
	}
}

} // namespace wh

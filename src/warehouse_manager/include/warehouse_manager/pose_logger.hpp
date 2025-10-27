#ifndef WAREHOUSE_MANAGER__POSE_LOGGER_HPP
#define WAREHOUSE_MANAGER__POSE_LOGGER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <optional>
#include <string>

namespace wh {

/**
 * PoseLogger — subscribes to /amcl_pose and writes x, y, yaw to a file periodically.
 * Runs on an existing Node (e.g., WarehouseManager) — no separate spin needed.
 */
class PoseLogger {
public:
	PoseLogger(rclcpp::Node & node, const std::string & log_path);

private:
	// callback to be run each timer interval
	void pose_cb(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
	void on_timer();

	// node stuff
	rclcpp::Node & node_;
	rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_;
	rclcpp::TimerBase::SharedPtr timer_;

	// stuff for logging pose
	std::string log_path_;
	std::optional<geometry_msgs::msg::PoseWithCovarianceStamped> last_pose_;
	
};

} // namespace wh
#endif

//	MTRX3760 2025 Project 2: Warehouse Robot DevKit
//	File: warehouse_manager.hpp
//	Author(s): max rogers
//
//	High-level controller for warehouse robot behaviour across phases.
// the warehouse manager starts the processes needed to manage the behaviour of the turtlebot
// it manages the current state of the turtlebot from a high level
// mapping, patrolling are implemented so far

#ifndef WAREHOUSE_MANAGER__WAREHOUSE_MANAGER_HPP
#define WAREHOUSE_MANAGER__WAREHOUSE_MANAGER_HPP

#include <memory>
#include <string>
#include "warehouse_manager/node_interfaces.hpp"
#include "warehouse_manager/pose_logger.hpp"
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>


namespace wh {

enum class Phase { Idle = 0, Mapping, Patrolling };

class WarehouseManager : public rclcpp::Node {
public:
	explicit WarehouseManager(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
	//	Parameters
	std::string log_dir_;
	std::string map_dir_;

	//	State
	Phase phase_{Phase::Idle};
	std::string last_map_yaml_path_;	

	//	Interfaces (processes started and managed by the warehouse manager)
	std::unique_ptr<CartographerInterface> cartographer_;
	std::unique_ptr<MazeDriverInterface> maze_driver_;
	std::unique_ptr<MapServerInterface> map_server_;	
	std::unique_ptr<AMCLInterface> amcl_;				

	//	Utilities
	rclcpp::TimerBase::SharedPtr heartbeat_timer_;
	rclcpp::TimerBase::SharedPtr mapping_timer_;
	std::unique_ptr<PoseLogger> pose_logger_;		

	geometry_msgs::msg::PoseWithCovarianceStamped last_robot_pose_;		

	// --- Mapping progress ---
	geometry_msgs::msg::Pose start_pose_;
	bool start_pose_set_{false};
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;


	// --- TF2 tools for global position tracking ---
	std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
	std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
	geometry_msgs::msg::TransformStamped start_tf_;
	bool start_tf_set_{false};
	rclcpp::TimerBase::SharedPtr tf_check_timer_;   // periodic distance check

	
	//	Core
	void set_phase(Phase p);
	const char* phase_name(Phase p) const;
	void heartbeat_tick();

	//	Phases
	void start_mapping_phase();
	void end_mapping_phase();
	void start_patrolling_phase();					

	//	fs helper
	void ensure_dir(const std::string &path);
};

} // namespace wh
#endif
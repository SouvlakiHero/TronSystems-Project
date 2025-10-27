//	MTRX3760 2025 Project 2: Warehouse Robot DevKit
//	File: warehouse_manager.cpp
//	Author(s): max rogers

#include "warehouse_manager/warehouse_manager.hpp"
#include <chrono>
#include <cstdlib>
#include <filesystem>

using namespace std::chrono_literals;
namespace fs = std::filesystem;

namespace wh {

// start the manager
WarehouseManager::WarehouseManager(const rclcpp::NodeOptions & options)
: rclcpp::Node("warehouse_manager", options)
{
	// set locations for map and logs to be sent to
	log_dir_ = this->declare_parameter<std::string>("log_dir", "/home/george-anastasiadis/turtlebot3_ws/logs");
	map_dir_ = this->declare_parameter<std::string>("map_dir", "/home/george-anastasiadis/turtlebot3_ws/maps");

	ensure_dir(log_dir_);
	ensure_dir(map_dir_);

	// TF2 listener setup (for localization later on)
	tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
	tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


	RCLCPP_INFO(this->get_logger(), "WarehouseManager started. log_dir='%s', map_dir='%s'",
				log_dir_.c_str(), map_dir_.c_str());

	// initial phase, and start heartbeat
	set_phase(Phase::Idle);
	heartbeat_timer_ = this->create_wall_timer(1s, std::bind(&WarehouseManager::heartbeat_tick, this));
}


void WarehouseManager::ensure_dir(const std::string &path) {
	// check directory exists
	try {
		if (!fs::exists(path)) fs::create_directories(path);
	} catch (...) {
		RCLCPP_WARN(this->get_logger(), "Could not ensure directory exists: %s", path.c_str());
	}
}

void WarehouseManager::set_phase(Phase p) {
	// change phase to new phase
	if (phase_ == p) return;
	auto old = phase_;
	phase_ = p;
	RCLCPP_INFO(this->get_logger(), "Phase change: %s -> %s", phase_name(old), phase_name(phase_));

	// run the corresponding start function for the new phase
	switch (phase_) {
		case Phase::Mapping:		start_mapping_phase(); break;
		case Phase::Patrolling:		start_patrolling_phase(); break;
		default:					break;
	}
}

const char* WarehouseManager::phase_name(Phase p) const {
	// helper to return str version of phase name
	switch (p) {
		case Phase::Idle:		return "Idle";
		case Phase::Mapping:	return "Mapping";
		case Phase::Patrolling:	return "Patrolling";
		default:				return "Unknown";
	}
}

void WarehouseManager::heartbeat_tick() {
	// heartbeat for waiting 3 secs to start mapping phase
	static int ticks = 0;
	++ticks;
	if (ticks == 3) set_phase(Phase::Mapping);
}

void WarehouseManager::start_mapping_phase() {
	// start necessary processes for mapping
	RCLCPP_INFO(this->get_logger(), "Starting Mapping phase...");
	cartographer_ = std::make_unique<CartographerInterface>(this->get_logger());
	maze_driver_ = std::make_unique<MazeDriverInterface>(this->get_logger());
	cartographer_->start();
	maze_driver_->start();

	// Periodically check position in the 'map' frame to see if were back at start
	tf_check_timer_ = this->create_wall_timer(500ms, [this]() {
		try {
			auto tf = tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
			// the first time, log our start 'tf' position so we know when we're back here later
			if (!start_tf_set_) {
				start_tf_ = tf;
				start_tf_set_ = true;
				RCLCPP_INFO(this->get_logger(),
					"Stored start position at (%.2f, %.2f)",
					tf.transform.translation.x, tf.transform.translation.y);
				return;
			}
			
			// get distance from start position
			double dx = tf.transform.translation.x - start_tf_.transform.translation.x;
			double dy = tf.transform.translation.y - start_tf_.transform.translation.y;
			double dist = std::sqrt(dx*dx + dy*dy);

			static auto start_time = this->now();
			double elapsed = (this->now() - start_time).seconds();

			// check if we're back, and its been over 30s
			if (dist < 0.15 && elapsed > 30.0) {	
				RCLCPP_INFO(this->get_logger(),
					"Returned to start (%.2f m away, %.1f s elapsed) — ending mapping.",
					dist, elapsed);
				tf_check_timer_->cancel();
				end_mapping_phase();
			}
		} catch (const tf2::TransformException &ex) {
			RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
				"TF not ready yet: %s", ex.what());
		}
	});


}

void WarehouseManager::end_mapping_phase() {
	if (mapping_timer_) mapping_timer_->cancel();
	RCLCPP_INFO(this->get_logger(), "Ending Mapping phase...");

	//	Save while Cartographer is alive
	const std::string stamp = std::to_string(std::time(nullptr));
	const std::string prefix = map_dir_ + "/map_" + stamp;
	const std::string cmd = "ros2 run nav2_map_server map_saver_cli -f " + prefix;

	RCLCPP_INFO(this->get_logger(), "Saving map to: %s", prefix.c_str());
	system(cmd.c_str());

	//	Remember the YAML for patrolling
	last_map_yaml_path_ = prefix + ".yaml";

	// Mock: record a rough estimate of the current pose										TODO: estimate pos at start of patrol phase from current pose
	// (later we’ll get this from tf2)
	last_robot_pose_.header.frame_id = "map";
	last_robot_pose_.pose.pose.position.x = 0.0;
	last_robot_pose_.pose.pose.position.y = 0.0;
	last_robot_pose_.pose.pose.orientation.w = 1.0;


	//	Stop Cartographer (keep MazeDriver running)
	if (cartographer_) cartographer_->stop();

	set_phase(Phase::Patrolling);
}

void WarehouseManager::start_patrolling_phase() {
	RCLCPP_INFO(this->get_logger(), "Starting Patrolling phase...");
	if (last_map_yaml_path_.empty()) {
		RCLCPP_WARN(this->get_logger(), "No saved map YAML path set; AMCL may fail.");
	}

	//	Map server -> AMCL
	map_server_ = std::make_unique<MapServerInterface>(last_map_yaml_path_, this->get_logger());
	amcl_ = std::make_unique<AMCLInterface>(this->get_logger());

	map_server_->start();
	amcl_->start();

	// Wait for map_server and amcl to appear, then configure + activate
	auto wait_for_node = [](const std::string &name) {
		std::string cmd = "ros2 node list | grep -q " + name;
		while (system(cmd.c_str()) != 0) {
			std::this_thread::sleep_for(std::chrono::milliseconds(200));
		}
	};

	// Wait until both nodes are running
	wait_for_node("map_server");
	wait_for_node("amcl");

	// Configure + activate each
	system("ros2 lifecycle set map_server configure");
	system("ros2 lifecycle set map_server activate");
	system("ros2 lifecycle set amcl configure");
	system("ros2 lifecycle set amcl activate");

	// --- give AMCL its initial pose guess ---
	RCLCPP_INFO(this->get_logger(), "Publishing initial pose to /initialpose");

	// Create a publisher for AMCL's /initialpose topic
	auto pub = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
		"/initialpose", 10);

	// Give AMCL a moment to create its subscriber
	std::this_thread::sleep_for(std::chrono::seconds(1));

	// Fill timestamp and covariance
	last_robot_pose_.header.stamp = this->now();
	last_robot_pose_.pose.covariance.fill(0.0);
	last_robot_pose_.pose.covariance[0] = 0.25;   // variance in x
	last_robot_pose_.pose.covariance[7] = 0.25;   // variance in y
	last_robot_pose_.pose.covariance[35] = 0.0685; // variance in yaw (approx. 15°)

	// Publish the message
	pub->publish(last_robot_pose_);

	RCLCPP_INFO(this->get_logger(), "Initial pose published.");




	//	Log poses every 5 s
	const std::string pose_log = log_dir_ + "/positions_" + std::to_string(std::time(nullptr)) + ".txt";
	pose_logger_ = std::make_unique<PoseLogger>(*this, pose_log);

}

}// namespace wh

//	MTRX3760 2025 Project 2: Warehouse Robot DevKit
//	File: node_interfaces.cpp
//	Author(s): max rogers

#include "warehouse_manager/node_interfaces.hpp"
#include <unistd.h>
#include <csignal>
#include <sys/wait.h>

namespace wh {

ProcessInterface::ProcessInterface(const std::string &name,
								   const std::string &cmd,
								   rclcpp::Logger logger)
: name_(name), cmd_(cmd), logger_(logger) {}

ProcessInterface::~ProcessInterface() { stop(); }

// when starting process, log to terminal and run the bash command 
void ProcessInterface::start() {
	if (pid_ > 0) return;
	RCLCPP_INFO(logger_, "Starting %s process...", name_.c_str());
	pid_ = fork();
	if (pid_ == 0) {
		execl("/bin/bash", "bash", "-c", cmd_.c_str(), (char *)nullptr);
		_exit(1);
	}
}
// handle stopping processes
void ProcessInterface::stop() {
	if (pid_ > 0) {
		RCLCPP_INFO(logger_, "Stopping %s process...", name_.c_str());
		kill(pid_, SIGINT);
		waitpid(pid_, nullptr, 0);
		pid_ = 0;
	}
}

bool ProcessInterface::running() const { return pid_ > 0; }

// each process needs to provide its name, and the bash command/arguments that will be run to start it
CartographerInterface::CartographerInterface(rclcpp::Logger logger)
: ProcessInterface(
	"Cartographer",
	"ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True use_rviz:=True  __log_level:=warn",
	logger) {}

MazeDriverInterface::MazeDriverInterface(rclcpp::Logger logger)
: ProcessInterface(
	"MazeDriver",
	"ros2 run maze_driver maze_driver_node",
	logger) {}

MapServerInterface::MapServerInterface(const std::string &map_yaml, rclcpp::Logger logger)
: ProcessInterface(
	"MapServer",
	"ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=" + map_yaml + " -p use_sim_time:=True",
	logger) {}

AMCLInterface::AMCLInterface(rclcpp::Logger logger)
: ProcessInterface(
	"AMCL",
	"ros2 run nav2_amcl amcl --ros-args -p use_sim_time:=True -p scan_topic:=/scan "
	"-p map_topic:=/map -p odom_frame_id:=odom -p base_frame_id:=base_footprint -p global_frame_id:=map",
	logger) {}

} // namespace wh

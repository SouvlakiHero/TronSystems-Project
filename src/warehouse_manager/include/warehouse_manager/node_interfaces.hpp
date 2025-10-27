//	MTRX3760 2025 Project 2: Warehouse Robot DevKit
//	File: node_interfaces.hpp
//	Author(s): max rogers
//
//	Lightweight wrappers to launch/kill external ROS 2 nodes as subprocesses.


#ifndef WAREHOUSE_MANAGER__NODE_INTERFACES_HPP
#define WAREHOUSE_MANAGER__NODE_INTERFACES_HPP

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <sys/types.h>

namespace wh {

// process interfaces should manage interfacing with a process
// handles starting/stopping new processes
class ProcessInterface {
public:
	ProcessInterface(const std::string &name,
					 const std::string &cmd,
					 rclcpp::Logger logger);
	virtual ~ProcessInterface();

	virtual void start();
	virtual void stop();
	bool running() const;

protected:
	std::string name_;
	std::string cmd_;
	pid_t pid_ = 0;
	rclcpp::Logger logger_;
};

// interface for each process we need to run

class CartographerInterface : public ProcessInterface {
public:
	explicit CartographerInterface(rclcpp::Logger logger);
};

class MazeDriverInterface : public ProcessInterface {
public:
	explicit MazeDriverInterface(rclcpp::Logger logger);
};


class MapServerInterface : public ProcessInterface {
public:
	MapServerInterface(const std::string &map_yaml, rclcpp::Logger logger);
};


class AMCLInterface : public ProcessInterface {
public:
	explicit AMCLInterface(rclcpp::Logger logger);
};

} // namespace wh
#endif

// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: warehouse_manager_node.cpp
// Author(s): max rogers
//
// Node executable that instantiates WarehouseManager.

#include "warehouse_manager/warehouse_manager.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<wh::WarehouseManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

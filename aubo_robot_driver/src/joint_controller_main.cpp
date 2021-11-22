/*******************************************************************************
 *  Copyright (c) Gezp (https://github.com/gezp), All rights reserved.
 *
 *  This program is free software: you can redistribute it and/or modify it 
 *  under the terms of the MIT License, See the MIT License for more details.
 *
 *  You should have received a copy of the MIT License along with this program.
 *  If not, see <https://opensource.org/licenses/MIT/>.
 *
 ******************************************************************************/

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "aubo_robot_driver/joint_controller_node.hpp"

int main(int argc, char * argv[])
{
  // create ros2 node
  rclcpp::init(argc, argv);
  auto node = std::make_shared<aubo_robot_driver::JointControllerNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
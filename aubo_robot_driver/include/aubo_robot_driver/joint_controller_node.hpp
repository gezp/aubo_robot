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
#ifndef AUBO_ROBOT_DRIVER__JOINT_CONTROLLER_HPP_
#define AUBO_ROBOT_DRIVER__JOINT_CONTROLLER_HPP_

#include <mutex>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class ServiceInterface;

namespace aubo_robot_driver {

class JointControllerNode {
public:
  explicit JointControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~JointControllerNode() {};
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface()
  {
    return node_->get_node_base_interface();
  }

private:
  void joint_state_timer_cb();
  void set_joint_state_cb(const sensor_msgs::msg::JointState::SharedPtr msg);

private:
  rclcpp::Node::SharedPtr node_;
  // ros pub and sub
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr set_joint_state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  // robot interface
  std::string ip_{"127.0.0.1"};
  int port_{8899};
  std::shared_ptr<ServiceInterface> robot_service_;
  // data
  sensor_msgs::msg::JointState cur_joint_msg_;

};

}  // namespace aubo_robot_driver

#endif  // AUBO_ROBOT_DRIVER__JOINT_CONTROLLER_HPP_

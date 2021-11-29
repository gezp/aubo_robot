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
#include "trajectory_msgs/msg/joint_trajectory.hpp"

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
  void set_joint_trajectory_cb(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);

private:
  rclcpp::Node::SharedPtr node_;
  // ros pub and sub
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr set_joint_state_sub_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr set_joint_trajectory_sub_;
  rclcpp::TimerBase::SharedPtr joint_state_timer_;
  // robot interface
  std::string ip_{"127.0.0.1"};
  int port_{8899};
  std::shared_ptr<ServiceInterface> robot_service_;
  // data
  std::vector<std::string> joint_names_;
  sensor_msgs::msg::JointState cur_joint_msg_;
  std::vector<double> joint_max_vels_{3, 3, 3, 3, 3, 3};
  std::vector<double> joint_max_accs_{3, 3, 3, 3, 3, 3};
  // rclcpp::Time trajectory_start_time_;
  // std::vector<trajectory_msgs::msg::JointTrajectoryPoint> trajectory_points_;
  // unsigned int trajectory_index_;
  // bool has_trajectory_{false};

};

}  // namespace aubo_robot_driver

#endif  // AUBO_ROBOT_DRIVER__JOINT_CONTROLLER_HPP_

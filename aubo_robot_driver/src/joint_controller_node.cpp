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

#include "aubo_robot_driver/joint_controller_node.hpp"
#include "serviceinterface.h" //机械臂接口

namespace aubo_robot_driver{

JointControllerNode::JointControllerNode(const rclcpp::NodeOptions & options){
  node_ =  std::make_shared<rclcpp::Node>("joint_controller", options);
  // init parameters
  node_->declare_parameter("ip", ip_);
  node_->declare_parameter("port", port_);
  node_->declare_parameter("joint_max_vels", joint_max_vels_);
  node_->declare_parameter("joint_max_accs", joint_max_accs_);
  node_->get_parameter("ip", ip_);
  node_->get_parameter("port", port_);
  node_->get_parameter("joint_max_vels", joint_max_vels_);
  node_->get_parameter("joint_max_accs", joint_max_accs_);
  // init robot service
  robot_service_ = std::make_shared<ServiceInterface>();
  int ret = robot_service_->robotServiceLogin(ip_.c_str(), port_, "aubo", "123456");
  if( ret != aubo_robot_namespace::InterfaceCallSuccCode) {
    RCLCPP_ERROR(node_->get_logger(), "failed to login.");
    return;
  }
  // set max vel of joints
  aubo_robot_namespace::JointVelcAccParam max_joint_limit;
  if (joint_max_vels_.size() != aubo_robot_namespace::ARM_DOF) {
    RCLCPP_ERROR(
      node_->get_logger(), "the size of parameter joint_max_vels must be %d",
      aubo_robot_namespace::ARM_DOF);
    return;
  }
  for(int i = 0; i < aubo_robot_namespace::ARM_DOF; i++){
    max_joint_limit.jointPara[i] = joint_max_vels_[i]; //  180/360*3.14
  }
  ret = robot_service_->robotServiceSetGlobalMoveJointMaxAcc(max_joint_limit);
  // set max acc of joints
  if (joint_max_accs_.size() != aubo_robot_namespace::ARM_DOF) {
    RCLCPP_ERROR(
      node_->get_logger(), "the size of parameter joint_max_accs must be %d",
      aubo_robot_namespace::ARM_DOF);
    return;
  }
  for(int i = 0; i < aubo_robot_namespace::ARM_DOF; i++){
    max_joint_limit.jointPara[i] = joint_max_accs_[i]; //  180/360*3.14
  }
  ret = robot_service_->robotServiceSetGlobalMoveJointMaxVelc(max_joint_limit);
  // init cur_joint_msg_
  joint_names_ = {"shoulder_joint", "upper_arm_joint", "fore_arm_joint",
    "wrist1_joint", "wrist2_joint", "wrist3_joint"};
  cur_joint_msg_.header.frame_id = "aubo_i5";
  for (auto i = 0u; i < joint_names_.size(); ++i) {
    cur_joint_msg_.name.push_back(joint_names_[i]);
    cur_joint_msg_.position.push_back(0);
    cur_joint_msg_.velocity.push_back(0);
    cur_joint_msg_.effort.push_back(0);
  }
  // create ros timer, pub and sub
  auto period_ms = std::chrono::milliseconds(static_cast<int64_t>(1000.0 / 50));
  joint_state_timer_ = node_->create_wall_timer(
    period_ms, std::bind(&JointControllerNode::joint_state_timer_cb, this));
  joint_state_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  set_joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "set_joint_state", 10,
    std::bind(&JointControllerNode::set_joint_state_cb, this, std::placeholders::_1));
  set_joint_trajectory_sub_ = node_->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    "set_joint_trajectory", 10,
    std::bind(&JointControllerNode::set_joint_trajectory_cb, this, std::placeholders::_1));
  set_joint_limit_sub_ = node_->create_subscription<std_msgs::msg::Float32>(
    "set_joint_limit", 10,
    std::bind(&JointControllerNode::set_joint_limit_cb, this, std::placeholders::_1));
  RCLCPP_INFO(node_->get_logger(), "init successfully.");

}

void JointControllerNode::joint_state_timer_cb()
{
  aubo_robot_namespace::JointStatus joint_states[6];
  int ret = robot_service_->robotServiceGetRobotJointStatus(joint_states, 6);
  if(ret != aubo_robot_namespace::InterfaceCallSuccCode){
    return;
  }
  // set position
  cur_joint_msg_.header.stamp = node_->now();
  for(int i = 0; i < 6; i++){
    cur_joint_msg_.position[i] = joint_states[i].jointPosJ;
    cur_joint_msg_.velocity[i] = joint_states[i].jointSpeedMoto;
  }
  // publish
  joint_state_pub_->publish(cur_joint_msg_);
}

void JointControllerNode::set_joint_state_cb(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  int ret;
  double joint_pos[aubo_robot_namespace::ARM_DOF];
  // check
  if (msg->name.size() != joint_names_.size()) {
    return;
  }
  // set position
  for (int i = 0; i < 6; i++) {
    joint_pos[i] = msg->position[i];
  }
  // move
  ret = robot_service_->robotServiceJointMove(joint_pos, false);
  if (ret != aubo_robot_namespace::ErrnoSucc) {
    RCLCPP_ERROR(node_->get_logger(), "set_joint_state_cb error : %d.", ret);
  }
}

void JointControllerNode::set_joint_trajectory_cb(
  const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
  int ret;
  double joint_pos[aubo_robot_namespace::ARM_DOF];
  // check
  if (msg->joint_names.size() != joint_names_.size()) {
    RCLCPP_ERROR(node_->get_logger(), "joint_trajectory msg is invalid (joint size).");
    return;
  }
  // clear trajectory points
  robot_service_->robotServiceClearGlobalWayPointVector();
  // get trajectory points
  auto chain_size = static_cast<unsigned int>(joint_names_.size());
  auto points_size = static_cast<unsigned int>(msg->points.size());
  for (unsigned int i = 0; i < points_size; ++i) {
    for (unsigned int j = 0; j < chain_size; ++j) {
        if (msg->points[i].positions.size() != chain_size) {
          RCLCPP_ERROR(node_->get_logger(), "joint_trajectory msg is invalid (point).");
          return;
        }
        joint_pos[j] = msg->points[i].positions[j];
    }
    ret = robot_service_->robotServiceAddGlobalWayPoint(joint_pos);
  }
  // move
  ret = robot_service_->robotServiceTrackMove(aubo_robot_namespace::move_track::JIONT_CUBICSPLINE, false);
  if (ret != aubo_robot_namespace::ErrnoSucc) {
    RCLCPP_ERROR(node_->get_logger(), "set_joint_trajectory_cb error : %d.", ret);
  }
}

void JointControllerNode::set_joint_limit_cb(const std_msgs::msg::Float32::SharedPtr msg)
{
  // set max vel of joints
  aubo_robot_namespace::JointVelcAccParam max_joint_limit;
  if (joint_max_accs_.size() != aubo_robot_namespace::ARM_DOF) {
    RCLCPP_ERROR(
      node_->get_logger(), "the size of parameter joint_max_accs must be %d",
      aubo_robot_namespace::ARM_DOF);
    return;
  }
  for(int i = 0; i < aubo_robot_namespace::ARM_DOF; i++){
    max_joint_limit.jointPara[i] = joint_max_accs_[i] * msg->data; //  180/360*3.14
  }
  robot_service_->robotServiceSetGlobalMoveJointMaxVelc(max_joint_limit);
}
}




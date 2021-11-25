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
  // parameters
  node_->declare_parameter("ip", ip_);
  node_->declare_parameter("port", port_);
  node_->get_parameter("ip", ip_);
  node_->get_parameter("port", port_);
  // robot service
  robot_service_ = std::make_shared<ServiceInterface>();
  int ret = robot_service_->robotServiceLogin(ip_.c_str(), port_, "aubo", "123456");
  if( ret != aubo_robot_namespace::InterfaceCallSuccCode) {
    RCLCPP_ERROR(node_->get_logger(), "failed to login.");
    return ;
  }
  // init cur_joint_msg_
  joint_names_ = {"shoulder_joint", "upper_arm_joint", "fore_arm_joint",
    "wrist1_joint", "wrist2_joint", "wrist3_joint"};
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
  auto period2_ms = std::chrono::milliseconds(static_cast<int64_t>(1000.0 / 10));
  traj_update_timer_ = node_->create_wall_timer(
    period2_ms, std::bind(&JointControllerNode::traj_update_timer_cb, this));
  joint_state_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  set_joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "set_joint_state", 10,
    std::bind(&JointControllerNode::set_joint_state_cb, this, std::placeholders::_1));
  set_joint_trajectory_sub_ = node_->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    "set_joint_trajectory", 10,
    std::bind(&JointControllerNode::set_joint_trajectory_cb, this, std::placeholders::_1));
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
  for(int i = 0; i < 6; i++){
    cur_joint_msg_.position[i] = joint_states[i].jointPosJ;
    cur_joint_msg_.velocity[i] = joint_states[i].jointSpeedMoto;
  }
  // publish
  joint_state_pub_->publish(cur_joint_msg_);
}

void JointControllerNode::traj_update_timer_cb(){
  if (!has_trajectory_) {
    return;
  }
}

void JointControllerNode::set_joint_state_cb(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  has_trajectory_ = false;
  // int robotServiceSetGlobalMoveJointMaxAcc (const aubo_robot_namespace::JointVelcAccParam &moveMaxAcc)
  // int robotServiceSetGlobalMoveJointMaxVelc (const aubo_robot_namespace::JointVelcAccParam &moveMaxVelc)
  aubo_robot_namespace::JointVelcAccParam max_vel;
  double joint_pos[aubo_robot_namespace::ARM_DOF];
  int ret;
  for(int i = 0; i < 6; i++){
    max_vel.jointPara[i] = 0.5; //  180/360*3.14
  }
  for(int i = 0; i < 6; i++){
    joint_pos[i] = msg->position[i]; //  45/360*3.14
  }
  // ret = robot_service_->robotServiceSetGlobalMoveJointMaxVelc(max_vel);
  ret = robot_service_->robotServiceJointMove(joint_pos, false);
}

void JointControllerNode::set_joint_trajectory_cb(
  const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
  int ret;
  aubo_robot_namespace::JointVelcAccParam max_vel;
  for(int i = 0; i < 6; i++){
    max_vel.jointPara[i] = 3; //  180/360*3.14
  }
  ret = robot_service_->robotServiceSetGlobalMoveJointMaxAcc(max_vel);
  for(int i = 0; i < 6; i++){
    max_vel.jointPara[i] = 3; //  180/360*3.14
  }
  ret = robot_service_->robotServiceSetGlobalMoveJointMaxVelc(max_vel);
  //check
  if (msg->joint_names.size() < joint_names_.size()) {
    return;
  }
  robot_service_->robotServiceClearGlobalWayPointVector();
  //get points
  auto chain_size = static_cast<unsigned int>(joint_names_.size());
  auto points_size = static_cast<unsigned int>(msg->points.size());
  //std::cout<<"get trajectory msg:"<<points_size<<std::endl;
  trajectory_points_.resize(points_size);
  double joint_pos[aubo_robot_namespace::ARM_DOF];
  for (unsigned int i = 0; i < points_size; ++i) {
    for (unsigned int j = 0; j < chain_size; ++j) {
        joint_pos[j] = msg->points[i].positions[j];
    }
    ret = robot_service_->robotServiceAddGlobalWayPoint(joint_pos);
  }
  ret = robot_service_->robotServiceTrackMove(aubo_robot_namespace::move_track::JIONT_CUBICSPLINE, false);
}

}




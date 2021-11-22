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
  aubo_robot_namespace::RobotDiagnosis info;
  while(true){
    memset(&info, 0 ,sizeof(info));
    //接口调用: 获取机械臂诊断信息
    if(robot_service_->robotServiceGetRobotDiagnosisInfo(info) == aubo_robot_namespace::InterfaceCallSuccCode){
      if(info.armPowerStatus == true){
        break;
      }
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
    RCLCPP_ERROR(node_->get_logger(), "diagnosis...");
  }
  // init cur_joint_msg_
  std::vector<std::string> joint_names = {"shoulder_joint","upperArm_joint","foreArm_joint","wrist1_joint","wrist2_joint","wrist3_joint"};
  for (auto i = 0u; i < joint_names.size(); ++i) {
    cur_joint_msg_.name.push_back(joint_names[i]);
    cur_joint_msg_.position.push_back(0);
    cur_joint_msg_.velocity.push_back(0);
    cur_joint_msg_.effort.push_back(0);
  }
  // create ros timer, pub and sub
  auto period_ms = std::chrono::milliseconds(static_cast<int64_t>(1000.0 / 50));
  timer_ = node_->create_wall_timer(
    period_ms, std::bind(&JointControllerNode::joint_state_timer_cb, this));
  joint_state_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  set_joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "set_joint_state", 10,
    std::bind(&JointControllerNode::set_joint_state_cb, this, std::placeholders::_1));
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

void JointControllerNode::set_joint_state_cb(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // int robotServiceSetGlobalMoveJointMaxAcc (const aubo_robot_namespace::JointVelcAccParam &moveMaxAcc)
  // int robotServiceSetGlobalMoveJointMaxVelc (const aubo_robot_namespace::JointVelcAccParam &moveMaxVelc)
  aubo_robot_namespace::JointVelcAccParam max_vel;
  double joint_pos[aubo_robot_namespace::ARM_DOF];
  int ret;
  for(int i = 0; i < 6; i++){
    max_vel.jointPara[i] = 0.4; //  45/360*3.14
  }
  for(int i = 0; i < 6; i++){
    joint_pos[i] = msg->position[i]; //  45/360*3.14
  }
  // ret = robot_service_->robotServiceSetGlobalMoveJointMaxVelc(max_vel);
  // ret = robot_service_->robotServiceJointMove(joint_pos, false);
}

}




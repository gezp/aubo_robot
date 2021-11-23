#!/usr/bin/env python3
import threading

import rclpy
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
import time
from math import sin, cos, pi


#refer https://index.ros.org/doc/ros2/Tutorials/URDF/Using-URDF-with-Robot-State-Publisher/
def euler_to_quaternion(roll, pitch, yaw):
  qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
  qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
  qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
  qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
  return Quaternion(x=qx, y=qy, z=qz, w=qw)

def create_msg(joint_positions):
    msg = JointState()
    for pos in joint_positions:
        msg.name.append('test')
        msg.position.append(pos)
    return msg

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('test_client')
    pub = node.create_publisher(JointState, '/set_joint_state', 10)
    # Set pose goal to reach
    while(1): 
        # move to a
        joint_positions = [1.036, -0.169, -1.493, -1.015, -0.595, -0.046]
        msg = create_msg(joint_positions)
        pub.publish(msg)
        time.sleep(5)
        # move to b
        joint_positions = [1.036, -0.169, -1.493, -1.015, -0.595, 2.8]
        msg = create_msg(joint_positions)
        pub.publish(msg)
        time.sleep(5)
   
    rclpy.shutdown()


if __name__ == "__main__":
    main()
#! /usr/bin/env python3


# Generic python packages
import time  # Time library
import numpy as np
import csv
import yaml
from follower_controller.controller_config.gmpc_ackermann import GeometricMPC_ackermann as gmpc_ackermann

# ROS specific packages
from rclpy.duration import Duration # Handles time for ROS 2
import rclpy # Python client library for ROS 2
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Pose,Twist, TwistStamped # Pose with ref frame and timestamp
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation
from std_msgs.msg import Bool, Float32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from pathlib import Path

class experiment_forward_speed(Node):

    def __init__(self):
        super().__init__('experiment_forward_speed_node')

        # =========================================================
        # Parameters
        # =========================================================
        self.declare_parameter('qcarnumber', 1)
        self.qcarnumber = self.get_parameter('qcarnumber').get_parameter_value().integer_value
        
        self.declare_parameter('speed', 0.0)
        self.speed = self.get_parameter('speed').get_parameter_value().double_value

        # =========================================================
        # State holders / initial values
        # =========================================================
        self.position = Point()
        self.orientation = Quaternion()

        self.position.x = 0.0
        self.position.y = 0.0
        self.position.z = 0.0

        self.step = 0

        self.phi = 0.0
        self.yaw = 0.0

        self.dt = 0.02  # control period
        
        self.FSM = 0  # Finite State Machine: 0 = stop, 1 = run

        self.flag = 1


        self.subscription_vycon = self.create_subscription(
            PoseStamped,
            '/qcar2_1/vrpn_mocap/Qcar2_1/pose',
            self.pose_vycon_callback,
            QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        )

        self.subscription_follower_velocity = self.create_subscription(
            TwistStamped,
            '/qcar2_1/vrpn_mocap/Qcar2_1/twist',
            self.follower_velocity_callback,
            QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        )

        self.timer = self.create_timer(self.dt, self.control_algorithm)


        self.subscription_stop_flag = self.create_subscription(
            Bool,
            '/qcar2_1/vrpn_mocap/Qcar2_1/stop',
            self.stop_experiment_callback,
            10
        )

        self.publisher_start_finish_flag = self.create_publisher(
            Bool,
            '/qcar2_1/experiment_status',
            1)

        self.get_logger().info("Ready to run experiment")

        

    # mocap pose callback
    def pose_vycon_callback(self,msg):
      self.position = msg.pose.position
      orientation = msg.pose.orientation  
      rotation = [orientation.x, orientation.y, orientation.z, orientation.w]
      roll, pitch, self.yaw = Rotation.from_quat(rotation).as_euler('xyz', degrees=False)
    

    def follower_velocity_callback(self, msg):
      vx = msg.twist.linear.x
      vy = msg.twist.linear.y
      omega = msg.twist.angular.z

    def stop_experiment_callback(self, msg: Bool):
      self.FSM = msg.data
      if not self.FSM:
        self.get_logger().info("User called STOP ")
        self.nav_command(0.0,0.0)
      else:
        self.get_logger().info("User called START ")
    

    def control_algorithm(self):

      
      x = np.array([self.position.x, self.position.y, self.yaw, self.current_steering_angle])

      if self.FSM == 1:
        speed_command = 0.0
        self.publisher_start_finish_flag.publish(Bool(data=True))
        if self.step > 50:
            speed_command = self.speed
        if self.step > 150:
            speed_command = 0.0
        if self.step > 200:
           self.FSM = 0
           self.publisher_start_finish_flag.publish(Bool(data=False))
           self.get_logger().info("Experiment finished")
        self.step += 1
      else:    
          speed_command = 0.0
          self.step = 0

      if x[0] > 3.4 or x[0] < -3.4 or x[1] > 2 or x[1] < -2:
          speed_command = 0.0

      self.nav_command(speed_command, 0.0)  # No steering angle for straight line motion

    

    def nav_command(self, speed_command, steering_angle):
      # Create a Twist message
      twist_msg = Twist()
      twist_msg.linear.x = speed_command
      twist_msg.angular.z = steering_angle
      # Publish the message
      self.publisher_.publish(twist_msg)
       
    
      

       


def main():


  # Start the ROS 2 Python Client Library
  rclpy.init()

  node = experiment_forward_speed()
  try:
      rclpy.spin(node)
  except KeyboardInterrupt:
      speed_command = 0.0
      steering_angle = 0.0
      node.nav_command(speed_command,steering_angle)
      
  node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()
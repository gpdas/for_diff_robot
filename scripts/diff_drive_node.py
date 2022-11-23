#!/usr/bin/env python
# author: gpdas
# a node to work with turtlesim
# this node performs forward and inverse kinematics of differential drive robots

import rospy
import turtlesim.msg
import for_diff_robot.msg
import geometry_msgs.msg

class Robot(object):
  def __init__(self, name):
    """Robot object initialiser
    """
    self.name = name
    self.wheel_radius = 0.1 # wheel radius, m
    self.robot_width = 0.8  # robot width, m

    # pose variables
    self.x = 0.0
    self.y = 0.0
    self.theta = 0.0

    # wheel velocity variables
    self.rw_velocity = 0.0
    self.lw_velocity = 0.0

    # message objects for easy publishing
    self.nav_wheel_vel_msg = for_diff_robot.msg.DiffWheelVel()
    self.cmd_vel_msg = geometry_msgs.msg.Twist()

    # pose subscriber - get pose and velocities; convert to wheel velocities; publish nav_wheel_vel
    self.pose_sub = rospy.Subscriber(self.name+"/pose", turtlesim.msg.Pose, self.pose_cb)

    # publisher to send cmd_vel from cmd_wheel_vel 
    self.cmd_vel_pub = rospy.Publisher(self.name+"/cmd_vel", geometry_msgs.msg.Twist)

    # nav_wheel_vel publisher; published from pose
    self.nav_wheel_vel_pub = rospy.Publisher(self.name+"/nav_wheel_vel", for_diff_robot.msg.DiffWheelVel)

    # cmd_wheel_vel subscriber; converts to cmd_vel and publishes it 
    self.cmd_wheel_vel_sub = rospy.Subscriber(self.name+"/cmd_wheel_vel", for_diff_robot.msg.DiffWheelVel, self.cmd_wheel_vel_cb)

  def pose_cb(self, msg):
    """callback for pose msgs
    inverse kinematics to find wheel velocities from robot velocities
    publish wheel velocities as nav_wheel_vel
    """
    rospy.loginfo("---Pose---")
    rospy.loginfo(msg)
    self.x = msg.x
    self.y = msg.y
    self.theta = msg.theta
    self.linear_velocity = msg.linear_velocity
    self.angular_velocity = msg.angular_velocity
    # update wheel velocities
    self.rw_velocity = (self.linear_velocity + self.robot_width*self.angular_velocity)/self.wheel_radius
    self.lw_velocity = (self.linear_velocity - self.robot_width*self.angular_velocity)/self.wheel_radius

    # publish wheel velocities
    self.nav_wheel_vel_msg.rw_velocity = self.rw_velocity
    self.nav_wheel_vel_msg.lw_velocity = self.lw_velocity
    self.nav_wheel_vel_pub.publish(self.nav_wheel_vel_msg)

  def cmd_wheel_vel_cb(self, msg):
    """callback for pose msgs
    forward kinematics to find robot velocities from wheel velocities
    publish robot velocity command as cmd_vel
    """
    rospy.loginfo("---cmd_wheel_vel---")
    rospy.loginfo(msg)
    # convert wheel velocity to cmd_vel
    self.cmd_vel_msg.linear.x = self.wheel_radius*(msg.rw_velocity+msg.lw_velocity)/2.0
    self.cmd_vel_msg.angular.z = self.wheel_radius*(msg.rw_velocity-msg.lw_velocity)/(2.0*self.robot_width)
    self.cmd_vel_pub.publish(self.cmd_vel_msg)


if __name__ == "__main__":
  # initialise the node
  rospy.init_node("diff_drive_node")

  # create a robot object
  robot = Robot("turtle1")

  # spin until exited
  rospy.spin()

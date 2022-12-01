#!/usr/bin/env python
# author: gpdas
# a node to work with turtlesim
# this node performs forward and backward movement of differential drive robots
# to reach a set goal position. The robot is assumed to be moving in straight 
# line (only along x). therefore, the goal position needs only x. different
# controllers could be used to see the performance with different parameter values

import rospy
import turtlesim.msg
import for_diff_robot.msg
import geometry_msgs.msg
import actionlib

import sys
import string
import numpy

class Robot(object):
  def __init__(self, name, controller):
    """Robot object initialiser
    """
    self.name = name
    if controller == 'ONOFF':
      self.controller = self.on_off_control
    elif controller == 'P':
      self.controller = self.p_control
    elif controller == 'PI':
      self.controller = self.pi_control
    elif controller == 'PID':
      self.controller = self.pid_control
      
    self.wheel_radius = 0.1 # wheel radius, m
    self.robot_width = 0.8  # robot width, m
    
    self.linear_velocity = 0.5
    self.angular_velocity = 0.0
    
    self.kp = 0.1
    self.ki = 0.1
    self.kd = 0.1

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
    self.pose_sub = rospy.Subscriber(self.name+'/pose', turtlesim.msg.Pose, self.pose_cb)

    # publisher to send cmd_vel from cmd_wheel_vel 
    self.cmd_vel_pub = rospy.Publisher(self.name+'/cmd_vel', geometry_msgs.msg.Twist, queue_size=5)

    # nav_wheel_vel publisher; published from pose
    self.nav_wheel_vel_pub = rospy.Publisher(self.name+'/nav_wheel_vel', for_diff_robot.msg.DiffWheelVel, queue_size=5)

    self.gtg_success = for_diff_robot.msg.GoToGoalActionResult()
    self.gtg_server = actionlib.SimpleActionServer(self.name+'/go_to_goal',
                                                   for_diff_robot.msg.GoToGoalAction,
                                                   self.go_to_goal,
                                                   False)
    # give some to settle all variables    
    rospy.wait_for_message(self.name+'/pose', turtlesim.msg.Pose, 2)
    
    self.gtg_server.start()
    

  def pose_cb(self, msg):
    """callback for pose msgs
    inverse kinematics to find wheel velocities from robot velocities
    publish wheel velocities as nav_wheel_vel
    """
    #rospy.loginfo('---Pose---')
    #rospy.loginfo(msg)
    self.x = msg.x
    self.y = msg.y
    self.theta = msg.theta
    lin_velocity = msg.linear_velocity
    ang_velocity = msg.angular_velocity
    # update wheel velocities
    self.rw_velocity = (lin_velocity + self.robot_width*ang_velocity)/self.wheel_radius
    self.lw_velocity = (lin_velocity - self.robot_width*ang_velocity)/self.wheel_radius

    # publish wheel velocities
    self.nav_wheel_vel_msg.rw_velocity = self.rw_velocity
    self.nav_wheel_vel_msg.lw_velocity = self.lw_velocity
    self.nav_wheel_vel_pub.publish(self.nav_wheel_vel_msg)

  def on_off_control(self, goal_x, goal_y=0.0, delta=0.1):
    """
    """
    self.cmd_vel_msg.angular.z = 0.0 # no turning
    count = 0
    
    rospy.loginfo('here')
    
    err = goal_x-self.x
    rospy.loginfo(err)
    success = False
      
    while (not rospy.is_shutdown()):
      err = goal_x-self.x
      rospy.loginfo(self.x)
      rospy.loginfo(goal_x)
      rospy.loginfo(err)
      rospy.loginfo('----')
      
      if (err > delta):
        self.cmd_vel_msg.linear.x = self.linear_velocity
        count = 0
      elif (numpy.abs(err) <= delta):
        self.cmd_vel_msg.linear.x = 0.0
        count += 1
      else:
        self.cmd_vel_msg.linear.x = - self.linear_velocity
        count = 0
      
      if count > 3:
        success = True
        break
      else:
        self.cmd_vel_pub.publish(self.cmd_vel_msg)
        rospy.sleep(0.1)
    return success

  def p_control(self, goal_x, goal_y=0.0):
    """
    """
    # TODO
    return False

  def pi_control(self, goal_x, goal_y=0.0):
    """
    """
    # TODO
    return False

  def pid_control(self, goal_x, goal_y=0.0):
    """
    """
    # TODO
    return False

  def go_to_goal(self, goal):
    """go to goal action callback
    """
    rospy.loginfo(goal)
    success = self.controller(goal.x.data)
    if success:
      self.gtg_server.set_succeeded(True)
    else:
      self.gtg_server.set_succeeded(False)
    

if __name__ == '__main__':
  # initialise the node
  if len(sys.argv) < 2:
    print ("""usage is rosrun for_diff_robot for_diff_controllers.py [controller]
           where [controller] should be one among ONOFF, P, PI and PID.
           If no controller option is provided, ONOFF will be selected.
           """)
    controller = 'ONOFF'
  elif string.upper(sys.argv[1]) in ['ONOFF', 'P', 'PI', 'PID']:
    controller = string.upper(sys.argv[1])
  else:
    print ("""usage is rosrun for_diff_robot for_diff_controllers.py [controller]
           where [controller] should be one among ONOFF, P, PI and PID.
           If no controller option is provided, PID will be selected.
           """)
    exit(1)
    
  rospy.init_node('diff_drive_controllers')
  
  # create a robot object
  robot = Robot('turtle1', controller)

  # spin until exited
  rospy.spin()

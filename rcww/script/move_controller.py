#!/usr/bin/env python
#-*-coding:utf-8-*-
# 참조 : https://github.com/wjwwood/pyturtlebot

import sys
import time
import numpy as np
import random
import tf


from math import radians

import rospy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf import transformations as trans
from std_msgs.msg import Bool

class Move_controller():
    max_linear = 1.0
    max_angular = 2.0

    def __init__(self):

        rospy.myargv(argv=sys.argv)
        #로봇을 움직이기 위한 필요한 변수들
        self.__x = None
        self.__y = None
        self.__angle = None
        self.__cumulative_angle = 0.0
        self.__have_odom = False
        
        self.tf_listener = tf.TransformListener()
        
        #라이더가 장애물을 발견하고 발견한 횟수가 5번을 넘어가면 확실히 장애물을 인식한 것으로 취급
        self.__ridar_cnt = 0

        self.on_bumper = None
        self.movement_enabled = True
        
        #주차중에 장애물 여부를 확인하기 위한 변수
        self.chk_obstacle = False

        self.__odom_sub = rospy.Subscriber('/odom', Odometry, self.__odom_handler)
        self.__scan_sub = rospy.Subscriber('/scan', LaserScan, self.__scan_handler)
        self.__cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size = 1)

        self.forward_state = False
        self.backward_state = False
        self.right_state = False
        self.left_state = False
        self.stop_state = False

        self.forward_sub = rospy.Subscriber('tb/control/forward',Bool, self.forward_callback)
        self.backward_sub = rospy.Subscriber('tb/control/backward',Bool, self.backward_callback)
        self.right_sub = rospy.Subscriber('tb/control/right',Bool, self.right_callback)
        self.left_sub = rospy.Subscriber('tb/control/left',Bool, self.left_callback)
        self.stop_sub = rospy.Subscriber('tb/control/stop',Bool, self.stop_callback)
        self.cmd_vel_mux_sub = rospy.Subscriber('cmd_vel_mux/input/teleop', Twist, self.cmd_vel_check_callback)

        self.forward_pub = rospy.Publisher('/tb/control/forward', Bool, queue_size = 1)
        self.backward_pub = rospy.Publisher('/tb/control/backward', Bool, queue_size = 1)
        self.right_pub = rospy.Publisher('/tb/control/right', Bool, queue_size = 1)
        self.left_pub = rospy.Publisher('/tb/control/left', Bool, queue_size = 1)
        self.stop_pub = rospy.Publisher('/tb/control/stop', Bool, queue_size = 1)

    def forward_callback(self,msg):
        self.forward_state = msg.data
        print "self.forward_state", self.forward_state
        if self.forward_state and (not self.stop_state):
            self.go_forward()
            self.forward_state = False
            self.forward_pub.publish(self.forward_state)

        
    def backward_callback(self,msg):
        self.backward_state = msg.data
        print "self.backward_state", self.backward_state
        if self.backward_state and (not self.stop_state):
            self.go_backward()
            self.backward_state = False
            self.backward_pub.publish(self.backward_state)


    def right_callback(self,msg):
        self.right_state = msg.data
        print "self.right_state", self.right_state
        if (self.right_state) and (not self.stop_state) :
            self.turn_right()
            self.right_state = False
            self.right_pub.publish(self.right_state)

    def left_callback(self,msg):
        self.left_state = msg.data
        print "self.left_state", self.left_state
        if self.left_state and (not self.stop_state):
            self.turn_left()
            self.left_state = False
            self.left_pub.publish(self.left_state)

    def stop_callback(self,msg):
        self.stop_state = msg.data
        print "self.stop_state", self.stop_state
        if self.stop_state :
            self.stop()
        
    def cmd_vel_check_callback(self,msg):
        temp_x = msg.linear.x
        temp_l = msg.angular.z
        print "self.temp_state", temp_x, temp_l
        if temp_x==0 and temp_l ==0:
          self.wait(0.6)
          self.stop_state = False
          self.stop_pub.publish(self.stop_state)


    def move(self, linear=0.0, angular=0.0):
        """Moves the robot at a given linear speed and angular velocity

        The speed is in meters per second and the angular velocity is in radians per second

        """
        self.__exit_if_movement_disabled()
        # Bounds checking
        if abs(linear) > self.max_linear:
            linear = self.max_linear if linear > self.max_linear else linear
            linear = -self.max_linear if linear < -self.max_linear else linear
        if abs(angular) > self.max_angular:
            angular = self.max_angular if angular > self.max_angular else angular
            angular = -self.max_angular if angular < -self.max_angular else angular
        # Message generation
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        # Announce and publish
        self.__cmd_vel_pub.publish(msg)

    def move_distance(self, distance, velocity=0.05):
        """Moves a given distance in meters

        You can also give it a speed in meters per second to travel at:

            robot.move_distance(1, 0.5)  # Should take 2 seconds
        """
        
#        print "move_distance entered"
        self.__exit_if_movement_disabled()
        print "move_distance entered, distance : " + str(distance)
        # No bounds checking because we trust people. Not like William.
        r = rospy.Rate(1)
        print "move_distance entered2"
        while not self.__have_odom and not rospy.is_shutdown():
            r.sleep()
        print "move_distance entered3"
        msg = Twist()
        if distance < 0 :
          msg.linear.x = velocity * (-1)
        else : 
          msg.linear.x = velocity
        print "move_distance entered4"
        x0 = self.__x
        y0 = self.__y
        r = rospy.Rate(30)
        print "move_distance entered5"
        while not rospy.is_shutdown():
            d = ((self.__x - x0)**2 + (self.__y - y0)**2)**0.5
            print "move_distance entered6"
            if self.stop_state : 
                break
    
            if d >= abs(distance):
                print "move_distance entered7"
                break

            self.__cmd_vel_pub.publish(msg)
            print "move_distance entered8"
            r.sleep()
        msg.linear.x = 0.0
        print "move_distance entered9"
        self.__cmd_vel_pub.publish(msg)


    def turn_angle(self, angle, velocity=0.22):
        """Turns the robot a given number of degrees in radians

        You can easily convert degress into radians with the radians() function:

            robot.turn_angle(radians(45))  # Turn 45 degrees

        You can also give an angular velocity to turn at, in radians per second:

            robot.turn_angle(radians(-45), radians(45))  # Turn back over a second
        """
        self.__exit_if_movement_disabled()
        # No bounds checking because we trust people. Not like William.
        r = rospy.Rate(1)
        while not self.__have_odom and not rospy.is_shutdown():
            r.sleep()

        msg = Twist()
        if angle >= 0:
            msg.angular.z = np.abs(velocity)
        else:
            msg.angular.z = -np.abs(velocity)
        angle0 = self.__cumulative_angle
        r = rospy.Rate(100)
        while not rospy.is_shutdown():
            if self.stop_state : 
              break
            a_diff = self.__cumulative_angle - angle0
            if (angle > 0 and a_diff >= angle) or (angle < 0 and a_diff <= angle):
              break

            self.__cmd_vel_pub.publish(msg)
            r.sleep()
        msg.angular.z = 0.0
        self.__cmd_vel_pub.publish(msg)


    def wait(self, seconds):
        """This function will wait for a given number of seconds before returning"""
        time.sleep(seconds)

    
    def __odom_handler(self, msg):
        self.__x = msg.pose.pose.position.x
        self.__y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        a = trans.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

        # cumulative angle doesn't wrap. assumes we've not moved more than pi radians
        # since last odom message
        if self.__have_odom:
            a_diff = a - self.__angle
            if a_diff > np.pi:
                a_diff -= 2*np.pi
            elif a_diff < -np.pi:
                a_diff += 2*np.pi
            self.__cumulative_angle += a_diff

        self.__angle = a
        self.__have_odom = True
        
    #로봇기준의 범위 내에 장애물이 존재하는지 확인(__Scan_handler에서 발생)
    def is_ac(self, msg):
      range_len = len(msg.ranges[181:325])
      range_ahead = msg.ranges[181:325]

      for i in range(0,range_len-1) :
        if 0.15 <= round(range_ahead[i],1) <=0.45 : 
          if (245-range_len) <= i <= (315-range_len) :
              return True

    #라이다 장애물 확인
    def __scan_handler(self, msg):
         
          if self.is_ac(msg) :
            self.chk_obstacle = True
            self.__ridar_cnt = self.__ridar_cnt + 1
          else:
            self.chk_obstacle = False
            
          if self.__ridar_cnt > 5:
            self.chk_obstacle = True
          #print self.chk_obstacle
          
    #움직임이 제어가 되지 않을 때 발생
    def __exit_if_movement_disabled(self):
        if not self.movement_enabled:
            self.say("Movement currently disabled")
            sys.exit()

    def stop(self):
        """Stops the robot"""
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.__cmd_vel_pub.publish(msg)


    #로봇이 앞으로 가기위해 명령전달, 약 1m이동
    def go_forward(self):
        self.move_distance(0.1)#forward
        self.wait(0.6)

    #로봇이 뒤로 가기위해 명령전달, 약 1m이동
    def go_backward(self):
        self.move_distance(-0.1)#backward
        self.wait(0.6)

    #로봇이 옆으로(우) 돌기위해 명령전달, 약 90도 회전
    def turn_right(self):
        self.turn_angle(1.58)
        self.wait(0.6)

    #로봇이 옆으로(좌) 돌기위해 명령전달, 약 90도 회전
    def turn_left(self):
        self.turn_angle(-1.58)
        self.wait(0.6)

if __name__ == '__main__':
    rospy.init_node('sign',log_level=rospy.INFO)
    robot = Move_controller()
    rospy.spin()        

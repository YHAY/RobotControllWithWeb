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
        self.movement_enabled = True

        self.__odom_sub = rospy.Subscriber('/odom', Odometry, self.__odom_handler)
        #self.__scan_sub = rospy.Subscriber('/scan', LaserScan, self.__scan_handler)
        self.__cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size = 1)

        #명령에 맞게 움직이기 위해 필요한 변수들
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

    #전진 명령을 받았을 때
    def forward_callback(self,msg):
        self.forward_state = msg.data
        print "self.forward_state", self.forward_state
        if self.forward_state:
            self.go_forward()
            #앞으로 가는 명령을 실행시킨 뒤 다시 상태를 바꿔줘야 다음에 또 사용가능해진다.
            self.forward_state = False
            self.forward_pub.publish(self.forward_state)

    #후진 명령을 받았을 때
    def backward_callback(self,msg):
        self.backward_state = msg.data
        print "self.backward_state", self.backward_state
        if self.backward_state and (not self.stop_state):
            self.go_backward()
            self.backward_state = False
            self.backward_pub.publish(self.backward_state)

    #우회전 명령을 받았을 때
    def right_callback(self,msg):
        self.right_state = msg.data
        print "self.right_state", self.right_state
        if (self.right_state) and (not self.stop_state) :
            self.turn_right()
            self.right_state = False
            self.right_pub.publish(self.right_state)

    #좌회전 명령을 받았을 때
    def left_callback(self,msg):
        self.left_state = msg.data
        print "self.left_state", self.left_state
        if self.left_state and (not self.stop_state):
            self.turn_left()
            self.left_state = False
            self.left_pub.publish(self.left_state)

    #정지 명령을 받았을 때
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

    #직선위를 움직이는 걸 조절하는 부분
    def move_distance(self, distance, velocity=0.05):
        
        self.__exit_if_movement_disabled()
        r = rospy.Rate(1)
        while not self.__have_odom and not rospy.is_shutdown():
            r.sleep()

        msg = Twist()
        if distance < 0 :
          msg.linear.x = velocity * (-1)
        else : 
          msg.linear.x = velocity

        x0 = self.__x
        y0 = self.__y
        r = rospy.Rate(30)

        while not rospy.is_shutdown():
            d = ((self.__x - x0)**2 + (self.__y - y0)**2)**0.5
            if self.stop_state : 
                break
    
            if d >= abs(distance):
                break

            self.__cmd_vel_pub.publish(msg)
            r.sleep()
        msg.linear.x = 0.0
        self.__cmd_vel_pub.publish(msg)

    #회전을 조절하는 부분
    def turn_angle(self, angle, velocity=0.22):
	# 단위는 radian으로 좀 더 섬세하게 작동시키고 싶으면 바로 숫자를 적기보다 radian(45)와 같은 함수를 쓰자!
        self.__exit_if_movement_disabled()
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
            #정지상태가 발생하면 무조건 그만두기
            if self.stop_state : 
              break
            #아직 움직여야하는 회전정도를 구하기 위해서 (돌아야 할 각 - 남은 각)을 계산한다.
            a_diff = self.__cumulative_angle - angle0
            if (angle > 0 and a_diff >= angle) or (angle < 0 and a_diff <= angle):
              break

            self.__cmd_vel_pub.publish(msg)
            r.sleep()
        msg.angular.z = 0.0
        self.__cmd_vel_pub.publish(msg)

    #로봇 작동에 잠깐의 멈춤을 주고 다시 움직이고 싶을 때, 정지와 달리 멈췄다가 다시 동작을 이어서 하는 것이 가능하다.
    def wait(self, seconds):
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

    #로봇이 옆으로(우) 돌기위해 명령전달, 약 45도 회전
    def turn_right(self):
        self.turn_angle(0.79)
        self.wait(0.6)

    #로봇이 옆으로(좌) 돌기위해 명령전달, 약 45도 회전
    def turn_left(self):
        self.turn_angle(-0.79)
        self.wait(0.6)

if __name__ == '__main__':
    rospy.init_node('sign',log_level=rospy.INFO)
    robot = Move_controller()
    rospy.spin()        

#!/usr/bin/env python
import rospy
import move_controller as Robot
import numpy as np


if __name__ == '__main__':
    rospy.init_node('sign',log_level=rospy.INFO)
    robot = Robot.Move_controller()
    robot.stop_behavior()
    rospy.spin()        

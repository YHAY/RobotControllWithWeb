# RobotControllWithWeb

Before starting, you need to install ROS KINETIC.

If you done, you need to install the launch of usb_cam and mjpeg_server.
you can install through this 
-> 1. usb_cam : 
   2. mjpeg_server : https://github.com/RobotWebTools/mjpeg_server.git

Then, you can try this system.

1) $ roscore
2) $ roslaunch roslaunch usb_cam usb_cam-test.launch 
3) $ rosrun mjpeg_server mjpeg_server

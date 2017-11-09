# RobotControllWithWeb

Before starting, you need to install ROS KINETIC.

If you done, you need to install the launch of usb_cam ,mjpeg_server and rosbridge.
you can install through this-> 
   1. usb_cam : https://github.com/ros-drivers/usb_cam.git
   2. mjpeg_server : https://github.com/RobotWebTools/mjpeg_server.git
   3. rosbridge : sudo apt-get install ros-kinetic-rosbridge-server

Then, you can try this system.

1) $ roscore
2) $ roslaunch roslaunch usb_cam usb_cam-test.launch 
3) $ rosrun mjpeg_server mjpeg_server
4) $ roslaunch rosbridge_server rosbridge_websocket.launch

----------------------------------------------------------------------------
without these, you just run the package "rcww". 
it means that you just typing on the screen "$ roslaunch rcww rcww.launch"

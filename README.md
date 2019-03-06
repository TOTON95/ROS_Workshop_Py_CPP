# [Spanish README](https://github.com/TOTON95/ROS_Workshop_Py_CPP/blob/master/README_ES.md)


# ROS_Workshop_Py_CPP
Workshop to teach the basics of ROS

## Instalation

### Create the workspace and move there:

`mkdir -p catkin_ws/src && cd catkin_ws/src`

### Clone this repository: 

`git clone https://github.com/TOTON95/ROS_Workshop_Py_CPP.git`

### Move back to directory's root:

`cd ..`

### Follow the instructions mentioned in [Drawing Robot](https://github.com/TOTON95/Arduino_Drawing_Robot_OpenCV_OpenNI) (build and upload the Arduino Code).

### At the file `ros_py_arm_sender.py` change line 37 to the Arduino's serial port (e.g. /dev/ttyACM0, /dev/ttyUSB0).

### Build the example:

`catkin_make`

### Execute the experiment

`roslaunch ros_workshop_py_cpp ros_cpp_py.launch`





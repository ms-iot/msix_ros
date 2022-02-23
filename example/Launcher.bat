@echo off
:: activate the base installation
call C:\ProgramData\ros\distribution\x64\local_setup.bat

:: activate the additional workspace
:: call C:\ProgramData\install\setup.bat

:: invoke talker_listener example for ROS1
:: roslaunch rospy_tutorials talker_listener.launch

:: invoke talker_listener example for ROS2
:: ros2 launch demo_nodes_cpp talker_listener.launch.py

:: rosrun dynamic_tutorials dynamic_tutorials_node
:: another example to invoke TurtleBot3 SLAM with RViz and Gazebo
:: some environment variables are required for libraries to look for the correct runtime locations

:: set TURTLEBOT3_MODEL=waffle
:: set "CARTOGRAPHER_CONFIG_DIR=C:\ProgramData\ros\melodic\x64\share\cartographer\configuration_files"
:: set "SDF_PATH=C:\ProgramData\rosdeps\x64\share\sdformat\1.6"
:: roslaunch turtlebot3_gazebo turtlebot3_gazebo_cartographer_demo.launch

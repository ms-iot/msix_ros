@echo off
call C:\ProgramData\ros-forge\Scripts\activate.bat C:\ProgramData\ros-forge\envs\dev

ros2 launch demo_nodes_cpp talker_listener.launch.py

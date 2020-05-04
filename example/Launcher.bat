@echo off
:: activate the base installation
call C:\ProgramData\ros\melodic\x64\local_setup.bat

:: activate the additional workspace
:: call C:\ProgramData\install\setup.bat

:: invoke roslaunch
roslaunch rospy_tutorials talker_listener.launch

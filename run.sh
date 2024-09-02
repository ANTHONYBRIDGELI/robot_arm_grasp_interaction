#!/bin/bash
gnome-terminal --tab -- bash -c "catkin_make; exec bash"
sleep 2
gnome-terminal --tab -- bash -c "source devel/setup.bash; roslaunch ar3_moveit demo_gazebo.launch; exec bash"
sleep 2
gnome-terminal --tab -- bash -c "source devel/setup.bash; rosrun yolo detect.py; exec bash"
sleep 2
gnome-terminal --tab -- bash -c "source devel/setup.bash; ./excute/controller; exec bash"
gnome-terminal --tab -- bash -c "source devel/setup.bash; ./excute/grasp_obj; exec bash"
sleep 2
gnome-terminal --tab -- bash -c "source devel/setup.bash; ./excute/get_input; exec bash"

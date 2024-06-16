#!/bin/bash
gnome-terminal -t "car" -- bash -c "cd ~/car;source devel/setup.bash;roslaunch qingzhou_gazebo  qingzhou_bringup.launch"

sleep 20
gnome-terminal -t "qingzhou_0" --tab -- bash -c "cd ~/car;source devel/setup.bash;roslaunch qingzhou_nav  demo_nav.launch  robot_name:=qingzhou_0 initial_pose_x:=14 initial_pose_y:=11 initial_pose_z:=0.1"

sleep 20
gnome-terminal -t "rviz"--tab -- bash -c "cd ~/car;source devel/setup.bash;roslaunch qingzhou_nav  total_show.launch "


sleep 20
gnome-terminal -t "qingzhou_1" --tab -- bash -c "cd ~/car;source devel/setup.bash;roslaunch qingzhou_nav  demo_nav.launch  robot_name:=qingzhou_1 initial_pose_x:=19 initial_pose_y:=34 initial_pose_z:=0.1"


sleep 20
gnome-terminal -t "qingzhou_2" --tab -- bash -c "cd ~/car;source devel/setup.bash;roslaunch qingzhou_nav  demo_nav.launch  robot_name:=qingzhou_2 initial_pose_x:=5 initial_pose_y:=35 initial_pose_z:=0.1"

sleep 20
gnome-terminal -t "qingzhou_3" --tab -- bash -c "cd ~/car;source devel/setup.bash;roslaunch qingzhou_nav  demo_nav.launch  robot_name:=qingzhou_3 initial_pose_x:=5 initial_pose_y:=42 initial_pose_z:=0.1"

sleep 20
gnome-terminal -t "qingzhou_4" --tab -- bash -c "cd ~/car;source devel/setup.bash;roslaunch qingzhou_nav  demo_nav.launch  robot_name:=qingzhou_4 initial_pose_x:=27 initial_pose_y:=8 initial_pose_z:=0.1"

sleep 20
gnome-terminal -t "qingzhou_5" --tab -- bash -c "cd ~/car;source devel/setup.bash;roslaunch qingzhou_nav  demo_nav.launch  robot_name:=qingzhou_5 initial_pose_x:=35 initial_pose_y:=35 initial_pose_z:=0.1"







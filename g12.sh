#!/bin/bash


roslaunch qingzhou_nav  move_car.launch  &
sleep 1
echo "qingzhou_0 success" &




sleep 5
roslaunch qingzhou_nav  move_car1.launch  &
sleep 1
echo "qingzhou_1 success" & 
 

sleep 5
roslaunch qingzhou_nav  move_car2.launch &
sleep 1
echo "qingzhou_2 success" &

sleep 5
roslaunch qingzhou_nav  move_car3.launch  &
sleep 1
echo "qingzhou_3 success"

sleep 5
roslaunch qingzhou_nav  move_car4.launch  &
sleep 1
echo "qingzhou_4 success" &

sleep 5
roslaunch qingzhou_nav  move_car5.launch  &
sleep 1
echo "qingzhou_5 success"





#!/bin/bash


rosnode kill /qingzhou_0/next_goal
rosnode kill /qingzhou_0/path_planner
rosnode kill /qingzhou_0/path_tracker


rosnode kill /qingzhou_1/next_goal
rosnode kill /qingzhou_1/path_planner
rosnode kill /qingzhou_1/path_tracker

rosnode kill /qingzhou_2/next_goal
rosnode kill /qingzhou_2/path_planner
rosnode kill /qingzhou_2/path_tracker

rosnode kill /qingzhou_3/next_goal
rosnode kill /qingzhou_3/path_planner
rosnode kill /qingzhou_3/path_tracker

rosnode kill /qingzhou_4/next_goal
rosnode kill /qingzhou_4/path_planner
rosnode kill /qingzhou_4/path_tracker

rosnode kill /qingzhou_5/next_goal
rosnode kill /qingzhou_5/path_planner
rosnode kill /qingzhou_5/path_tracker

echo "quie sucessfully"

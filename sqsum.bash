#!/usr/bin bash

cd ~/ros2_ws
colcon build --packages-select mypkg

[ $? -eq 0 ] || exit

source ~/.bashrc

timeout 15 ros2 launch mypkg sqsum.launch.py

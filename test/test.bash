#!/bin/bash

# SPDX-FileCopyrightText: 2022 ShinagwaKazemaru
# SPDX-FileCopyrightIdentifer: MIT License

dir=~
[ "$1" != "" ] && dir="$1"

source $dir/.bashrc
cd $dir/ros2_ws
colcon build

[ $? -eq 0 ] || exit

source $dir/.bashrc
timeout 10 ros2 launch mypkg talk_listen.launch.py > /tmp/mypkg.log

cat /tmp/mypkg.log | grep "Listen: 5"

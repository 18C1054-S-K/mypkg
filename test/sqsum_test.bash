#!/bin/bash

# SPDX-FileCopyrightText: 2022 ShinagwaKazemaru
# SPDX-FileCopyrightIdentifer: MIT License

dir=~
[ "$1" != "" ] && dir="$1"

source $dir/.bashrc
cd $dir/ros2_ws
colcon build

[ $? -eq 0 ] || exit 1

source $dir/.bashrc
timeout 20 ros2 launch mypkg sqsum.launch.py > /tmp/mypkg_sqsum.log

cat /tmp/mypkg_sqsum.log | grep "0 = 0^2 + 0^2"
[ $? -eq 0 ] || exit 2
cat /tmp/mypkg_sqsum.log | grep "2 = 1^2 + 1^2"
[ $? -eq 0 ] || exit 3
cat /tmp/mypkg_sqsum.log | grep "3 can't be sum of 2 square numbers"
[ $? -eq 0 ] || exit 4
cat /tmp/mypkg_sqsum.log | grep "4 = 2^2 + 0^2"
[ $? -eq 0 ] || exit 5
cat /tmp/mypkg_sqsum.log | grep "5 = 2^2 + 1^2"
[ $? -eq 0 ] || exit 6
cat /tmp/mypkg_sqsum.log | grep "9 = 3^2 + 0^2"
[ $? -eq 0 ] || exit 10

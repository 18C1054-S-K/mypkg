#!/usr/bin/env python3

# SPDX-FileCopyrightText: 2022 ShinagwaKazemaru
# SPDX-FileCopyrightIdentifer: MIT License

import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

def generate_launch_description():
	talker = launch_ros.actions.Node(package='mypkg', executable='talker')
	result = launch_ros.actions.Node(package='mypkg', executable='result', output='screen')
	sqsum_calculator = launch_ros.actions.Node(package='mypkg', executable='sqsum_calculator')
	prime_factorizer = launch_ros.actions.Node(package='mypkg', executable='prime_factorizer')

	return launch.LaunchDescription([talker, sqsum_calculator, prime_factorizer, result])


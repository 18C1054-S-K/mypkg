#!/usr/bin/env python3

# SPDX-FileCopyrightText: 2022 ShinagwaKazemaru
# SPDX-FileCopyrightIdentifer: MIT License

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16

class Listener():
	def __init__(self, node):
		self.node = node
		self.sub = node.create_subscription(Int16, 'countup', self.cb, 10)


	def cb(self, msg):
		self.node.get_logger().info("Listen: %d" % msg.data)


def main():
	rclpy.init()
	node = Node('listener')
	listener = Listener(node)
	rclpy.spin(node)


if __name__ == '__main__':
	main()

#!/usr/bin/env python3

# SPDX-FileCopyrightText: 2022 ShinagwaKazemaru
# SPDX-FileCopyrightIdentifer: MIT License

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from mypkg_msgs.msg import Primes

class PrimeFactorizer(Node):
	def __init__(self, node_name='prime_factorizer'):
		super().__init__(node_name)
		self.sub = self.create_subscription(Int16, 'countup', self.cb, 10)
		self.pub = self.create_publisher(Primes, 'primes', 10)
		self.primes = [2]


	def cb(self, sub_msg):
		pub_msg = Primes()
		pub_msg.original = sub_msg.data
		pub_msg.primes = []
		pub_msg.indexs = []

		# メッセージの値が2以上なら素因数分解
		# (1以下はprimes、indexsを空のままpublish)
		if sub_msg.data >= 2:
			n = sub_msg.data
			for p in self.primes:
				if n % p == 0:
					cnt = 0
					while n % p == 0:
						n = n / p
						cnt += 1
					pub_msg.primes.append(p)
					pub_msg.indexs.append(cnt)
			if n > 1:
				for i in range(2, n+1):
					if n < i:
						break
					if n % i == 0:
						cnt = 0
						while n % i == 0:
							cnt = 0
							while n % i == 0:
								n = n / i
								cnt += 1
						pub_msg.primes.append(i)
						pub_msg.indexs.append(cnt)
						self.primes.append(i)

		self.pub.publish(pub_msg)


def main():
	rclpy.init()
	pf = PrimeFactorizer()
	rclpy.spin(pf)
	pf.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()

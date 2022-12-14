#!/usr/bin/env python3

# SPDX-FileCopyrightText: 2022 ShinagwaKazemaru
# SPDX-FileCopyrightIdentifer: MIT License

import rclpy
from rclpy.node import Node
from mypkg_msgs.srv import CalcSqSum

class SqSumCalculator(Node):
	def __init__(self, node_name='sqsum_calculator'):
		super().__init__(node_name)
		self.srv = self.create_service(CalcSqSum, 'calc_sq_sum', self.cb)
		self.sqsums = {2:(1,1)} # 計算量削減のため、素数の二平方和の計算結果は保持


	def cb(self, req, res):
		x = 1
		y = 0
		for i,p in enumerate(req.primes):
			if not p in self.sqsums:
				self.sqsums[p] = self.find_sqsums(p)
			(z,w) = self.sqsums[p]
			for j in range(req.indexs[i]):
				(x,y) = self.product(x,y, z,w)
		res.x = x
		res.y = y
		return res


	# (x1^2 + y1^2) * (x2^2 + y2^2)
	def product(self, x1,y1, x2,y2):
		x = x1*x2 - y1*y2
		y = x1*y2 + y1*x2
		if x >= y:
			return (x,y)
		else:
			return (y,x)


	# pの二平方和を総当たりで計算する
	def find_sqsums(self, p):
		x = 1
		y = 0
		for i in range(1,p):
			t = p - i*i
			for j in range(1,t):
				if j*j == t:
					x = j
					y = i
					break
			if not x == 1 and not y == 0:
				break
		return (x,y)


def main():
	rclpy.init()
	ssc = SqSumCalculator()
	rclpy.spin(ssc)
	ssc.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()

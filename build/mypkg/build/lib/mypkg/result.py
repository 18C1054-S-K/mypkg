#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mypkg_msgs.msg import Primes
from mypkg_msgs.srv import CalcSqSum

class Result(Node):
	def __init__(self, node_name='result'):
		super().__init__(node_name)
		self.sub = self.create_subscription(Primes, 'primes', self.cb, 10)
		self.client = self.create_client(CalcSqSum, 'calc_sq_sum')
		self.future = None
		while not self.client.wait_for_service(timeout_sec=2.0): pass


	def cb(self, msg):
		if msg.original < 0:
			self.get_logger().info("%d can't be 2 squares sum" % msg.original)
		elif msg.original == 0:
			self.get_logger().info("0 = 0^2 + 0^2")
		elif msg.original == 1:
			self.get_logger().info("1 = 1^2 + 0^2")
		else:
			isSqSum = True
			c = 1
			primes = []
			indexs = []
			for i,p in enumerate(msg.primes):
				if p % 4 == 3:
					if not msg.indexs[i] % 2 == 0:
						isSqSum = False
						break
					else:
						c *= p ** (msg.indexs[i]/2)
				else:
					primes.append(p)
					indexs.append(msg.indexs[i])

			if isSqSum:
				req = CalcSqSum.Request()
				req.primes = primes
				req.indexs = indexs
				self.future = self.client.call_async(req)
				self.n = msg.original
				self.c = c
			else:
				self.get_logger().info("%d can't be 2 squares sum" % msg.original)


	def on_respond(self, res):
		self.get_logger().info("%d = %d^2 + %d^2" % (self.n, self.c * res.x, self.c * res.y))
		self.future = None


def main():
	rclpy.init()
	r = Result()
	while rclpy.ok():
		rclpy.spin_once(r)
		if not r.future is None and r.future.done():
			try:
				res = r.future.result()
			except Exception as e:
				r.get_logger().info("err %r" % e)
			else:
				r.on_respond(res)
	r.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()

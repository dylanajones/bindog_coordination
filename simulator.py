#!/usr/bin/env python

import random, rospy
from std_msgs.msg import String

class bin(object):

	def __init__(self, loc, capacity = 1.0):
		self.loc = loc
		self.efficiency = .9 * random.random()*.2
		self.bot_assigned = False

	def estimateTimeToFull(self):
		picking_rate = 1 / (30 * 60 * 1)
		return self.capacity / picking_rate

class sim_bindog(object):

	def __init__(self, loc, bot_id):
		self.bot_id = bot_id
		self.loc = loc
		self.status = "idle"
		self.target = None

	def takeAction(self):
		x, y = self.loc

		if self.status != "idle":
			# Test if at the bin location
			# Move horizontal if not at the row, move verticle otherwise untill at the bin
			return 0

class real_bindog(object):

	def __init__(self, loc, pub):
		self.loc = loc
		self.pub = pub

	def giveGoalRow(self, row_num):
		self.pub.publish(row_num)

class simulator(object):

	def __init__(self, row_locs, bin_locs, sim_bindog_locs, bindog_loc, pub):
		self.row_locs = row_locs
		self.bin_locs = bin_locs
		self.real_bindog = real_bindog(bindog_loc, pub)

	def step(self):
		print "stepping"


def callback(msg, args):
	loc = msg.loc
	args[0].real_bindog.loc = loc

if __name__ == '__main__':

	rospy.init_node('Simulator')

	pub = rospy.Publisher('row_goals', String, queue_size = 10)

	row_locs = [[1,1],[2,2],[3,3]]
	bin_locs = [[4,4],[5,5],[6,6]]
	sim_bindog_locs = [[0,0],[0,0]]
	bindog_loc = [0,0]

	sim = simulator(row_locs, bin_locs, sim_bindog_locs, bindog_loc, pub)

	sub = rospy.Subscriber('bindog_loc', String, callback, [sim])

	rate = rospy.Rate(1)

	while not rospy.is_shutdown():
		sim.step()
		rate.sleep()
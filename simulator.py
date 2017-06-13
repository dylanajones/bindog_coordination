#!/usr/bin/env python

import random, rospy

from bindog.msg import mcusensor
from std_msgs.msg import Float32
from std_msgs.msg import Bool

def manhattanDist(p1, p2):
  #This too
  res = 0
  for pair in zip(p1, p2):
    res += abs(pair[1] - pair[0])

  return res

def getFill():
    fill = random.gauss(.8, .1)
    if fill > 1:
        fill = 1

    if fill < 0:
        fill = 0

    return 1 - fill


class bin(object):

    def __init__(self, loc, capacity = 1.0):
        self.loc = loc
        self.efficiency = .9 * random.random()*.2
        self.capacity = capacity
        self.bot_assigned = False
        self.picking_rate = 1.0 / (30. * 60. * 1.)

    def estimateTimeToFull(self):
        return self.capacity / self.picking_rate

    def pickApples(self):
        self.capacity -= self.picking_rate * self.efficiency
        if self.capacity < 0:
            self.capacity = 0

class dummy_bin(object):

	def __init__(self, loc):
		self.loc = loc
		self.capacity = -1

class sim_bindog(object):

    def __init__(self, loc, bot_id):
        self.bot_id = bot_id
        self.loc = loc
        self.status = "idle"
        self.target = None
        self.speed = 1
        self.real = False

    def takeAction(self, sim):
        x, y = self.loc

        if self.status != "idle":
            # Test if at the bin location
            if x == self.target.loc[0] and y == self.target.loc[1]:
            	if self.target.capacity == 0:
            		sim.num_bins_collected += 1
            		self.target.capacity = 1
            		self.target = dummy_bin([self.target.loc[0], 0])
            	else if self.target.capacity == -1:
            		self.status = 'idle'

            if x == self.target.loc[0]:
                if y > self.target.loc[1]:
                    new_y = y - self.speed
                else:
                    new_y = y + self.speed

                if  (y <= self.target.loc[1] <= new_y) or (new_y <= self.target.loc[1] <= y):
                    y = self.target.loc[1]
                else:
                    y = new_y
            else:
                if x > self.target.loc[0]:
                    new_x = x - self.speed
                else:
                    new_x = x + self.speed

                if  (x <= self.target.loc[0] <= new_x) or (new_x <= self.target.loc[0] <= x):
                    x = self.target.loc[0]
                else:
                    x = new_x
            # Move horizontal if not at the row, move verticle otherwise untill at the bin

        self.loc = [x, y]


class real_bindog(object):

    def __init__(self, loc, pub):
        self.loc = loc
        self.pub = pub
        self.target = None
        self.status = "idle"
        self.real = True
        self.speed = 1

    def pubGoalRow(self):
        msg = Float32()

        msg.data = float(self.target.loc[0])
        for i in range(0,10):
            self.pub.publish(msg)

class simulator(object):

    def __init__(self, row_locs, bin_locs, sim_bindog_locs, bindog_loc, pub, cord_method, file_name):
        self.row_locs = row_locs
        self.bin_locs = bin_locs
        self.real_bindog = real_bindog(bindog_loc, pub)
        self.sim_bindogs = []
        self.bins = []
        self.step_num = 0
        self.num_bins_collected = 0

        self.cord_method = cord_method

        for i in range(0,2):
            self.sim_bindogs.append(sim_bindog(sim_bindog_locs[i], i))

        for i in range(0,len(self.bin_locs)):
            self.bins.append(bin(self.bin_locs[i], getFill()))

        self.f = open(file_name, 'w')
        self.writeHeader()

    def step(self):
        self.step_num += 1
        idle_bots = self.getIdleBots()
        if idle_bots != []:
            self.greedyCord(idle_bots)
            #print "doing coordination"

        for bot in self.sim_bindogs:
            bot.takeAction(self)
            #print "taking actions"

        for bin in self.bins:
            bin.pickApples()

        self.writeStep()
        print self.real_bindog.loc

    def getIdleBots(self):
        idle_bots = []

        if self.real_bindog.status == "idle":
            idle_bots.append(self.real_bindog)

        for robot in self.sim_bindogs:
            if robot.status == "idle":
                idle_bots.append(robot)

        return idle_bots

    def greedyCord(self, idle_bots):
        for bot in idle_bots:
            best_bin = self.findBestBin(bot)
            print best_bin.loc
            best_bin.bot_assigned = True
            bot.target = best_bin
            bot.status = "in use"

            if bot.real:
                bot.pubGoalRow()

    def yaweiCord(self, idle_bots):
        for bot in idle_bots:
            best_bin = self.findBestBinFull(bot)

            best_bin.bot_assigned = True
            bot.target = best_bin
            bot.status = "in use"

            if bot.real:
                bot.pubGoalRow()

    def auctionCord(self, idle_bots):
        return 0

    def replanningCord(self, idle_bots):
        for bot in self.sim_bindogs:
        	bot.status = 'idle'

        self.real_bindog.status = 'idle'

        for bin in self.bins:
        	bin.bot_assigned = False

        self.greedyCord(self.getIdleBots())

    def findBestBin(self, bot):
        best_score = float('inf')
        best_bin = None
        for bin in self.bins:
            if bin.bot_assigned == False:
                time = bin.estimateTimeToFull()
                dist = manhattanDist(bot.loc, bin.loc)
                score = max([time, dist / bot.speed])

                if score < best_score:
                    best_score = score
                    best_bin = bin

        return best_bin

    def findBestBinFull(self, bot):
        best_score = float('inf')
        best_bin = None
        for bin in self.bins:
            if bin.bot_assigned == False and bin.capacity == 0:
                score = manhattanDist(bot.loc, bin.loc)

                if score < best_score:
                    best_score = score
                    best_bin = bin

        return best_bin

    def writeHeader(self):
        self.f.write("Start of Header\n")

        self.f.write("Real bindog start: ")
        self.f.write(str(self.real_bindog.loc))
        self.f.write("\n")

        self.f.write("Sim bindog starts: ")
        for robot in self.sim_bindogs:
            self.f.write(str(robot.loc))
            self.f.write(', ')
        self.f.write("\n")

        self.f.write("Bin loc / fill levels: ")
        for bin in self.bins:
            self.f.write(str(bin.loc))
            self.f.write(', ')
            self.f.write(str(bin.capacity))
            self.f.write('| ')
        self.f.write("\n")

        self.f.write("End of Header\n")

    def writeStep(self):
        self.f.write('----------------\n')
        self.f.write('Step num: ')
        self.f.write(str(self.step_num))
        self.f.write('\n')

        self.f.write("Real bindog loc: ")
        self.f.write(str(self.real_bindog.loc))
        self.f.write("\n")

        self.f.write("Sim bindog loc: ")
        for robot in self.sim_bindogs:
            self.f.write(str(robot.loc))
            self.f.write(', ')
        self.f.write("\n")

        self.f.write("Bin loc / fill levels: ")
        for bin in self.bins:
            self.f.write(str(bin.loc))
            self.f.write(', ')
            self.f.write(str(bin.capacity))
            self.f.write('| ')
        self.f.write("\n")





def callback(msg, args):
    loc = [msg.x, msg.y]
    args[0].real_bindog.loc = loc

def resetCallback(msg, args):
    args[0].real_bindog.status = 'idle'
    args[0].num_bins_collected += 1
    args[0].real_bindog.target.capacity = 1

if __name__ == '__main__':

    pub = rospy.Publisher('row_goal', Float32, latch = True, queue_size = 10)



    row_locs = [[1,1],[2,2],[3,3]]
    bin_locs = [[4,4],[5,5],[-6,-6]]
    sim_bindog_locs = [[0,0],[0,0]]
    bindog_loc = [0,0]

    # 1 = greedy, 2 = yawei, 3 = auction, 4 = replanning
    cord_method = 1

    sim = simulator(row_locs, bin_locs, sim_bindog_locs, bindog_loc, pub, cord_method, 'test_file.txt')

    sub = rospy.Subscriber('mcu2ros', mcusensor, callback, [sim])

    sub = rospy.Subscriber('bindog_reset', Bool, resetCallback, [sim])

    rospy.init_node('Simulator')

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        sim.step()
        rate.sleep()

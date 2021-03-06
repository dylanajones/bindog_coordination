#!/usr/bin/env python

import random, rospy, copy

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
    fill = random.gauss(.9, .1)
    if fill > 1:
        fill = 1

    if fill < 0:
        fill = 0

    return 1 - fill


class bin(object):

    def __init__(self, loc, id, row, capacity = 1.0):
        self.loc = loc
        self.efficiency = .9 * random.random()*.2
        self.capacity = capacity
        self.bot_assigned = False
        self.picking_rate = 1.0 / (30. * 6. * 1.)
        self.id = id
        self.row = row

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
        self.speed = 6.4
        self.real = False
        self.update_count = 6

    def takeAction(self, sim):
        x, y = self.loc

        if self.status != "idle":
            # Test if at the bin location
            if x == self.target.loc[0] and y == self.target.loc[1]:
                if self.target.capacity == 0 and self.update_count == 0:
                    sim.num_bins_collected += 1
                    self.target.capacity = 1
                    self.target.bot_assigned = False
                    self.target = dummy_bin([0, 0])
                elif self.target.capacity == 0 and self.update_count != 0:
                    self.update_count -= 1
                elif self.target.capacity == -1:
                    self.status = 'idle'
                    self.update_count = 6

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

    def pubGoalRow(self,sim):
        msg = Float32()

        msg.data = float(float(sim.row_locs[self.target.row][1]))
        print msg.data
        for i in range(0,10):
            self.pub.publish(msg)

class simulator(object):

    def __init__(self, row_locs, bin_locs, sim_bindog_locs, bindog_loc, pub, cord_method, file_name, bin_rows):
        self.row_locs = row_locs
        self.bin_locs = bin_locs
        self.real_bindog = real_bindog(bindog_loc, pub)
        self.sim_bindogs = []
        self.bins = []
        self.step_num = 0
        self.num_bins_collected = 0
        self.stop = False

        self.cord_method = cord_method

        for i in range(0,2):
            self.sim_bindogs.append(sim_bindog(sim_bindog_locs[i], i))

        for i in range(0,len(self.bin_locs)):
            self.bins.append(bin(self.bin_locs[i], i, bin_rows[i], getFill()))

        self.f = open(file_name, 'w')
        self.writeHeader()

    def step(self):
        self.step_num += 1
        idle_bots = self.getIdleBots()
        if idle_bots != []:
            self.yaweiCord(idle_bots)
            #print "doing coordination"

        for bot in self.sim_bindogs:
            bot.takeAction(self)
            #print "taking actions"

        for bin in self.bins:
            bin.pickApples()

        self.writeStep()
        #print self.real_bindog.loc

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
            best_bin, _ = self.findBestBin(bot)
            if best_bin is not None:
                #print best_bin.loc
                best_bin.bot_assigned = True
                bot.target = best_bin
                bot.status = "in use"

                if bot.real:
                    bot.pubGoalRow(self)

    def yaweiCord(self, idle_bots):
        for bot in idle_bots:
            best_bin, _ = self.findBestBinFull(bot)
            if best_bin is not None:
                best_bin.bot_assigned = True
                bot.target = best_bin
                bot.status = "in use"

                if bot.real:
                    bot.pubGoalRow(self)

    def auctionCord(self, idle_bots):
        planning = True
        prev_idle = copy.deepcopy(idle_bots)

        while planning:
            assignment = []
            for bot in idle_bots:
                best_bin, score = self.findBestBin(bot)
                assignment.append([bot, best_bin, score])

            assignment = findNonConflictPlan(assignment)

            for plan in assignment:
                plan[1].bot_assigned = True
                plan[0].target = plan[1]
                plan[0].status = "in use"
                idle_bots.remove(plan[0])

            planning = not(idle_bots == [] or prev_idle == idle_bots)
            prev_idle = copy.deepcopy(idle_bots)


    def replanningCord(self, idle_bots):
        for bot in self.sim_bindogs:
            bot.status = 'idle'

        self.real_bindog.status = 'idle'

        for bin in self.bins:
            bin.bot_assigned = False

        self.auctionCord(self.getIdleBots())

    def findNonConflictPlan(self, plans):
        to_remove = []

        for i, plan in enumerate(plans):
            for j in range(i+1,len(plans)):
                if plans[j] != []:
                    if plan[1] == plans[j][1]:
                        if plan[2] <= plans[j][2]:
                            to_remove.append(plans[j])
                        else:
                            to_remove.append(plan)

        for item in to_remove:
            if item in plans:
                plans.remove(item)

        return plans

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

        return best_bin, best_score

    def findBestBinFull(self, bot):
        best_score = float('inf')
        best_bin = None
        for bin in self.bins:
            if bin.bot_assigned == False and bin.capacity == 0:
                score = manhattanDist(bot.loc, bin.loc)

                if score < best_score:
                    best_score = score
                    best_bin = bin

        return best_bin, best_score

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

        self.f.write("Coordination Method: ")
        self.f.write(str(self.cord_method))
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
            self.f.write(str(bin.bot_assigned))
            self.f.write('| ')
        self.f.write("\n")

        self.f.write("Bins Collected: ")
        self.f.write(str(self.num_bins_collected))
        self.f.write('\n')

def callback(msg, args):
    loc = [msg.x, msg.y]
    args[0].real_bindog.loc = loc

def resetCallback(msg, args):
    print "message recieved"
    args[0].real_bindog.status = 'idle'
    args[0].num_bins_collected += 1
    args[0].real_bindog.target.capacity = 1

def stopCallback(msg, args):
    print "Time to stop"
    args[0].stop = True

if __name__ == '__main__':

    pub = rospy.Publisher('row_goal', Float32, latch = True, queue_size = 10)

    row_locs = [[.64,-6.23],[.67, -9.66],[.83,-13.58],[1.15, -17.27],[1.02, -20.87],[1.20,-24.59],[1.55,-28.37],[1.33, -31.94],[1.61,-35.47],[1.85,-39.08]]
    bin_locs = [[10.07, -4.54],[15.38, -8.03],[13.78,-11.92],[12.13,-15.75],[12.35,-19.15],[12.84,-22.89],[13.22,-26.53],[13.11,-30.07],[14.91,-33.71],[14.16,-37.55]]
    #bin_locs = [[13.78,-11.92]]
    sim_bindog_locs = [[0,0],[0,0]]
    bindog_loc = [0,0]
    bin_rows = [0,1,2,3,4,5,6,7,8,9]
    #bin_rows = [2]

    # 1 = greedy, 2 = yawei, 3 = auction, 4 = replanning
    cord_method = 1

    sim = simulator(row_locs, bin_locs, sim_bindog_locs, bindog_loc, pub, cord_method, 'field_test_day2_yawei3.txt',bin_rows)

    sub = rospy.Subscriber('mcu2ros', mcusensor, callback, [sim])

    sub = rospy.Subscriber('bindog_reset', Bool, resetCallback, [sim])

    sub = rospy.Subscriber('stop_time', Bool, stopCallback, [sim])

    rospy.init_node('Simulator')

    rate = rospy.Rate(0.1)

    while not rospy.is_shutdown() and not sim.stop:
        sim.step()
        rate.sleep()

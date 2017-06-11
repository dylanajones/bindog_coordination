# Coordinator for bindog project

from utils import manhattanDist
from simulator import simulator
import copy

import random



class robotTask(object):
  """docstring for robotTask"""
  def __init__(self, robot_id, locations, tasks):
    self.robot_id = robot_id
    self.locations = locations
    self.tasks = tasks
 
    



class coordinator():

  def __init__ (self, cord_method = 0):
    if cord_method == 0:
      #print "Starting Greedy Coord"
      pass
    elif cord_method == 1:
      #print "Starting Auction Coord"
      pass
    elif cord_method == 2:
      #print "Starting Online replanning Coord"
      pass
    elif cord_method == 3:
      pass
    else:
      print "Invalid ID"
      return 0
    self.cord_method = cord_method

  def cordStep(self, simulator):
    # Command loop

    if self.cord_method == 0:
      # Use the greedy method of coordination
      return self.greedyCord(simulator)
    elif self.cord_method == 1:
      # Use Yawei's method of coordination
      return self.auctionCord(simulator)
    elif self.cord_method == 2:
      # Use a method incorporating online replanning
      return self.replanCord(simulator)
    elif self.cord_method == 3:
      return self.greedyCordYawei(simulator)
    else:
      pass

  def greedyCord(self, simulator):
    #print "Starting Greedy Coordination"

    task_allocation = []

    idle_bots = simulator.getIdleBots()
    assigned_bots = []

    random.shuffle(idle_bots)

    for bot in idle_bots:
      goal = self.findBestBin(bot, [], simulator)
      # print goal
      if goal is not None:
        loc = simulator.bins[goal].loc
        end_loc = [loc[0], 0]
        simulator.bins[goal].bot_assigned = True

        if len(simulator.orchard_map[loc[0]][loc[1]].wkrs) == 0:
          task_allocation.append(robotTask(bot, [loc, end_loc], ['get', 'place']))
        else:
          if simulator.bots[bot].hasBin():
            #task_allocation.append([bot, [loc], ['get']])
            task_allocation.append(robotTask(bot, [loc, end_loc], ['swap', 'place']))
          else:
            # Getting a bin then moving to goal location
            r_loc = simulator.bots[bot].loc
            r_loc = [r_loc[0],0]
            #task_allocation.append([bot, [r_loc,loc],['get', 'get']])
            task_allocation.append(robotTask(bot, [r_loc,loc,end_loc], ['get', 'swap', 'place']))
        assigned_bots.append(bot)

    for bot in assigned_bots:
      idle_bots.remove(bot)

    deliverys = simulator.getBinDeliveryRequests()

    for loc in deliverys:
      if idle_bots != []:
        c_bot = self.findClosestBot(loc, idle_bots, simulator)
        if simulator.bots[c_bot].hasBin():
          task_allocation.append([c_bot, [loc], ['place']])
        else:
          # Getting a bin then moving to goal location
          r_loc = simulator.bots[c_bot].loc
          r_loc = [r_loc[0],0]
          task_allocation.append(robotTask(c_bot, [r_loc,loc],['get', 'place']))
        
        wkrs = simulator.orchard_map[loc[0]][loc[1]].wkrs
        for worker in wkrs:
          simulator.wkrs[worker].request_akn = True
          simulator.wkrs[worker].delivery = False

        idle_bots.remove(c_bot)

    return task_allocation

  def greedyCordYawei(self, simulator):
    #print "Starting Greedy Coordination"

    task_allocation = []

    idle_bots = simulator.getIdleBots()
    assigned_bots = []

    random.shuffle(idle_bots)

    for bot in idle_bots:
      goal = self.findBestBinFull(bot, [], simulator)
      # print goal
      if goal is not None:
        loc = simulator.bins[goal].loc
        end_loc = [loc[0], 0]
        simulator.bins[goal].bot_assigned = True

        if len(simulator.orchard_map[loc[0]][loc[1]].wkrs) == 0:
          task_allocation.append(robotTask(bot, [loc, end_loc], ['get', 'place']))
        else:
          if simulator.bots[bot].hasBin():
            #task_allocation.append([bot, [loc], ['get']])
            task_allocation.append(robotTask(bot, [loc, end_loc], ['swap', 'place']))
          else:
            # Getting a bin then moving to goal location
            r_loc = simulator.bots[bot].loc
            r_loc = [r_loc[0],0]
            #task_allocation.append([bot, [r_loc,loc],['get', 'get']])
            task_allocation.append(robotTask(bot, [r_loc,loc,end_loc], ['get', 'swap', 'place']))
        assigned_bots.append(bot)

    for bot in assigned_bots:
      idle_bots.remove(bot)

    deliverys = simulator.getBinDeliveryRequests()

    for loc in deliverys:
      if idle_bots != []:
        c_bot = self.findClosestBot(loc, idle_bots, simulator)
        if simulator.bots[c_bot].hasBin():
          task_allocation.append([c_bot, [loc], ['place']])
        else:
          # Getting a bin then moving to goal location
          r_loc = simulator.bots[c_bot].loc
          r_loc = [r_loc[0],0]
          task_allocation.append(robotTask(c_bot, [r_loc,loc],['get', 'place']))
        
        wkrs = simulator.orchard_map[loc[0]][loc[1]].wkrs
        for worker in wkrs:
          simulator.wkrs[worker].request_akn = True
          simulator.wkrs[worker].delivery = False

        idle_bots.remove(c_bot)

    return task_allocation

  def findBestBin(self, bot, exclude_bins, simulator):
    bins = simulator.bins # get the dictionary of bins
    best_bin = None
    best_score = float('inf')

    for bin in bins:
      if not(bins[bin].loc in exclude_bins) and not(bins[bin].bot_assigned):
        if self.binScore(bot, bin, simulator) < best_score:
          best_bin = bin
          best_score = self.binScore(bot, bin, simulator)

    return best_bin

  def findBestBinFull(self, bot, exclude_bins, simulator):
    bins = simulator.bins # get the dictionary of bins
    best_bin = None
    best_score = float('inf')

    for bin in bins:
      if not(bins[bin].loc in exclude_bins) and not(bins[bin].bot_assigned):
        if bins[bin].capacity < 0.01:
          if self.binScore(bot, bin, simulator) < best_score:
            best_bin = bin
            best_score = self.binScore(bot, bin, simulator)

    if best_bin is None:

        best_bin = self.findBestBin(bot, exclude_bins, simulator)

    return best_bin



  def binScore(self, bot, bin, simulator):
    score = manhattanDist(simulator.bins[bin].loc, simulator.bots[bot].loc)

    score += max([simulator.bins[bin].estimateTimeToFull()-score,0])

    return score


  def auctionCord(self, simulator):
    task_allocation = []

    #Get idle bots, bins needing pickup, locations needing bin delivery
    idle_bots = simulator.getIdleBots() 
    deliverys = simulator.getBinDeliveryRequests() 

    # While still robots without plans
    still_planning = True
    prev_idle = copy.deepcopy(idle_bots)

    while still_planning:
      
      plans = []
      for bot in idle_bots:
        plans.append(self.getRobotPlan(bot, simulator))

      if len(plans) > 1:
        plans = self.findNonConflictPlan(plans)


      for plan in plans:
        if plan != []:
          c_bot = plan[0]
          loc = simulator.bins[plan[1]].loc
          end_loc = [loc[0], 0]
          simulator.bins[plan[1]].bot_assigned = True
          idle_bots.remove(c_bot)
          r_loc = simulator.bots[c_bot].loc
          r_loc = [r_loc[0],0]
          if len(simulator.orchard_map[loc[0]][loc[1]].wkrs) == 0:
            if simulator.bots[c_bot].hasBin():
              task_allocation.append(robotTask(bot, [simulator.bots[c_bot].loc, loc, end_loc], ['place', 'get', 'place']))
            else:
              task_allocation.append(robotTask(bot, [loc, end_loc], ['get', 'place']))
          else:
            if simulator.bots[c_bot].hasBin():
              #print "condition 2"
              task_allocation.append(robotTask(c_bot, [loc, end_loc], ['swap', 'place']))
            else:
              # Getting a bin then moving to goal location
              task_allocation.append(robotTask(c_bot, [r_loc,loc,end_loc],['get', 'swap', 'place']))


      still_planning = not(idle_bots == [] or prev_idle == idle_bots)
      prev_idle = copy.deepcopy(idle_bots)

    for loc in deliverys:
      if idle_bots != []:
        c_bot = self.findClosestBot(loc, idle_bots, simulator)
        if simulator.bots[c_bot].hasBin():
          task_allocation.append(robotTask(c_bot, [loc], ['place']))
        else:
          # Getting a bin then moving to goal location
          r_loc = simulator.bots[c_bot].loc
          r_loc = [r_loc[0],0]
          task_allocation.append(robotTask(c_bot, [r_loc,loc],['get', 'place']))
        wkrs = simulator.orchard_map[loc[0]][loc[1]].wkrs
        for worker in wkrs:
          simulator.wkrs[worker].request_akn = True
          simulator.wkrs[worker].delivery = False
        #print task_allocation[0].tasks
        idle_bots.remove(c_bot)



    return task_allocation


  def resetBots(self,simulator):
    #only reset idle bots, or all bots? Maybe just safer to do all bots, the idle bots case is commented below
    for botID in simulator.bots:
      if simulator.bots[botID].bin is not None:
        if(simulator.bins[simulator.bots[botID].bin].capacity == 1):#todo: is this a correct condition?
      #this becomes a little bit complex because repalnning doesnt' really helo because robots are onyl idle when they don't have bins
          simulator.bots[botID].status="idle"
          simulator.bots[botID].target=None
          simulator.bots[botID].plan=[]
      else:
          simulator.bots[botID].status="idle"
          simulator.bots[botID].target=None
          simulator.bots[botID].plan=[]
      # elif(simulator.bots[botID].bin!=None):
      #   if (simulator.bins[simulator.bots[botID].bin].capacity==1.):
    
      #     simulator.bots[botID].status="idle"
      #     simulator.bots[botID].target=None
      #     simulator.bots[botID].plan=[]
      # print simulator.bots[botID].bin
      # raw_input()

      # else:
      #   simulator.bots[botID].status="idle"
      #   simulator.bots[botID].target=None
      #   simulator.bots[botID].plan=[]

    # idle_bots = simulator.getIdleBots() 
    # bots_to_reset=[]
    # for bot in range(len(simulator.bots)):
    #   if(bot not in idle_bots):
    #     bots_to_reset.append(bot)

    # # print "To reset:",bots_to_reset

    # #reset all of the non-idle bots
    # for botID in bots_to_reset:
    #   simulator.bots[botID].status="idle"
    #   simulator.bots[botID].target=None
    #   simulator.bots[botID].bin=None
    #   simulator.bots[botID].plan=[]
    # print "All the bots that are idle: ",simulator.getIdleBots()

  def resetBins(self,simulator):
    for binID in simulator.bins:
      simulator.bins[binID].bot_assigned=False

  def resetRequests(self,simulator):
    deliveries = simulator.getBinDeliveryRequests() 

  def resetAck(self,simulator):
    #review how we want to reset the worker groups
    for worker in simulator.wkrs:
      #keep pickup as whatever it is?
      # self.pickup = False
      #keep delivery the same?
      # worker.delivery = False

      simulator.wkrs[worker].request_akn = False
    

  def replanCord(self, simulator):
    task_allocation = []

    #Reset idle bots, bins needing pickup, locations needing bin delivery,worker_request_acknowledgments
    self.resetBots(simulator)
    self.resetBins(simulator)
    self.resetRequests(simulator)
    self.resetAck(simulator)

    #now this should just be returning all of the bins, this doesn't seem to be returning anything?
    


    return self.auctionCord(simulator)


  def getRobotPlan(self, bot, simulator):
    # score each plan by travel and wait time

    best_bin = self.findBestBin(bot, [], simulator)
    if best_bin is not None:
      return [bot, best_bin, self.binScore(bot, best_bin, simulator)]
    else:
      return []

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


  def findClosestBot(self, loc, bots, simulator):
    # Probably need to write different distance function - not manhattan but up and down + sideways
    dist = float('inf')
    c_bot = None

    for item in bots:

      d = manhattanDist(loc, simulator.bots[item].loc)

      if d < dist:
        dist = d
        c_bot = item

    return c_bot

if __name__ == '__main__':

  num_rows = 10
  row_size = 10
  num_bots = 5
  num_bins = 0
  num_wkrs = 4
  sim = simulator(num_rows, row_size, num_bots, num_bins, num_wkrs)

  sim.wkrs[0].delivery = True

  cord = coordinator(cord_method=1)

  plans = cord.cordStep(sim)

  print "displaying the plans for the robots"
  for plan in plans:
    print plan

  sim.drawSimulator()

    # def greedyCord(self, simulator):
    
  #   print "Starting Greedy Coordination"
  #   # Greedily choose closest robot to each task needed to be completed

  #   task_allocation = []

  #   # Get idle bots, bins needing pickup, locations needing bin delivery
  #   idle_bots = simulator.getIdleBots()
  #   print idle_bots
  #   #bins = simulator.bins # We consider all bins
  #   pickups = simulator.getBinPickupRequests() 
  #   deliverys = simulator.getBinDeliveryRequests()

  #   # Loop through all pickup locations and assign then
  #   for loc in pickups:
      
  #     if idle_bots != []:
  #       c_bot = self.findClosestBot(loc, idle_bots, simulator)
  #       print simulator.bots[c_bot].loc
  #       print loc
  #       if simulator.bots[c_bot].hasBin():
  #         task_allocation.append([c_bot, [loc], ['get']])
  #       else:
  #         # Getting a bin then moving to goal location
  #         r_loc = simulator.bots[c_bot].loc
  #         r_loc = [r_loc[0],0]
  #         task_allocation.append([c_bot, [r_loc,loc],['get', 'get']])
  #       print "removing bot ", c_bot
  #       idle_bots.remove(c_bot)
  #       print idle_bots

  #   # Loop through all idle bots and find bins to go to
  #   random.shuffle(idle_bots)
  #   for bot in idle_bots:
  #     goal = self.findBestBin(bot, pickups, simulator)
  #     print goal
  #     if goal is not None:
  #       loc = simulator.bins[goal].loc
  #       simulator.bins[goal].bot_assigned = True
  #       if simulator.bots[bot].hasBin():
  #         task_allocation.append([c_bot, [loc], ['get']])
  #       else:
  #         # Getting a bin then moving to goal location
  #         r_loc = simulator.bots[c_bot].loc
  #         r_loc = [r_loc[0],0]
  #         task_allocation.append([c_bot, [r_loc,loc],['get', 'get']])
  #       print "removing bot ", bot
  #       idle_bots.remove(bot)
  #       print idle_bots

  #   # Loop through all delivery locations and assign then
  #   for loc in deliverys:
      
  #     if idle_bots != []:
  #       c_bot = self.findClosestBot(loc, idle_bots, simulator)
  #       print simulator.bots[c_bot].loc
  #       print loc
  #       if simulator.bots[c_bot].hasBin():
  #         task_allocation.append([c_bot, [loc], ['place']])
  #       else:
  #         # Getting a bin then moving to goal location
  #         r_loc = simulator.bots[c_bot].loc
  #         r_loc = [r_loc[0],0]
  #         task_allocation.append([c_bot, [r_loc,loc],['get', 'place']])
  #       print "removing bot ", c_bot
  #       idle_bots.remove(c_bot)
  #       print idle_bots


  #   return task_allocation
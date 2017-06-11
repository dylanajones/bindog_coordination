
import matplotlib.pyplot as plt
import matplotlib.patches as patches

import random, copy, operator

from utils import safeMax, manhattanDist


class simulator(object):
  """docstring for simulator"""
  def __init__(self, num_rows, row_size, num_bots, num_bins, num_wkrs):
    #random init

    self.num_rows = num_rows
    self.row_size = row_size
    self.total_apples = 0

    self.orchard_map = [[None for ii in range(row_size+2)] for jj in range(3*self.num_rows)]

    for row_id, row in enumerate(self.orchard_map):
      for spot_id, spot in enumerate(row):
        if spot_id == 0:
          #self.orchard_map[row_id][spot_id] = "depot"
          self.orchard_map[row_id][spot_id] = cell("depot")
        elif spot_id == len(row)-1:
          self.orchard_map[row_id][spot_id] = cell("headlands")
          #self.orchard_map[row_id][spot_id] = "headlands"
        else:
          if row_id%3 == 1:
            self.orchard_map[row_id][spot_id] = cell("path")
            #self.orchard_map[row_id][spot_id] = "orchard"
          else:
            self.orchard_map[row_id][spot_id] = cell("orchard")
            self.total_apples += self.orchard_map[row_id][spot_id].apples
            #self.orchard_map[row_id][spot_id] = "path"

    self.wkrs = {}
    self.bots = {}
    self.bins = {}

    self.num_bins = num_bins
    self.collected_bins = 0
    self.profit = 0
    self.apples_picked = 0
    self.wasted_time = 0

    # for ii in range(num_wkrs):
    while len(self.wkrs) < num_wkrs:
      worker_loc = [random.randint(0, num_rows-1)*3+1, 1+random.randint(0, row_size-1)]
      for worker in self.wkrs:
        if worker_loc == self.wkrs[worker].loc:
          break
      else:
        worker_id = len(self.wkrs)
        self.wkrs[worker_id] = workerGroup(worker_loc)
        self.orchard_map[worker_loc[0]][worker_loc[1]].wkrs.append(worker_id) 

    while len(self.bots) < num_bots:
      bot_loc = [random.randint(0, num_rows*2), 0]
      for bot in self.bots:
        if bot_loc == self.bots[bot].loc:
          break
      else:
        bot_id = len(self.bots)
        self.bots[bot_id] = bindog(bot_loc, bot_id)
        self.orchard_map[bot_loc[0]][bot_loc[1]].bots.append(bot_id)

    for bin_id, worker in enumerate(self.wkrs):
      bin_loc = [x for x in self.wkrs[worker].loc]
      self.bins[bin_id] = orchardBin(bin_loc)
      self.orchard_map[bin_loc[0]][bin_loc[1]].bins.append(bin_id)


  def applesLeft(self):
    apples_left = 0
    for row_id, row in enumerate(self.orchard_map):
      for spot_id, spot in enumerate(row):
        if self.orchard_map[row_id][spot_id].terrain == "orchard":
          apples_left += self.orchard_map[row_id][spot_id].apples
    return apples_left


  def pickFruit(self, worker):
    worker_obj = self.wkrs[worker]
    x, y = worker_obj.loc
    current_cell = self.orchard_map[x][y]

    if len(current_cell.bins) > 0:
      current_bin = self.bins[current_cell.bins[0]]
      #max_picked = min((.1 + .1 * random.random())*worker_obj.efficiency, current_bin.capacity)
      max_picked = min((.45 + .05 * random.random())*worker_obj.efficiency, current_bin.capacity)
      #max_picked = min(.2, self.bins[current_cell.bins[0]].capacity)

      total_apples_picked = 0
      for side in [1, -1]:
        apples_picked_side = 0
        max_picked_side = max_picked/2
      
        orchard_apples_picked = min(self.orchard_map[x+side][y].apples, max_picked_side)
        apples_picked_side += orchard_apples_picked
        self.orchard_map[x+side][y].apples -= orchard_apples_picked

        if apples_picked_side < max_picked_side:
          side_remaining = max_picked_side - apples_picked_side
          for offset in [-1, 1]:
            if self.orchard_map[x+side][y+offset].terrain == "orchard":
              orchard_apples_picked = min(self.orchard_map[x+side][y+offset].apples, side_remaining/2)
              apples_picked_side += orchard_apples_picked
              self.orchard_map[x+side][y+offset].apples -= orchard_apples_picked
        total_apples_picked += apples_picked_side
      current_bin.capacity -= total_apples_picked
      if total_apples_picked == 0:
        self.wasted_time += 1

      if total_apples_picked == 0 and current_bin.capacity != 0:
        loc = self.findNearApples(worker)
        if loc is not None:
          self.orchard_map[self.wkrs[worker].loc[0]][self.wkrs[worker].loc[1]].wkrs.remove(worker)
          self.wkrs[worker].loc = loc
          self.orchard_map[loc[0]][loc[1]].wkrs.append(worker)
    else:
      if not(self.wkrs[worker].request_akn):
        self.wkrs[worker].delivery = True
      else:
        pass

  def findNearApples(self, worker):
    #print "worker number: ", worker
    wkr_loc = self.wkrs[worker].loc

    best_loc = None
    best_dist = float("inf")

    for row_id, row in enumerate(self.orchard_map):
      for spot_id, spot in enumerate(row):
        if self.orchard_map[row_id][spot_id].terrain == "path":
          if (self.orchard_map[row_id-1][spot_id].apples + self.orchard_map[row_id+1][spot_id].apples) > 0:
            dist = manhattanDist([row_id, spot_id],wkr_loc)
            if dist < best_dist:
              best_dist = dist
              best_loc = [row_id, spot_id]
    return best_loc


  def createBin(self, loc):
    bin_id = safeMax(self.bins.keys())+1
    self.bins[bin_id] = orchardBin(loc)
    self.orchard_map[loc[0]][loc[1]].bins.append(bin_id)
    return bin_id

  def step(self):

    for bot in self.bots:
      bot = self.bots[bot]
      if bot.plan != []:
        bot.takeAction(bot.plan[0], self)
      else:
        bot.status = 'idle'

    for worker in self.wkrs:
      self.pickFruit(worker)

    to_remove = []
    for bin in self.bins:
      if not(self.bins[bin].bot_assigned) and self.orchard_map[self.bins[bin].loc[0]][self.bins[bin].loc[1]].terrain == 'depot':
        # Remove the bin
        # Add functionality here to count apples picked / bins used
        to_remove.append(bin)
    
    for bin in to_remove:
      if len(self.orchard_map[self.bins[bin].loc[0]][self.bins[bin].loc[1]].bins) > 0:
        self.apples_picked += (1 - self.bins[bin].capacity)
        if bin in self.orchard_map[self.bins[bin].loc[0]][self.bins[bin].loc[1]].bins:
          self.orchard_map[self.bins[bin].loc[0]][self.bins[bin].loc[1]].bins.remove(bin)
          del self.bins[bin]



  def drawSimulator(self):
    scale = 10
    #img_size = [(self.num_rows*2+1)*scale, self.row_size+2*scale]
    fig1 = plt.figure(1)
    plt.clf()
    ax1 = plt.gca()

    plt.xlim((-.5, (self.num_rows*3-.5)))
    plt.ylim((-.5, self.row_size+1.5))

    for row_id, row in enumerate(self.orchard_map):
      for col_id, item in enumerate(row):
        if item.terrain == "orchard":
          ax1.add_patch(patches.Rectangle((row_id-.5, col_id-.5), 1., 1., facecolor="#228b22"))
          #plt.text(row_id, col_id, str(item.apples)[:4])
        elif item.terrain == "headlands":
          ax1.add_patch(patches.Rectangle((row_id-.5, col_id-.5), 1., 1., facecolor="#776b46"))
        elif item.terrain == "depot":
          ax1.add_patch(patches.Rectangle((row_id-.5, col_id-.5), 1., 1., facecolor="#ffc000"))


    for bot in self.bots:
      drawBindog(ax1, self.bots[bot])
      plt.text(self.bots[bot].loc[0]-.1, self.bots[bot].loc[1]-.05, bot)

    for worker in self.wkrs:
      loc = self.wkrs[worker].loc
      ax1.add_patch(patches.Rectangle((loc[0]-.45, loc[1]-.45), .9, .9, facecolor="#c0ffee"))

    for bin in self.bins:
      drawBin(ax1, self.bins[bin])

    plt.show(block=False)
    plt.pause(0.05)


  def getIdleBots(self):
    # Function which returns a list of idle robots in the simulator
    idle_bots = []

    for key, item in self.bots.iteritems():
      if item.status == "idle":
        idle_bots.append(key)

    return idle_bots

  def getBinPickupRequests(self):
    res = []

    for worker in self.wkrs:
      if self.wkrs[worker].pickup:
        res.append(self.wkrs[worker].loc)

    return res


  def getBinDeliveryRequests(self):
    res = []

    for worker in self.wkrs:
      if self.wkrs[worker].delivery:
        res.append(self.wkrs[worker].loc)
        
    return res


def drawBin(ax1, bin_in):
  loc = bin_in.loc
  ax1.add_patch(patches.Rectangle((loc[0]-.3, loc[1]-.3), .6, .6, facecolor="purple"))

def drawBindog(ax1, bot):
  loc = bot.loc
  ax1.add_patch(patches.Rectangle((loc[0]-.25, loc[1]-.25), .5, .5, facecolor="#7b9095"))
  ax1.add_patch(patches.Circle((loc[0]-.25, loc[1]-.25), .1, color='k'))
  ax1.add_patch(patches.Circle((loc[0]+.25, loc[1]-.25), .1, color='k'))
  if bot.bin is not None:
    ax1.add_patch(patches.Rectangle((loc[0]-.2, loc[1]-.2), .4, .4, facecolor="k"))




class cell(object):
  """docstring for cell"""
  def __init__(self, terrain):
    self.terrain = terrain
    
    if self.terrain is not 'orchard':
      self.bins = []
      self.wkrs = []
      self.bots = []
      self.apples = None
    else:
      self.bins = None
      self.bots = None
      self.wkrs = None
      self.apples = .9 + .2*random.random()


class bindog(object):
  """docstring for bindog"""
  def __init__(self, loc, bot_id):
    self.bot_id = bot_id
    self.loc = loc
    self.status = "idle" #robot starts idle
    self.target = None #Default no target
    self.bin = None #Robot Starts with No Bin
    self.plan = []


  def takeAction(self, action, sim):
    x, y = self.loc
    if action == "NORTH":
      proceed = True

      for map_cell in sim.orchard_map[x]:
        #print map_cell.terrain, len(map_cell.bots)
        if map_cell.terrain == "path":
          if len(map_cell.bots) > 0 and self.bot_id not in map_cell.bots:
            proceed = False
      if proceed:
        self.moveBot([0, 1], sim)
        self.plan = self.plan[1:]
      #go North
      return 0
    elif action == "SOUTH":
      self.moveBot([0, -1], sim)
      self.plan = self.plan[1:]
      #go South
      return 0
    elif action == "EAST":
      self.moveBot([1, 0], sim)
      self.plan = self.plan[1:]
      #go East
      return 0
    elif action == "WEST":
      self.moveBot([-1, 0], sim)
      self.plan = self.plan[1:]
      #go West
      return 0
    elif action == "PLACE":
      self.placeBin(sim)
      self.plan = self.plan[1:]
      return 0
    elif action == "GET":
      self.getBin(sim)
      self.plan = self.plan[1:]
      return 0
    elif action == "SWAP":
      if len(sim.orchard_map[x][y].bins) != 0:
        
        if (sim.bins[sim.orchard_map[x][y].bins[0]].capacity < .01) or (len(sim.orchard_map[x][y].wkrs) == 0):
          self.swapBin(sim)
          self.plan = self.plan[1:]
        else:
            pass
      
      else:
        pass
      # self.getBin(sim.orchard_map)
      return 0
    else:
      self.status = "idle"
      return 0


  def swapBin(self, sim):
    x, y = self.loc
    if len(sim.orchard_map[x][y].bins) == 0:
      self.placeBin(sim)
    else:
      
      bin_id = self.bin
      self.placeBin(sim)
      new_bins = [item for item in sim.orchard_map[self.loc[0]][self.loc[1]].bins if item is not bin_id]
      sim.orchard_map[self.loc[0]][self.loc[1]].bins.remove(new_bins[0])
      self.bin = new_bins[0]
      sim.bins[self.bin].bot_assigned = True


  def getBin(self, sim):
    if self.bin is not None:
      print "Cannot Get bin, already got one, thanks!"
    elif len(sim.orchard_map[self.loc[0]][self.loc[1]].bins) > 0:
      self.bin = sim.orchard_map[self.loc[0]][self.loc[1]].bins[0]
      sim.orchard_map[self.loc[0]][self.loc[1]].bins.remove(self.bin)
    elif sim.orchard_map[self.loc[0]][self.loc[1]].terrain == 'depot':
      self.bin = sim.createBin(self.loc)
      sim.bins[self.bin].bot_assigned = True
      sim.orchard_map[self.loc[0]][self.loc[1]].bins.remove(self.bin)


  def placeBin(self, sim):
    if self.bin is not None:
      sim.bins[self.bin].bot_assigned = False
      sim.orchard_map[self.loc[0]][self.loc[1]].bins.append(self.bin)
      self.bin = None
      for worker in sim.orchard_map[self.loc[0]][self.loc[1]].wkrs:
        sim.wkrs[worker].request_akn = False

  def moveBot(self, action, sim):
    sim.orchard_map[self.loc[0]][self.loc[1]].bots.remove(self.bot_id)
    self.loc = map(operator.add, self.loc, action) # need to define action
    sim.orchard_map[self.loc[0]][self.loc[1]].bots.append(self.bot_id)
    if self.bin is not None:
      sim.bins[self.bin].loc = self.loc

 
  def hasBin(self):
    return self.bin is not None


class orchardBin(object):
  """docstring for orchardBin"""
  def __init__(self, loc, capacity = 1.0):
    self.loc = loc
    self.capacity = capacity #Default bin is empty, bin is full if capacity = 0
    self.bot_assigned = False

  def estimateTimeToFull(self):
    picking_rate = .45
    return self.capacity / picking_rate



class workerGroup(object):
  """docstring for workerGroup"""
  def __init__(self, loc):
    self.loc = loc
    self.efficiency = .9+random.random()*.2
    self.pickup = False
    self.delivery = False
    self.request_akn = False


  def pickupRequest(self):
    self.pickup = True

  def deliveryRequest(self):
    self.delivery = True
  

    
if __name__ == '__main__':
  # num_rows = 10
  # row_size = 10
  # num_bots = 5
  # num_bins = 30
  # num_wkrs = 5
  sim = simulator(15, 8, 4, 30, 12)
  bot = sim.bots[sim.bots.keys()[0]]
  sim.drawSimulator()
  raw_input()
  bot.plan = ["N", "S", "E", "W"]
  sim.step()
  print bot.loc
  sim.drawSimulator()
  sim.step()
  print bot.loc
  sim.drawSimulator()
  sim.step()
  print bot.loc
  sim.drawSimulator()
  sim.step()
  print bot.loc
  sim.drawSimulator()
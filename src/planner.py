import Queue, operator, random
import random

import numpy as np
import matplotlib.pyplot as plt

from utils import euclideanDist, manhattanDist
from simulator import simulator,bindog

class planner():
  def __init__ (self,sim):
    self.obstacle_map=self.init_obstacle_map(sim)
    # self.obstacle_map[0, 3] = 7
    # print self.obstacle_map
    # raw_input()
    return None

  def getPlan(self,list_rob_targets,sim):



    self.robotIDs=[a.robot_id for a in list_rob_targets]
    # print self.robotIDs
    self.robotTargets = [task.locations for task in list_rob_targets]
    # print "Targets", self.robotTargets
    self.robotTasks = [task.tasks for task in list_rob_targets]
    # print "Tasks", self.robotTasks
    #[list_rob_targets[i][1:] for i in range(len(list_rob_targets))]
    

    # print self.robotTargets
    # raw_input()

    #for every robot, make a plan with ASTAR
    set_of_robots_plan=[]

    for i, robID in enumerate(self.robotIDs):
      current_pos=sim.bots[robID].loc
      plan_string=''
      robot_plan=[]
      #for every target location
      for target_id, target in enumerate(self.robotTargets[i]):
        
        # print "current_pos: ",current_pos
        current_pos=current_pos[:2]
        goal_pos=target[:2]
        # print "target: ",target

    #     # coordinates[0],coordinates[1] = coordinates[1],coordinates[0]
    #     # current_pos[0],current_pos[1] = current_pos[1],current_pos[0]

    #     #astar return the path and length


        # print "current_pos", current_pos
        # print "goal_pos", goal_pos
        # raw_input()
        plan_coordinates,path_length= self.astar2d(current_pos, goal_pos, self.obstacle_map)[:2]
        # print plan_coordinates, path_length
        # raw_input()

        #if no errors occurred, the path length will be less than inf
        if(path_length != float('inf')):
          # print "Plan Coordinates", plan_coordinates
          # raw_input()
          #make the coordinate sequence into a cardinal direction string sequence
          plan_string=self.coordinatesToString(plan_coordinates)
          # try:
            #add pick-up or dro-off action if it exists at the end of the sequence
            # plan_string.append(target[2])
            #if the last command is pick-up, bring the robot back to the repository
          if(self.robotTasks[i][target_id]=='get'):
            plan_string.append("GET")
          elif(self.robotTasks[i][target_id]=='place'):
            plan_string.append("PLACE")
          elif(self.robotTasks[i][target_id]=='swap'):
            plan_string.append("SWAP")
            # print target
            # print plan_string
            # raw_input()
            #append W as many times as its needs to get back to the repo
            # for i in range(target[1]):
            #   plan_string.append('SOUTH')

          # except:
          #   #do nothing
          #   pass
          #for multiple target paths, we set the new starting location
          current_pos=target
          # print plan_coordinates
          # print plan_string
          #building this specific robot's path
          robot_plan+=plan_string
        else:
          print "Error occured with either target or start location of bot."
      #append the robots specific final path to the global final set of robIDs
      #and paths
      sim.bots[robID].plan = robot_plan
      sim.bots[robID].status = 'busy'
      #set_of_robots_plan.append([robID,robot_plan])
    # print set_of_robots_plan
    return set_of_robots_plan

  def coordinatesToString(self,plan):
    plan_string=[]
    curr_pos=plan[0]
    for next_pos in plan[1:]:
      if(next_pos[1]<curr_pos[1] and next_pos[0]==curr_pos[0]):
        plan_string.append('SOUTH')
      elif(next_pos[1]>curr_pos[1] and next_pos[0]==curr_pos[0]):
        plan_string.append('NORTH')
      elif(next_pos[1]==curr_pos[1] and next_pos[0]>curr_pos[0]):
        plan_string.append('EAST')
      else:
        plan_string.append('WEST')
      curr_pos=next_pos
    return plan_string


  def astar2d(self,start, goal, obstacle_map, epsilon=1):
    # obstacle_map = np.transpose(obstacle_map)
    # obstacle_map[start[0], start[1]] = 8
    # print "Planning From", start, "to", goal
    # #Error Check Trivial Paths

    # print obstacle_map
    # print self.isFree(start, obstacle_map)

    # obstacle_map[start[0], start[1]] = 0


    if not self.isFree(start, obstacle_map):
      # print "Invalid Start"
      return [[], float('Inf'), 0]
    if not self.isFree(goal, obstacle_map):
      # print "Invalid Goal"
      return [[], float('Inf'), 0]
    if start == goal:
      # print "Trivial Path"
      return [[goal], 0.0, 0]


    #Initialize Distance Matrix
    [x, y] = obstacle_map.shape 
    distances = {}

    for ii in range(x):
      for jj in range(y):
        distances[ii, jj] = float('inf')

    #Expand Start Node
    expanded_node = start
    distances[start[0], start[1]] = 0.0
    open_list = Queue.PriorityQueue()
    closed_list = []
    current_node = start
    num_expanded = 1

    for node in self.getNeighbors(start, obstacle_map):
      distances[node[0], node[1]] = min(distances[current_node[0], current_node[1]] + 1, distances[node[0], node[1]])
      if node == goal:
        #print "Astar Complete"
        #print distances[goal[0], goal[1]]
        return self.getPath(start, goal, distances, obstacle_map) + [len(closed_list)]
      scored_node = (self.f(node, goal, distances, epsilon), num_expanded, node)
      #print scored_node
      open_list.put(scored_node)



    closed_list.append(start)

    while not open_list.empty():
      current_scored_node = open_list.get(block=False)
      current_node = current_scored_node[2]


      closed_list.append(current_node)
      #print "Expanding", current_node, f(current_node, goal, distances)
      num_expanded += 1
      for node in self.getNeighbors(current_node, obstacle_map, closed_list):
        distances[node[0], node[1]] = min(distances[current_node[0], current_node[1]] + 1, distances[node[0], node[1]]) #4-connected graph assumption

        if node == goal:
          #print "Astar Complete"
          #print distances[goal[0], goal[1]]
          return self.getPath(start, goal, distances, obstacle_map) + [len(closed_list)]

        scored_node = (self.f(node, goal, distances, epsilon), num_expanded, node)
        self.updatePQueue(scored_node, open_list)
    print "astar Failed"


  def updatePQueue(self,scored_node, q):
    #Augment the Enqueue operation with ability to re-assign priority
    for q_id, item in enumerate(q.queue):
      if item[-1] == scored_node[-1]:
        break
    else:
      #If the node is not in the queue, enqueue it
      q.put(scored_node)
      return
    #Otherwise, remove it, and enqueue the lower-cost of the two
    if item[0] <= scored_node[0]:
      #Already enqueued item is better or the same as the new one, do nothing
      return
    else:
      #Remove the old item and enqueue the new one
      q.queue = q.queue[:q_id] + q.queue[q_id+1:]
      q.put(scored_node)
      return


  def getNeighbors(self,loc, obstacle_map, closed_list = []):
    #Get List of Neighbors, Ignoring ones in obstacles or ones out of bounds
    translations = [[1, 0], [-1, 0], [0, 1], [0, -1]]
    neighbors = [map(operator.add, loc, translation) for translation in translations if self.isFree(map(operator.add, loc, translation), obstacle_map)]
    return [item for item in neighbors if item not in closed_list]


  def h(self,pos, goal, epsilon = 1):
    #Heuristic function
    #return euclideanDist(pos, goal)
    return manhattanDist(pos, goal)

  def f(self,pos, goal, distances, epsilon = 1):
    # f = h + d
    return self.h(pos, goal) + distances[pos[0], pos[1]]


  def getPath(self,start, goal, distances, obstacle_map):
    #Reconstruct path from distance matrix
    path = [goal]
    path_length = distances[goal[0], goal[1]]
    current_dist = path_length
    current_node = goal
    while current_dist > 0:
      neighbors = self.getNeighbors(current_node, obstacle_map)
      best_neighbor_dist = float('inf')
      for neighbor in neighbors:
        if distances[neighbor[0], neighbor[1]] < best_neighbor_dist:
          best_neighbor = neighbor
          best_neighbor_dist = distances[neighbor[0], neighbor[1]]
      current_node = best_neighbor
      current_dist = best_neighbor_dist
      path.append(current_node)

    return [path[::-1], path_length]


  def isFree(self,loc, obstacle_map):
    #Return True if loc is free, False otherwise or if loc is outside map
    x = loc[0]
    y = loc[1]
    map_size = obstacle_map.shape
    return (x >= 0) and (x < map_size[0]) and (y >= 0) and (y < map_size[1]) and not bool(obstacle_map[x, y])

  def init_obstacle_map(self,sim):
    #Fill the orchard with trees, used for testing purposes
    orch_map=np.array(sim.orchard_map[:])
    m=np.zeros((orch_map.shape[0],orch_map.shape[1]))
    for i in range(orch_map.shape[0]):
      for j in range(orch_map.shape[1]):
        if(sim.orchard_map[i][j].terrain=="orchard"):
          m[i][j]=1
    # print m
    return m
    # print m
    # raw_input()
    # for i in range(m.shape[1]):
    #   if(orch=True)
    #   m[:,i]=
    # return m

  #def getCurrentMap from the simulator, which builds a map of 1s and 0s
  #from the state of the map
  # def getCurrentMap(self,sim):
    #Returns a map of 1s and 0s based on obstacles in the env

    #Build 1s and 0s for the terrain type
    # terrain=np.zeros((sim.orchard_map.shape[0],sim.orchard_map.shape[1]))

    





if __name__=='__main__':
  l=[[1,[1,1,'G']],[2,[10,11,'G'],[7,5,'P']],[3,[4,6]]]
  # p=planner()
  # p.getPlan(l)
  # print p.robotIDs
  # print p.robotTargets
  s=simulator(10, 20, 5, 30, 5)
  num_rows=5
  row_size=8
  p=planner(s)
  p.getPlan(l,s)
  # print s.orchard_map[0][1].terrain
  # astar2d()


  # OLD MAP INITIALIZATION CODE
 # m=np.zeros((row_size+2,num_rows*2-1))
    #make every other row full of trees
    # m[:,0::2]=np.ones((m[:,0::2].shape[0],m[:,0::2].shape[1]))
    # orch_module=np.array([1,1,0])
    # orch=np.array([])
    # for k in range(num_rows):
    #   orch=np.concatenate((orch,orch_module))
    # orch=orch[1:-2]
    # orch_module=orch#module is now a row
    # for i in range(row_size):
    #   orch=np.hstack((orch,orch_module))

#ASSUME COLLISION CHECKING AND THUS ROBOTS ARE HANDLED
    #BY THE SIMULATOR
    #build 1s and 0s for the bots in the rows
    # bot_fill_locations=(len(simulatorMap.bots)>0)
    #cancel out the bots who are in the headlands b/c we assume infinite width
    # bot_fill_locations[0]=np.zeros(len(bot_fill_locations[0]))
    # bot_fill_locations[-1]=np.zeros(len(bot_fill_locations[-1]))

    # for i in range(bot_fill_locations.shape[1]):
      # row=bot_fill_locations[:,i]
      # if(sum(row)>0):
        #if there is a bot in a row, block the entire row
        # bot_fill_locations[:,i]=np.ones(len(row))

    #finally we concantenate all of the occupancies
    # final_map=terrain+bot_fill_locations

    # final_map=terrain

    # return final_map
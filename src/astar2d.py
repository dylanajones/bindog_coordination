import Queue, operator, random
import random

import numpy as np
import matplotlib.pyplot as plt

from utils import euclideanDist, manhattanDist

def astar2d(start, goal, obstacle_map, epsilon=1):
  #print "Planning From", start, "to", goal


  #Error Check Trivial Paths
  if not isFree(start, obstacle_map):
    # print "Invalid Start"
    return [[], float('Inf'), 0]
  if not isFree(goal, obstacle_map):
    # print "Invalid Goal"
    return [[], float('Inf'), 0]
  if start == goal:
    # print "Astar Complete"
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

  for node in getNeighbors(start, obstacle_map):
    distances[node[0], node[1]] = min(distances[current_node[0], current_node[1]] + 1, distances[node[0], node[1]])
    if node == goal:
      #print "Astar Complete"
      #print distances[goal[0], goal[1]]
      return getPath(start, goal, distances, obstacle_map) + [len(closed_list)]
    scored_node = (f(node, goal, distances, epsilon), num_expanded, node)
    #print scored_node
    open_list.put(scored_node)



  closed_list.append(start)

  while not open_list.empty():
    current_scored_node = open_list.get(block=False)
    current_node = current_scored_node[2]


    closed_list.append(current_node)
    #print "Expanding", current_node, f(current_node, goal, distances)
    num_expanded += 1
    for node in getNeighbors(current_node, obstacle_map, closed_list):
      distances[node[0], node[1]] = min(distances[current_node[0], current_node[1]] + 1, distances[node[0], node[1]]) #4-connected graph assumption

      if node == goal:
        #print "Astar Complete"
        #print distances[goal[0], goal[1]]
        return getPath(start, goal, distances, obstacle_map) + [len(closed_list)]

      scored_node = (f(node, goal, distances, epsilon), num_expanded, node)
      updatePQueue(scored_node, open_list)
  print "astar Failed"


def updatePQueue(scored_node, q):
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


def getNeighbors(loc, obstacle_map, closed_list = []):
  #Get List of Neighbors, Ignoring ones in obstacles or ones out of bounds
  translations = [[1, 0], [-1, 0], [0, 1], [0, -1]]
  neighbors = [map(operator.add, loc, translation) for translation in translations if isFree(map(operator.add, loc, translation), obstacle_map)]
  return [item for item in neighbors if item not in closed_list]


def h(pos, goal, epsilon = 1):
  #Heuristic function
  #return euclideanDist(pos, goal)
  return manhattanDist(pos, goal)

def f(pos, goal, distances, epsilon = 1):
  # f = h + d
  return h(pos, goal) + distances[pos[0], pos[1]]


def getPath(start, goal, distances, obstacle_map):
  #Reconstruct path from distance matrix
  path = [goal]
  path_length = distances[goal[0], goal[1]]
  current_dist = path_length
  current_node = goal
  while current_dist > 0:
    neighbors = getNeighbors(current_node, obstacle_map)
    best_neighbor_dist = float('inf')
    for neighbor in neighbors:
      if distances[neighbor[0], neighbor[1]] < best_neighbor_dist:
        best_neighbor = neighbor
        best_neighbor_dist = distances[neighbor[0], neighbor[1]]
    current_node = best_neighbor
    current_dist = best_neighbor_dist
    path.append(current_node)

  return [path[::-1], path_length]


def isFree(loc, obstacle_map):
  #Return True if loc is free, False otherwise or if loc is outside map
  x = loc[0]
  y = loc[1]
  map_size = obstacle_map.shape
  return (x >= 0) and (x < map_size[0]) and (y >= 0) and (y < map_size[1]) and not bool(obstacle_map[y, x])

def init_map(num_rows,row_size):
  #Fill the orchard with trees, used for testing purposes
  m=np.zeros((row_size+2,num_rows*2-1))
  # m=np.arange(((num_rows*2)*(row_size+2)))
  # m=m.reshape((num_rows*2,row_size+2))

  #make every other row full of trees
  m[:,0::2]=np.ones((m[:,0::2].shape[0],m[:,0::2].shape[1]))
  return m

#def getCurrentMap from the simulator, which builds a map of 1s and 0s
#from the state of the map
def getCurrentMap(simulatorMap):
  #Returns a map of 1s and 0s based on obstacles in the env
  #Build 1s and 0s for the terrain type
  terrain=(simulatorMap.terrain_type<=0)
  #build 1s and 0s for the bots in the rows
  bot_fill_locations=(len(simulatorMap.bots)>0)
  #cancel out the bots who are in the headlands b/c we assume infinite width
  bot_fill_locations[0]=np.zeros(len(bot_fill_locations[0]))
  bot_fill_locations[-1]=np.zeros(len(bot_fill_locations[-1]))

  for i in range(bot_fill_locations.shape[1]):
    row=bot_fill_locations[:,i]
    if(sum(row)>0):
      #if there is a bot in a row, block the entire row
      bot_fill_locations[:,i]=np.ones(len(row))

  #finally we concantenate all of the occupancies
  final_map=terrain+bot_fill_locations

  return final_map





if __name__=='__main__':
  init_map=init_map(5,8)

  # astar2d()